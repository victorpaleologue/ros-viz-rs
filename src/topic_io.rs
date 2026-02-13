//! Automatic ROS 2 subscription / publication for discovered topics.
//!
//! When a [`TopicInfo`] entity appears in the world, this module:
//!
//! 1. Creates a ROS 2 **subscriber** so we can display the latest value, and
//! 2. Creates a ROS 2 **publisher** so the user can send values from the UI.
//!
//! Currently only `std_msgs/String` is handled – other types are silently
//! skipped.  Adding a new type only requires extending the `match` arms in
//! [`setup_subscription`] and [`setup_publisher`].
//!
//! # ECS layout
//!
//! | Component | Meaning |
//! |---|---|
//! | [`TopicLatestValue`] | Latest value received from the subscription |
//! | [`TopicEditBuffer`]  | Text the user is typing in the egui field |
//! | [`HasSubscription`]  | Marker: a subscription was created |
//! | [`HasPublisher`]     | Marker: a publisher was created |
//!
//! The actual `ros2_client` subscription / publisher handles live inside the
//! [`RosNode`] resource (behind `Arc<Mutex<…>>`), keyed by topic name.

use std::collections::{HashMap, HashSet};
use std::sync::{Arc, Mutex};

use bevy::prelude::*;
use ros2_client::{
    DEFAULT_PUBLISHER_QOS, DEFAULT_SUBSCRIPTION_QOS, Name, Node, NodeName, NodeOptions,
};

use crate::ros_systems::RosContext;
use crate::ros2_msgs::{self, MessageType};
use crate::topics_view::{TopicDataSource, TopicInfo, TopicKind};

// ---------------------------------------------------------------------------
// Components
// ---------------------------------------------------------------------------

/// Latest value received from a ROS 2 subscription, as a display string.
#[derive(Component, Default, Debug, Clone)]
pub struct TopicLatestValue(pub String);

/// Editing buffer bound to the egui text input.
#[derive(Component, Default, Debug, Clone)]
pub struct TopicEditBuffer(pub String);

/// Marker: we have an active subscription for this topic entity.
#[derive(Component, Debug)]
pub struct HasSubscription;

/// Marker: we have an active publisher for this topic entity.
#[derive(Component, Debug)]
pub struct HasPublisher;

// ---------------------------------------------------------------------------
// Resource: ROS 2 Node + I/O handles
// ---------------------------------------------------------------------------

/// Bevy resource owning the local ROS 2 node and the communication channels
/// used to exchange data between background subscription threads and Bevy
/// systems.
#[derive(Resource)]
pub struct RosNode {
    /// The local node, behind a mutex so background threads can't race with
    /// the main thread during topic/subscription/publisher creation. After
    /// creation the handles are moved to their own threads.
    node: Arc<Mutex<Node>>,

    /// Receivers for subscription values, keyed by DDS topic name.
    ///
    /// Each receiver delivers `String` payloads produced by a dedicated
    /// background polling thread.  Wrapped in `Mutex` so the resource is
    /// `Sync` (required by Bevy).
    sub_receivers: HashMap<String, Mutex<std::sync::mpsc::Receiver<String>>>,

    /// Senders for publishing, keyed by DDS topic name.
    ///
    /// Writing a `String` into the sender causes the corresponding background
    /// thread to publish it on the ROS 2 topic.
    pub_senders: HashMap<String, std::sync::mpsc::SyncSender<String>>,

    /// Set of topic names we already attempted to set up (avoid retrying on
    /// every frame).
    managed_topics: HashSet<String>,
}

impl RosNode {
    /// Create a new local ROS 2 node named `ros_viz_rs` under the root
    /// namespace.
    pub fn new(context: &RosContext) -> Result<Self, String> {
        let node_name =
            NodeName::new("/", "ros_viz_rs").map_err(|e| format!("bad node name: {e:?}"))?;
        let node = context
            .0
            .new_node(node_name, NodeOptions::new().enable_rosout(false))
            .map_err(|e| format!("failed to create node: {e:?}"))?;
        Ok(Self {
            node: Arc::new(Mutex::new(node)),
            sub_receivers: HashMap::new(),
            pub_senders: HashMap::new(),
            managed_topics: HashSet::new(),
        })
    }
}

// ---------------------------------------------------------------------------
// Systems
// ---------------------------------------------------------------------------

/// Initialise the [`RosNode`] resource from the [`RosContext`] if it doesn't
/// exist yet.
pub fn init_ros_node(world: &mut World) {
    if world.get_resource::<RosNode>().is_some() {
        return;
    }
    let Some(ctx) = world.get_resource::<RosContext>() else {
        return;
    };
    let ctx = ctx.clone();
    match RosNode::new(&ctx) {
        Ok(ros_node) => {
            tracing::info!("RosNode created");
            world.insert_resource(ros_node);
        }
        Err(e) => {
            tracing::warn!("Failed to create RosNode: {e}");
        }
    }
}

/// For every [`TopicInfo`] entity that we haven't handled yet, set up a
/// subscription and/or publisher depending on the message type.
pub fn auto_manage_topics(
    mut commands: Commands,
    ros_node: Option<ResMut<RosNode>>,
    new_topics: Query<
        (Entity, &TopicInfo),
        (
            With<TopicDataSource>,
            Without<HasSubscription>,
            Without<HasPublisher>,
        ),
    >,
) {
    let Some(mut ros_node) = ros_node else { return };

    // Destructure to allow simultaneous borrows of different fields.
    let RosNode {
        ref node,
        ref mut sub_receivers,
        ref mut pub_senders,
        ref mut managed_topics,
    } = *ros_node;

    for (entity, info) in new_topics.iter() {
        if managed_topics.contains(&info.topic_name) {
            continue;
        }

        // Only handle Normal topics for now.
        let ros_topic_name = match &info.kind {
            TopicKind::Normal(clean) => clean.clone(),
            _ => continue,
        };

        // --- subscription ---
        let sub_ok = setup_subscription(
            node,
            &info.topic_name,
            &ros_topic_name,
            &info.type_name,
            sub_receivers,
        );

        // --- publisher ---
        let pub_ok = setup_publisher(
            node,
            &info.topic_name,
            &ros_topic_name,
            &info.type_name,
            pub_senders,
        );

        managed_topics.insert(info.topic_name.clone());

        let mut entity_commands = commands.entity(entity);
        if sub_ok {
            entity_commands.insert((HasSubscription, TopicLatestValue::default()));
        }
        if pub_ok {
            entity_commands.insert((HasPublisher, TopicEditBuffer::default()));
        }
    }
}

/// Drain subscription receivers and update [`TopicLatestValue`] on the
/// corresponding entities.
pub fn poll_subscription_values(
    ros_node: Option<Res<RosNode>>,
    mut topics: Query<(&TopicInfo, &mut TopicLatestValue), With<HasSubscription>>,
) {
    let Some(ros_node) = ros_node else { return };
    for (info, mut latest) in topics.iter_mut() {
        if let Some(rx_mutex) = ros_node.sub_receivers.get(&info.topic_name) {
            let rx = rx_mutex.lock().unwrap();
            // Drain – keep only the most recent value.
            while let Ok(msg) = rx.try_recv() {
                latest.0 = msg;
            }
        }
    }
}

/// Publish the contents of [`TopicEditBuffer`] when signalled by the UI.
///
/// The UI sets `TopicEditBuffer` and then inserts a [`PublishRequest`] marker
/// component.  This system picks those up, sends the value, and removes the
/// marker.
pub fn handle_publish_requests(
    mut commands: Commands,
    ros_node: Option<Res<RosNode>>,
    requests: Query<(Entity, &TopicInfo, &TopicEditBuffer), With<PublishRequest>>,
) {
    let Some(ros_node) = ros_node else { return };
    for (entity, info, buf) in requests.iter() {
        if let Some(tx) = ros_node.pub_senders.get(&info.topic_name) {
            if let Err(e) = tx.try_send(buf.0.clone()) {
                tracing::warn!("Failed to publish on '{}': {e}", info.topic_name);
            }
        }
        commands.entity(entity).remove::<PublishRequest>();
    }
}

/// Temporary marker inserted by the UI to request a publish.
#[derive(Component, Debug)]
pub struct PublishRequest;

// ---------------------------------------------------------------------------
// Plugin
// ---------------------------------------------------------------------------

/// Bevy plugin that manages automatic ROS 2 subscriptions and publishers.
pub struct TopicIOPlugin;

impl Plugin for TopicIOPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Update, init_ros_node);
        app.add_systems(
            Update,
            (
                auto_manage_topics,
                poll_subscription_values,
                handle_publish_requests,
            )
                .chain()
                .after(init_ros_node),
        );
    }
}

// ---------------------------------------------------------------------------
// Subscription / publisher setup helpers
// ---------------------------------------------------------------------------

/// Create a subscription for a supported message type and spawn a background
/// polling thread.  Returns `true` on success.
fn setup_subscription(
    node: &Arc<Mutex<Node>>,
    dds_topic_name: &str,
    ros_topic_name: &str,
    type_name: &str,
    receivers: &mut HashMap<String, Mutex<std::sync::mpsc::Receiver<String>>>,
) -> bool {
    match type_name {
        ros2_msgs::String::MESSAGE_TYPE_STR => setup_typed_subscription::<ros2_msgs::String>(
            node,
            dds_topic_name,
            ros_topic_name,
            receivers,
            |msg| msg.data,
        ),
        // Add more types here:
        // ros2_msgs::Int32::MESSAGE_TYPE_STR => { ... }
        _ => {
            tracing::debug!("No subscription handler for type '{type_name}'");
            false
        }
    }
}

/// Generic subscription setup: creates the DDS topic + subscription, spawns a
/// polling thread that converts each received message to a `String` via `to_string`.
fn setup_typed_subscription<T: MessageType>(
    node: &Arc<Mutex<Node>>,
    dds_topic_name: &str,
    ros_topic_name: &str,
    receivers: &mut HashMap<String, Mutex<std::sync::mpsc::Receiver<String>>>,
    to_string: fn(T) -> String,
) -> bool {
    let ros_name = match Name::parse(ros_topic_name) {
        Ok(n) => n,
        Err(e) => {
            tracing::warn!("Invalid ROS topic name '{ros_topic_name}': {e:?}");
            return false;
        }
    };

    let mut node_guard = node.lock().unwrap();
    let topic =
        match node_guard.create_topic(&ros_name, T::message_type_name(), &DEFAULT_SUBSCRIPTION_QOS)
        {
            Ok(t) => t,
            Err(e) => {
                tracing::warn!("create_topic failed for '{ros_topic_name}': {e:?}");
                return false;
            }
        };
    let subscription = match node_guard.create_subscription::<T>(&topic, None) {
        Ok(s) => s,
        Err(e) => {
            tracing::warn!("create_subscription failed for '{ros_topic_name}': {e:?}");
            return false;
        }
    };
    drop(node_guard); // release lock before spawning thread

    let (tx, rx) = std::sync::mpsc::channel::<String>();
    let topic_name_owned = dds_topic_name.to_owned();

    std::thread::Builder::new()
        .name(format!("sub:{topic_name_owned}"))
        .spawn(move || {
            loop {
                match subscription.take() {
                    Ok(Some((msg, _info))) => {
                        if tx.send(to_string(msg)).is_err() {
                            break; // receiver dropped – entity despawned
                        }
                    }
                    Ok(None) => {
                        std::thread::sleep(std::time::Duration::from_millis(50));
                    }
                    Err(e) => {
                        tracing::warn!("Subscription take error on '{topic_name_owned}': {e:?}");
                        std::thread::sleep(std::time::Duration::from_millis(200));
                    }
                }
            }
            tracing::debug!("Subscription thread for '{topic_name_owned}' exiting");
        })
        .expect("Failed to spawn subscription thread");

    receivers.insert(dds_topic_name.to_owned(), Mutex::new(rx));
    true
}

/// Create a publisher for a supported message type and spawn a background
/// publishing thread.  Returns `true` on success.
fn setup_publisher(
    node: &Arc<Mutex<Node>>,
    dds_topic_name: &str,
    ros_topic_name: &str,
    type_name: &str,
    senders: &mut HashMap<String, std::sync::mpsc::SyncSender<String>>,
) -> bool {
    match type_name {
        ros2_msgs::String::MESSAGE_TYPE_STR => setup_typed_publisher::<ros2_msgs::String>(
            node,
            dds_topic_name,
            ros_topic_name,
            senders,
            |s| ros2_msgs::String { data: s },
        ),
        // Add more types here.
        _ => {
            tracing::debug!("No publisher handler for type '{type_name}'");
            false
        }
    }
}

/// Generic publisher setup: creates the DDS topic + publisher, spawns a
/// background thread that reads `String` payloads from a channel and publishes
/// them as typed messages via `from_string`.
fn setup_typed_publisher<T: MessageType + serde::Serialize>(
    node: &Arc<Mutex<Node>>,
    dds_topic_name: &str,
    ros_topic_name: &str,
    senders: &mut HashMap<String, std::sync::mpsc::SyncSender<String>>,
    from_string: fn(String) -> T,
) -> bool {
    let ros_name = match Name::parse(ros_topic_name) {
        Ok(n) => n,
        Err(e) => {
            tracing::warn!("Invalid ROS topic name '{ros_topic_name}': {e:?}");
            return false;
        }
    };

    let mut node_guard = node.lock().unwrap();
    let topic =
        match node_guard.create_topic(&ros_name, T::message_type_name(), &DEFAULT_PUBLISHER_QOS) {
            Ok(t) => t,
            Err(e) => {
                tracing::warn!("create_topic failed for '{ros_topic_name}': {e:?}");
                return false;
            }
        };
    let publisher = match node_guard.create_publisher::<T>(&topic, None) {
        Ok(p) => p,
        Err(e) => {
            tracing::warn!("create_publisher failed for '{ros_topic_name}': {e:?}");
            return false;
        }
    };
    drop(node_guard);

    // Bounded channel – if the consumer (publisher thread) can't keep up we
    // drop old messages rather than blocking the Bevy frame.
    let (tx, rx) = std::sync::mpsc::sync_channel::<String>(4);
    let topic_name_owned = dds_topic_name.to_owned();

    std::thread::Builder::new()
        .name(format!("pub:{topic_name_owned}"))
        .spawn(move || {
            for payload in rx {
                let msg = from_string(payload);
                if let Err(e) = publisher.publish(msg) {
                    tracing::warn!("Publish error on '{topic_name_owned}': {e:?}");
                }
            }
            tracing::debug!("Publisher thread for '{topic_name_owned}' exiting");
        })
        .expect("Failed to spawn publisher thread");

    senders.insert(dds_topic_name.to_owned(), tx);
    true
}
