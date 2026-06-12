//! Automatic ROS 2 subscription / publication for discovered topics.
//!
//! When a topic is discovered as subscribable ([`ReadersAndWriters::is_subscribable`]), this module
//! creates a ROS 2 **subscriber** and attaches a [`TopicLatestValue`] component.
//! When a topic is discovered as publishable ([`ReadersAndWriters::is_publishable`]), it creates a
//! ROS 2 **publisher** and attaches a [`TopicEditBuffer`] component.
//!
//! Currently only `std_msgs/String` is handled – other types are silently
//! skipped.  Adding a new type only requires extending the `match` arms in
//! [`setup_subscription`] and [`setup_publisher`].
//!
//! # ECS layout
//!
//! | Component | Meaning |
//! |---|---|
//! | [`TopicInfo`] | Discovered topic, with name and type |
//! | [`ReadersAndWriters`] | Readers/writers already present on the network for this topic |
//! | [`Subscription`]   | Subscription we set up when a topic is subscribable |
//! | [`Publisher`]    | Publisher we set up when a topic is publishable |
//! | [`TopicLatestValue`] | Latest value received from our subscription |
//! | [`TopicEditBuffer`]  | Text the user is typing in the egui field |
//!
//! The presence of [`TopicLatestValue`] or [`TopicEditBuffer`] on an entity
//! means it has been set up ("managed").
//!
//! The actual `ros2_client` subscription / publisher handles live inside the
//! [`RosNode`] resource (behind `Arc<Mutex<…>>`), keyed by topic name.
use std::sync::{Arc, Mutex};

use bevy::prelude::*;

use ros2_client::{DEFAULT_PUBLISHER_QOS, DEFAULT_SUBSCRIPTION_QOS, Name, Node};
use rustdds::QosPolicies;

use crate::ros_msgs::{self, MessageType};
use crate::ros_plugin::{ReadersAndWriters, RosSession, TopicInfo, TopicKind};

// ---------------------------------------------------------------------------
// Plugin
// ---------------------------------------------------------------------------
/// Bevy plugin that manages automatic ROS 2 subscriptions and publishers,
/// supposing there already a `RosSession` running and discovering topics.
pub struct TopicIOPlugin;

impl Plugin for TopicIOPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(
            Update,
            (
                auto_manage_topics,
                poll_subscription_values,
                handle_publish_requests,
            )
                .chain(),
        );
    }
}

// ---------------------------------------------------------------------------
// Components
// ---------------------------------------------------------------------------

/// Latest value received from a ROS 2 subscription, as a display string.
#[derive(Component, Debug, Clone, Default, PartialEq, Eq)]
pub enum TopicLatestValue {
    #[default]
    None,
    String(String),
    // Add more types here.
}

/// Editing buffer bound to the egui text input.
#[derive(Component, Default, Debug, Clone)]
pub enum TopicEditBuffer {
    #[default]
    None,
    String(String),
}

#[derive(Component)]
pub enum Subscription {
    String(ros2_client::Subscription<ros_msgs::String>), // Add more types here.
}

impl std::fmt::Debug for Subscription {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Subscription::String(_) => f
                .debug_tuple("Subscription::String")
                .field(&"<ros2_client::Subscription<ros2_msgs::String>>")
                .finish(),
            // Add more types here.
        }
    }
}

#[derive(Component)]
pub enum Publisher {
    String(ros2_client::Publisher<ros_msgs::String>), // Add more types here.
}

impl std::fmt::Debug for Publisher {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Publisher::String(_) => f
                .debug_tuple("Publisher::String")
                .field(&"<ros2_client::Publisher<ros2_msgs::String>>")
                .finish(),
            // Add more types here.
        }
    }
}

// ---------------------------------------------------------------------------
// Systems
// ---------------------------------------------------------------------------
/// For every subscribable [`TopicInfo`] entity without a [`TopicLatestValue`],
/// set up a subscription.  For every publishable entity without a
/// [`TopicEditBuffer`], set up a publisher.
#[allow(clippy::type_complexity)]
pub(crate) fn auto_manage_topics(
    mut commands: Commands,
    ros_session: ResMut<RosSession>,
    topics: Query<(
        Entity,
        &TopicInfo,
        &ReadersAndWriters,
        Option<&Subscription>,
        Option<&Publisher>,
    )>,
) {
    let node = &ros_session.node;
    for (entity, info, rs_and_ws, has_subscription, has_publisher) in topics.iter() {
        let TopicKind::Normal(ros_topic_name) = &info.kind else {
            continue; // only manage "normal" topics for now
        };

        // No publisher setup yet.
        if rs_and_ws.is_publishable() && has_publisher.is_none() {
            setup_publisher(&mut commands, entity, node, ros_topic_name, &info.type_name);
            commands.entity(entity).insert(TopicEditBuffer::default());
        }

        // No subscription setup yet.
        if rs_and_ws.is_subscribable() && has_subscription.is_none() {
            setup_subscription(&mut commands, entity, node, ros_topic_name, &info.type_name);
        }

        // Topic not publishable anymore.
        if !rs_and_ws.is_publishable() && has_publisher.is_some() {
            commands
                .entity(entity)
                .remove::<Publisher>()
                .remove::<TopicEditBuffer>();
        }

        // Topic not subscribable anymore.
        if !rs_and_ws.is_subscribable() && has_subscription.is_some() {
            commands
                .entity(entity)
                .remove::<Subscription>()
                .remove::<TopicLatestValue>();
        }
    }
}

/// Drain subscription receivers and update [`TopicLatestValue`] on the
/// corresponding entities.
pub fn poll_subscription_values(mut topics: Query<(&Subscription, &mut TopicLatestValue)>) {
    for (subscription, mut latest) in topics.iter_mut() {
        match subscription {
            Subscription::String(handle) => {
                while let Ok(Some((msg, _))) = handle.take() {
                    *latest = TopicLatestValue::String(msg.data);
                }
            } // Add more types here.
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
    mut requests: Query<
        (Entity, &TopicInfo, &TopicEditBuffer, &mut Publisher),
        With<PublishRequest>,
    >,
) {
    for (entity, info, buf, mut publisher) in requests.iter_mut() {
        match (&mut *publisher, buf) {
            (Publisher::String(publisher), TopicEditBuffer::String(s)) => {
                let msg = ros_msgs::String { data: s.clone() };
                if let Err(e) = publisher.publish(msg) {
                    tracing::error!("Failed to publish on '{}': {e}", info.topic_name);
                }
            }
            // Add more types here.
            // Cover mismatches between publisher type and buffer type, and report them as errors.
            (_, _) => {
                tracing::error!(
                    "Topic '{}' has a publisher of type {:?} but the edit buffer is of type {:?}",
                    info.topic_name,
                    buf,
                    publisher
                );
            }
        }
        commands.entity(entity).remove::<PublishRequest>();
    }
}

/// Temporary marker inserted by the UI to request a publish.
#[derive(Component, Debug)]
pub struct PublishRequest;

// ---------------------------------------------------------------------------
// Subscription / publisher setup helpers
// ---------------------------------------------------------------------------
/// Create a subscription for a supported message type and spawn a background
/// polling thread.  Returns `true` on success.
fn setup_subscription(
    commands: &mut Commands,
    entity: Entity,
    node: &Arc<Mutex<Node>>,
    ros_topic_name: &str,
    type_name: &str,
) {
    let subscription_res = match type_name {
        ros_msgs::String::MESSAGE_TYPE_STR => {
            setup_typed_subscription::<ros_msgs::String>(node, ros_topic_name, None)
                .map(Subscription::String)
        }
        // Add more types here:
        // ros2_msgs::Int32::MESSAGE_TYPE_STR => { ... }
        _ => {
            tracing::debug!("No subscription handler for type '{type_name}'");
            return;
        }
    };

    if let Ok(subscription) = subscription_res {
        commands.entity(entity).insert(subscription);
    } else if let Err(e) = subscription_res {
        tracing::error!(
            "Failed to setup subscription for '{}': {}",
            ros_topic_name,
            e
        );
    }
}

/// Generic subscription setup: creates the DDS topic + subscription, spawns a
/// polling thread that converts each received message to a `String` via `to_string`.
pub fn setup_typed_subscription<T: MessageType>(
    node: &Arc<Mutex<Node>>,
    ros_topic_name: &str,
    qos: Option<&QosPolicies>,
) -> Result<ros2_client::Subscription<T>, String> {
    let ros_name = Name::parse(ros_topic_name)
        .map_err(|e| format!("Invalid ROS topic name '{ros_topic_name}': {e:?}"))?;
    let mut node = node.lock().unwrap();
    let topic = node
        .create_topic(
            &ros_name,
            T::message_type_name(),
            qos.unwrap_or(&DEFAULT_SUBSCRIPTION_QOS),
        )
        .map_err(|e| format!("create_topic failed: {e:?}"))?;
    node.create_subscription::<T>(&topic, None)
        .map_err(|e| format!("create_subscription failed: {e:?}"))
}

/// Create a publisher for a supported message type and spawn a background
/// publishing thread.  Returns `true` on success.
fn setup_publisher(
    commands: &mut Commands,
    entity: Entity,
    node: &Arc<Mutex<Node>>,
    ros_topic_name: &str,
    type_name: &str,
) {
    let publisher_res = match type_name {
        ros_msgs::String::MESSAGE_TYPE_STR => {
            setup_typed_publisher::<ros_msgs::String>(node, ros_topic_name, None)
                .map(Publisher::String)
        }
        // Add more types here.
        _ => {
            tracing::debug!("No publisher handler for type '{type_name}'");
            return;
        }
    };
    if let Ok(publisher) = publisher_res {
        commands.entity(entity).insert(publisher);
    } else if let Err(e) = publisher_res {
        tracing::error!("Failed to setup publisher for '{}': {}", ros_topic_name, e);
    }
}

/// Generic publisher setup: creates the DDS topic + publisher, spawns a
/// background thread that reads `String` payloads from a channel and publishes
/// them as typed messages via `from_string`.
pub fn setup_typed_publisher<T: MessageType + serde::Serialize>(
    node: &Arc<Mutex<Node>>,
    ros_topic_name: &str,
    qos: Option<&QosPolicies>,
) -> Result<ros2_client::Publisher<T>, String> {
    let ros_name = Name::parse(ros_topic_name)
        .map_err(|e| format!("Invalid ROS topic name '{ros_topic_name}': {e:?}"))?;
    let mut node = node.lock().unwrap();
    let topic = node
        .create_topic(
            &ros_name,
            T::message_type_name(),
            qos.unwrap_or(&DEFAULT_PUBLISHER_QOS),
        )
        .map_err(|e| format!("create_topic failed for '{ros_topic_name}': {e:?}"))?;
    node.create_publisher::<T>(&topic, None)
        .map_err(|e| format!("create_publisher failed for '{ros_topic_name}': {e:?}"))
}
