//! Automatic ROS 2 subscription / publication for discovered topics.
//!
//! When a topic is discovered as subscribable (`ReadersAndWriters::is_subscribable`), this module
//! creates a ROS 2 **subscriber** and attaches [`Subscription`] + [`TopicValue`] components.
//! When a topic is discovered as publishable (`ReadersAndWriters::is_publishable`), it creates a
//! ROS 2 **publisher** and attaches [`Publisher`] + [`TopicEdit`] components.
//!
//! Message types are resolved through the [`MessageRegistry`] resource:
//! values flow as reflected [`serde_json::Value`] trees, so this module knows
//! nothing about concrete message structs. Adding a new type only requires
//! registering it in [`MessageRegistry::standard`].
//!
//! # ECS layout
//!
//! | Component | Meaning |
//! |---|---|
//! | [`TopicInfo`] | Discovered topic, with name and type |
//! | [`ReadersAndWriters`] | Readers/writers already present on the network for this topic |
//! | [`Subscription`] | Type-erased subscription we set up when a topic is subscribable |
//! | [`Publisher`] | Type-erased publisher we set up when a topic is publishable |
//! | [`TopicValue`] | Latest reflected value received from our subscription |
//! | [`TopicEdit`] | Reflected value the user is editing in the egui widgets |
//!
//! The actual `ros2_client` subscription / publisher handles live inside the
//! type-erased boxes; the node itself sits in the [`RosSession`] resource
//! (behind `Arc<Mutex<…>>`).
use std::sync::{Arc, Mutex};

use bevy::prelude::*;

use ros2_client::{DEFAULT_PUBLISHER_QOS, DEFAULT_SUBSCRIPTION_QOS, Name, Node};
use rustdds::QosPolicies;

use crate::messages::MessageRegistry;
use crate::ros_msgs::MessageType;
use crate::ros_plugin::{ReadersAndWriters, RosSession, TopicInfo, TopicKind};

// ---------------------------------------------------------------------------
// Plugin
// ---------------------------------------------------------------------------
/// Bevy plugin that manages automatic ROS 2 subscriptions and publishers,
/// supposing there already a `RosSession` running and discovering topics.
pub struct TopicIOPlugin;

impl Plugin for TopicIOPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<MessageRegistry>();
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

pub use crate::topics::{
    PublishRequest, Publisher, Subscription, TopicEdit, TopicValue, handle_publish_requests,
    poll_subscription_values,
};

// ---------------------------------------------------------------------------
// Systems
// ---------------------------------------------------------------------------
/// For every subscribable [`TopicInfo`] entity without a [`Subscription`],
/// set up a subscription.  For every publishable entity without a
/// [`Publisher`], set up a publisher.  Types absent from the
/// [`MessageRegistry`] are skipped (the UI reports them as unsupported).
#[allow(clippy::type_complexity)]
pub(crate) fn auto_manage_topics(
    mut commands: Commands,
    ros_session: ResMut<RosSession>,
    registry: Res<MessageRegistry>,
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
        if !registry.contains(&info.type_name) {
            tracing::debug!("No registered handler for type '{}'", info.type_name);
            continue;
        }

        // No publisher setup yet.
        if rs_and_ws.is_publishable() && has_publisher.is_none() {
            match registry.make_publisher(&info.type_name, node, ros_topic_name, None) {
                Ok(publisher) => {
                    let seed = registry
                        .default_value(&info.type_name)
                        .unwrap_or(serde_json::Value::Null);
                    commands
                        .entity(entity)
                        .insert((Publisher(publisher), TopicEdit(seed)));
                }
                Err(e) => {
                    tracing::error!("Failed to setup publisher for '{ros_topic_name}': {e}");
                }
            }
        }

        // No subscription setup yet.
        if rs_and_ws.is_subscribable() && has_subscription.is_none() {
            match registry.subscribe(&info.type_name, node, ros_topic_name, None) {
                Ok(subscription) => {
                    commands
                        .entity(entity)
                        .insert((Subscription(subscription), TopicValue::default()));
                }
                Err(e) => {
                    tracing::error!("Failed to setup subscription for '{ros_topic_name}': {e}");
                }
            }
        }

        // Topic not publishable anymore.
        if !rs_and_ws.is_publishable() && has_publisher.is_some() {
            commands
                .entity(entity)
                .remove::<Publisher>()
                .remove::<TopicEdit>();
        }

        // Topic not subscribable anymore.
        if !rs_and_ws.is_subscribable() && has_subscription.is_some() {
            commands
                .entity(entity)
                .remove::<Subscription>()
                .remove::<TopicValue>();
        }
    }
}

// ---------------------------------------------------------------------------
// Subscription / publisher setup helpers
// ---------------------------------------------------------------------------

/// Typed subscription setup: creates the DDS topic + subscription for a
/// concrete message type `T`.
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

/// Typed publisher setup: creates the DDS topic + publisher for a concrete
/// message type `T`.
pub fn setup_typed_publisher<T: MessageType>(
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

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::messages::{DynPublisher, DynSubscription};
    use serde_json::{Value, json};
    use std::sync::Mutex as StdMutex;

    /// Fake subscription returning a queued list of values, newest last.
    struct FakeSubscription(StdMutex<Vec<Value>>);

    impl DynSubscription for FakeSubscription {
        fn poll(&self) -> Option<Value> {
            self.0.lock().unwrap().pop()
        }
    }

    /// Fake publisher recording every published value.
    struct FakePublisher(Arc<StdMutex<Vec<Value>>>);

    impl DynPublisher for FakePublisher {
        fn publish(&self, value: &Value) -> Result<(), String> {
            self.0.lock().unwrap().push(value.clone());
            Ok(())
        }
    }

    fn test_topic_info() -> TopicInfo {
        TopicInfo::new(
            "rt/chatter",
            "std_msgs/String",
            TopicKind::Normal("/chatter".into()),
        )
    }

    #[test]
    fn poll_updates_topic_value() {
        let mut app = App::new();
        app.add_systems(Update, poll_subscription_values);

        let subscription = Subscription(Box::new(FakeSubscription(StdMutex::new(vec![
            json!({"data": "hello"}),
        ]))));
        let entity = app
            .world_mut()
            .spawn((subscription, TopicValue::default()))
            .id();

        app.update();
        assert_eq!(
            app.world().get::<TopicValue>(entity),
            Some(&TopicValue(Some(json!({"data": "hello"}))))
        );

        // No new message: the latest value must be kept, not cleared.
        app.update();
        assert_eq!(
            app.world().get::<TopicValue>(entity),
            Some(&TopicValue(Some(json!({"data": "hello"}))))
        );
    }

    #[test]
    fn publish_request_publishes_edit_buffer_and_clears_marker() {
        let mut app = App::new();
        app.add_systems(Update, handle_publish_requests);

        let published = Arc::new(StdMutex::new(Vec::new()));
        let entity = app
            .world_mut()
            .spawn((
                test_topic_info(),
                TopicEdit(json!({"data": "to publish"})),
                Publisher(Box::new(FakePublisher(published.clone()))),
                PublishRequest,
            ))
            .id();

        app.update();
        assert_eq!(
            published.lock().unwrap().as_slice(),
            &[json!({"data": "to publish"})]
        );
        assert!(
            app.world().get::<PublishRequest>(entity).is_none(),
            "PublishRequest marker must be removed"
        );

        // Without a new request, nothing further is published.
        app.update();
        assert_eq!(published.lock().unwrap().len(), 1);
    }

    #[test]
    fn publish_without_request_does_nothing() {
        let mut app = App::new();
        app.add_systems(Update, handle_publish_requests);

        let published = Arc::new(StdMutex::new(Vec::new()));
        app.world_mut().spawn((
            test_topic_info(),
            TopicEdit(json!({"data": "idle"})),
            Publisher(Box::new(FakePublisher(published.clone()))),
        ));

        app.update();
        assert!(published.lock().unwrap().is_empty());
    }
}
