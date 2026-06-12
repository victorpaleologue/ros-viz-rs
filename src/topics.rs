//! Transport-agnostic topic model.
//!
//! Every connection backend (DDS via [`crate::ros_plugin`], rosbridge via
//! [`crate::rosbridge`]) discovers topics into entities carrying
//! [`TopicInfo`], attaches [`Subscription`]/[`Publisher`] handles and a
//! [`TopicValue`]/[`TopicEdit`] pair, and honors [`PublishRequest`] markers.
//! The UI ([`crate::topics_view`]) and any other consumer only ever touch
//! these types — that is the seam that keeps transports swappable.

use bevy::prelude::*;

use crate::messages::{DynPublisher, DynSubscription};

/// Classification of a topic based on ROS 2's DDS naming conventions:
///
/// - `rt/…` — regular pub/sub topics
/// - `rq/…` — service request channels
/// - `rr/…` — service reply channels
/// - `rt/…/_action/…` — action-related topics
/// - anything else — unknown / internal
#[derive(Debug, Clone, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub enum TopicKind {
    /// A regular pub/sub topic, carrying the clean ROS topic name.
    Normal(String),
    /// A service request channel, carrying the service name.
    ServiceRequest(String),
    /// A service reply channel, carrying the service name.
    ServiceReply(String),
    /// An action-related topic, carrying the action name.
    Action(String),
    /// Anything that doesn't match a known pattern.
    Unknown,
}

/// A Bevy component that represents a single topic.
#[derive(Component, Debug, Clone, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub struct TopicInfo {
    /// The full topic name as the transport reports it
    /// (raw DDS name like `rt/joint_states`, or the plain ROS name).
    pub topic_name: String,

    /// The data type name of the topic, e.g. `sensor_msgs/JointState`.
    pub type_name: String,

    /// The kind of topic (normal, service, action, …).
    pub kind: TopicKind,
}

impl TopicInfo {
    pub fn new(name: impl Into<String>, type_name: impl Into<String>, kind: TopicKind) -> Self {
        Self {
            topic_name: name.into(),
            type_name: type_name.into(),
            kind,
        }
    }
}

/// Latest value received on a subscribed topic, reflected to JSON.
#[derive(Component, Debug, Clone, Default, PartialEq)]
pub struct TopicValue(pub Option<serde_json::Value>);

/// The value being edited in the UI for a publishable topic.
#[derive(Component, Debug, Clone, Default, PartialEq)]
pub struct TopicEdit(pub serde_json::Value);

/// An active subscription delivering reflected values.
#[derive(Component)]
pub struct Subscription(pub Box<dyn DynSubscription>);

impl std::fmt::Debug for Subscription {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_tuple("Subscription")
            .field(&"<dyn DynSubscription>")
            .finish()
    }
}

/// An active publisher accepting reflected values.
#[derive(Component)]
pub struct Publisher(pub Box<dyn DynPublisher>);

impl std::fmt::Debug for Publisher {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_tuple("Publisher")
            .field(&"<dyn DynPublisher>")
            .finish()
    }
}

/// Temporary marker inserted by the UI to request a publish of
/// [`TopicEdit`]'s current value.
#[derive(Component, Debug)]
pub struct PublishRequest;

/// Convert a DDS type name to a ROS 2 type name.
///
/// DDS discovery reports types like `std_msgs::msg::dds_::String_`.
/// This function normalises them to `std_msgs/String`.
///
/// If the name already looks like a ROS type (`pkg/Type`), it is returned
/// unchanged.
pub fn dds_type_to_ros_type(dds_type: &str) -> String {
    // Pattern: "<pkg>::msg::dds_::<Type>_"
    let parts: Vec<&str> = dds_type.split("::").collect();
    if parts.len() >= 4 && parts[1] == "msg" && parts[2] == "dds_" {
        let pkg = parts[0];
        let raw_type = parts[3];
        // The trailing underscore is a DDS convention – strip it.
        let type_name = raw_type.strip_suffix('_').unwrap_or(raw_type);
        return format!("{pkg}/{type_name}");
    }
    // Already in ROS form or unknown – return as-is.
    dds_type.to_owned()
}

/// Classify a raw DDS topic name into a [`TopicKind`].
pub fn topic_kind_from_dds_name(dds_name: &str) -> TopicKind {
    if let Some(rest) = dds_name.strip_prefix("rq/") {
        let service = rest.strip_suffix("/Request").unwrap_or(rest);
        TopicKind::ServiceRequest(format!("/{service}"))
    } else if let Some(rest) = dds_name.strip_prefix("rr/") {
        let service = rest.strip_suffix("/Reply").unwrap_or(rest);
        TopicKind::ServiceReply(format!("/{service}"))
    } else if let Some(rest) = dds_name.strip_prefix("rt/") {
        if let Some(idx) = rest.find("/_action/") {
            TopicKind::Action(format!("/{}", &rest[..idx]))
        } else {
            TopicKind::Normal(format!("/{rest}"))
        }
    } else {
        TopicKind::Unknown
    }
}

/// Drain subscriptions and update [`TopicValue`] on the corresponding
/// entities with the latest reflected message.
pub fn poll_subscription_values(mut topics: Query<(&Subscription, &mut TopicValue)>) {
    for (subscription, mut latest) in topics.iter_mut() {
        if let Some(value) = subscription.0.poll() {
            latest.0 = Some(value);
        }
    }
}

/// Publish the contents of [`TopicEdit`] when signalled by the UI.
///
/// The UI edits `TopicEdit` and then inserts a [`PublishRequest`] marker
/// component.  This system picks those up, sends the value, and removes the
/// marker.
pub fn handle_publish_requests(
    mut commands: Commands,
    requests: Query<(Entity, &TopicInfo, &TopicEdit, &Publisher), With<PublishRequest>>,
) {
    for (entity, info, edit, publisher) in requests.iter() {
        if let Err(e) = publisher.0.publish(&edit.0) {
            tracing::error!("Failed to publish on '{}': {e}", info.topic_name);
        }
        commands.entity(entity).remove::<PublishRequest>();
    }
}
