//! Bevy systems for discovering ROS 2 topics and feeding them into the ECS.
//!
//! This module is only compiled when the `ros` feature is enabled.
//!
//! The main entry point is [`RosDiscoveryPlugin`], which registers the
//! [`poll_discovered_topics`] system.  That system periodically calls
//! [`Context::discovered_topics()`](ros2_client::Context::discovered_topics)
//! and reconciles the result with the set of [`TopicInfo`] entities in the
//! world.

use bevy::prelude::*;
use ros2_client::Context;
use std::collections::HashSet;

use crate::topics_view::{TopicDataSource, TopicInfo, TopicKind};

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

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
///
/// ROS 2 maps its concepts onto DDS using naming conventions:
/// - `rt/…` — regular pub/sub topics
/// - `rq/…` — service request channels
/// - `rr/…` — service reply channels
/// - `rt/…/_action/…` — action-related topics
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

// ---------------------------------------------------------------------------
// Resource
// ---------------------------------------------------------------------------

/// Bevy resource holding a clone of the ROS 2 [`Context`].
///
/// Because `Context` wraps `Arc<Mutex<…>>`, cloning is cheap and the
/// resource can coexist with any other handle to the same context.
#[derive(Resource, Clone)]
pub struct RosContext(pub Context);

impl std::fmt::Debug for RosContext {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("RosContext").finish()
    }
}

// ---------------------------------------------------------------------------
// System
// ---------------------------------------------------------------------------

/// Poll `Context::discovered_topics()` each frame and reconcile the set of
/// [`TopicInfo`] entities in the world.
///
/// - New topics are spawned as entities with a [`TopicInfo`] component.
/// - Topics that disappeared are despawned.
pub fn poll_discovered_topics(
    mut commands: Commands,
    ros_ctx: Option<Res<RosContext>>,
    existing: Query<(Entity, &TopicInfo), With<TopicDataSource>>,
) {
    let Some(ctx) = ros_ctx else { return };

    let discovered = ctx.0.discovered_topics();

    // Deduplicate: discovered_topics() may return the same topic name from
    // multiple endpoints (publishers + subscribers).
    let discovered_names: HashSet<String> =
        discovered.iter().map(|d| d.topic_name().clone()).collect();

    // Build a set of names we already track.
    let existing_names: HashSet<String> = existing
        .iter()
        .map(|(_, info)| info.topic_name.clone())
        .collect();

    // Spawn new topics.
    for topic_data in &discovered {
        let name = topic_data.topic_name().clone();
        if !existing_names.contains(&name) {
            let type_name = dds_type_to_ros_type(&topic_data.type_name());
            let kind = topic_kind_from_dds_name(&name);
            tracing::debug!("Discovered new topic: {name} (type: {type_name})");
            commands.spawn((TopicInfo::new(name, type_name, kind), TopicDataSource));
        }
    }

    // Despawn topics that are no longer discovered.
    for (entity, info) in existing.iter() {
        if !discovered_names.contains(&info.topic_name) {
            tracing::debug!("Topic disappeared: {}", info.topic_name);
            commands.entity(entity).despawn();
        }
    }
}

// ---------------------------------------------------------------------------
// Plugin
// ---------------------------------------------------------------------------

/// Bevy plugin that polls ROS 2 topic discovery and keeps [`TopicInfo`]
/// entities in sync.
pub struct RosDiscoveryPlugin;

impl Plugin for RosDiscoveryPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Update, poll_discovered_topics);
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use bevy::MinimalPlugins;
    use ros2_client::{
        Context, ContextOptions, DEFAULT_SUBSCRIPTION_QOS, MessageTypeName, Name, NodeName,
        NodeOptions,
    };
    use std::time::{Duration, Instant};

    /// Pick a random DDS domain ID in 1..=232 to isolate tests from each other
    /// and from any running ROS 2 system on domain 0.
    fn random_domain_id() -> u16 {
        use std::collections::hash_map::RandomState;
        use std::hash::{BuildHasher, Hasher};
        let hash = RandomState::new().build_hasher().finish();
        (hash % 232 + 1) as u16
    }

    /// Helper: extract all TopicInfo from the world.
    fn topic_infos(app: &mut App) -> Vec<TopicInfo> {
        app.world_mut()
            .query::<&TopicInfo>()
            .iter(app.world())
            .cloned()
            .collect()
    }

    // -- TopicKind classification tests --

    // -- dds_type_to_ros_type tests --

    #[test]
    fn dds_type_string() {
        assert_eq!(
            dds_type_to_ros_type("std_msgs::msg::dds_::String_"),
            "std_msgs/String"
        );
    }

    #[test]
    fn dds_type_joint_state() {
        assert_eq!(
            dds_type_to_ros_type("sensor_msgs::msg::dds_::JointState_"),
            "sensor_msgs/JointState"
        );
    }

    #[test]
    fn ros_type_passthrough() {
        // Already in ROS form – should be returned as-is.
        assert_eq!(dds_type_to_ros_type("std_msgs/String"), "std_msgs/String");
    }

    #[test]
    fn unknown_type_passthrough() {
        assert_eq!(dds_type_to_ros_type("SomeWeirdType"), "SomeWeirdType");
    }

    // -- TopicKind classification tests --

    #[test]
    fn kind_normal_topic() {
        assert_eq!(
            topic_kind_from_dds_name("rt/joint_states"),
            TopicKind::Normal("/joint_states".into())
        );
        assert_eq!(
            topic_kind_from_dds_name("rt/robot/description"),
            TopicKind::Normal("/robot/description".into())
        );
    }

    #[test]
    fn kind_service_request() {
        assert_eq!(
            topic_kind_from_dds_name("rq/some_service/Request"),
            TopicKind::ServiceRequest("/some_service".into())
        );
    }

    #[test]
    fn kind_service_reply() {
        assert_eq!(
            topic_kind_from_dds_name("rr/some_service/Reply"),
            TopicKind::ServiceReply("/some_service".into())
        );
    }

    #[test]
    fn kind_action_topic() {
        assert_eq!(
            topic_kind_from_dds_name("rt/navigate/_action/send_goal"),
            TopicKind::Action("/navigate".into())
        );
        assert_eq!(
            topic_kind_from_dds_name("rt/my_ns/my_action/_action/feedback"),
            TopicKind::Action("/my_ns/my_action".into())
        );
    }

    #[test]
    fn kind_unknown() {
        assert_eq!(topic_kind_from_dds_name("other/topic"), TopicKind::Unknown);
        assert_eq!(topic_kind_from_dds_name(""), TopicKind::Unknown);
    }

    #[test]
    fn topic_info_new_infers_kind() {
        let normal = TopicInfo::new(
            "rt/joint_states",
            "sensor_msgs/JointState",
            topic_kind_from_dds_name("rt/joint_states"),
        );
        assert_eq!(normal.kind, TopicKind::Normal("/joint_states".into()));

        let svc = TopicInfo::new(
            "rq/get_params/Request",
            "std_srvs/Empty_Request",
            topic_kind_from_dds_name("rq/get_params/Request"),
        );
        assert_eq!(svc.kind, TopicKind::ServiceRequest("/get_params".into()));

        let action = TopicInfo::new(
            "rt/nav/_action/status",
            "action_msgs/GoalStatusArray",
            topic_kind_from_dds_name("rt/nav/_action/status"),
        );
        assert_eq!(action.kind, TopicKind::Action("/nav".into()));
    }

    // -- Integration test --

    /// Integration test: create a ROS node with a publisher and a subscriber,
    /// then verify that `poll_discovered_topics` populates `TopicInfo` entities
    /// with raw DDS names and correct kinds.
    ///
    /// This test requires a DDS network (e.g. a local daemon) but does **not**
    /// need any external ROS nodes.  It may take a moment for DDS discovery to
    /// propagate.
    #[test]
    fn discovered_topics_appear_as_entities() {
        // -- set up ROS context + node + topics --
        let domain_id = random_domain_id();
        let ctx = Context::with_options(ContextOptions::new().domain_id(domain_id))
            .expect("create ROS context");
        let node_name = NodeName::new("/", "test_discovery_node").expect("valid node name");
        let mut node = ctx
            .new_node(node_name, NodeOptions::new())
            .expect("create node");

        // Create a publisher topic and a subscriber topic.
        let pub_topic = node
            .create_topic(
                &Name::new("/", "test_pub_topic").unwrap(),
                MessageTypeName::new("std_msgs", "String"),
                &DEFAULT_SUBSCRIPTION_QOS,
            )
            .expect("create pub topic");

        let sub_topic = node
            .create_topic(
                &Name::new("/", "test_sub_topic").unwrap(),
                MessageTypeName::new("std_msgs", "String"),
                &DEFAULT_SUBSCRIPTION_QOS,
            )
            .expect("create sub topic");

        // Actually create endpoints so DDS discovery advertises them.
        let _publisher = node
            .create_publisher::<String>(&pub_topic, None)
            .expect("create publisher");
        let _subscription = node
            .create_subscription::<String>(&sub_topic, None)
            .expect("create subscription");

        // -- set up Bevy app --
        let mut app = App::new();
        app.add_plugins(MinimalPlugins);
        app.insert_resource(RosContext(ctx.clone()));
        app.add_systems(Update, poll_discovered_topics);

        // -- wait for discovery (with timeout) --
        // DDS topic names are raw: "rt/test_pub_topic", "rt/test_sub_topic"
        let deadline = Instant::now() + Duration::from_secs(10);
        let mut found_pub = false;
        let mut found_sub = false;

        while Instant::now() < deadline && !(found_pub && found_sub) {
            app.update();
            let infos = topic_infos(&mut app);
            found_pub = infos.iter().any(|t| t.topic_name == "rt/test_pub_topic");
            found_sub = infos.iter().any(|t| t.topic_name == "rt/test_sub_topic");

            if !(found_pub && found_sub) {
                std::thread::sleep(Duration::from_millis(100));
            }
        }

        let infos = topic_infos(&mut app);
        let pub_info = infos.iter().find(|t| t.topic_name == "rt/test_pub_topic");
        let sub_info = infos.iter().find(|t| t.topic_name == "rt/test_sub_topic");

        assert!(
            pub_info.is_some(),
            "expected rt/test_pub_topic in discovered topics, got: {:?}",
            infos.iter().map(|t| &t.topic_name).collect::<Vec<_>>()
        );
        assert!(
            sub_info.is_some(),
            "expected rt/test_sub_topic in discovered topics, got: {:?}",
            infos.iter().map(|t| &t.topic_name).collect::<Vec<_>>()
        );

        // Verify kind classification
        assert_eq!(
            pub_info.unwrap().kind,
            TopicKind::Normal("/test_pub_topic".into())
        );
        assert_eq!(
            sub_info.unwrap().kind,
            TopicKind::Normal("/test_sub_topic".into())
        );
    }
}
