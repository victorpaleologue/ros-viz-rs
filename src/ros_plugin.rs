//! Bevy systems for discovering ROS 2 topics and feeding them into the ECS.
//!
//! This module is only compiled when the `ros` feature is enabled.
//!
//! The main entry point is [`RosPlugin`], which owns the DDS context, node
//! and spinner, and registers `populate_topics`: that system drains DDS
//! discovery events and reconciles them with the set of [`TopicInfo`]
//! entities in the world.

use std::collections::{HashMap, HashSet};
use std::sync::Arc;
use std::sync::Mutex;
use std::thread::JoinHandle;

use bevy::prelude::*;

use ros2_client::Context;
use ros2_client::NodeEvent;
use ros2_client::rustdds::DomainParticipantStatusEvent;
use ros2_client::rustdds::EndpointDescription;
use ros2_client::rustdds::GUID;

// ---------------------------------------------------------------------------
// Plugin
// ---------------------------------------------------------------------------

/// Bevy plugin that maintains a ROS context and a node,
/// and populate the ECS with topics and associated subscribers and publishers.
pub struct RosPlugin {
    session: RosSession,
    _spinner_thread: Option<JoinHandle<()>>,
}

impl RosPlugin {
    pub fn new(domain_id: u16, node_name: &str) -> Result<Self, String> {
        let ctx = Context::with_options(ros2_client::ContextOptions::new().domain_id(domain_id))
            .map_err(|e| format!("failed to create ROS context: {e:?}"))?;
        let node_name = ros2_client::NodeName::new("/", node_name)
            .map_err(|e| format!("invalid node name: {e:?}"))?;
        let mut node = ctx
            .new_node(node_name, ros2_client::NodeOptions::new())
            .map_err(|e| format!("failed to create ROS node: {e:?}"))?;

        let spinner = node
            .spinner()
            .map_err(|e| format!("failed to create spinner: {e:?}"))?;

        let spinner_thread = std::thread::Builder::new()
            .name("ros_spinner".into())
            .spawn(move || {
                if let Err(e) = futures::executor::block_on(spinner.spin()) {
                    tracing::warn!("Spinner exited with error: {e:?}");
                }
            })
            .map_err(|e| format!("failed to spawn spinner thread: {e}"))?;

        let event_receiver = node.status_receiver();

        Ok(Self {
            session: RosSession {
                ctx: Arc::new(Mutex::new(ctx)),
                node: Arc::new(Mutex::new(node)),
                event_receiver,
            },
            _spinner_thread: Some(spinner_thread),
        })
    }
}

impl Plugin for RosPlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(self.session.clone());
        app.init_resource::<TopicIndex>();
        app.add_systems(Update, populate_topics);
    }
}

// ---------------------------------------------------------------------------
// Resource
// ---------------------------------------------------------------------------
#[derive(Resource, Clone)]
pub struct RosSession {
    pub ctx: Arc<Mutex<ros2_client::Context>>,
    pub node: Arc<Mutex<ros2_client::Node>>,
    pub event_receiver: async_channel::Receiver<NodeEvent>,
}

impl std::fmt::Debug for RosSession {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("RosSession").finish()
    }
}

// ---------------------------------------------------------------------------
// Components
// ---------------------------------------------------------------------------
pub use crate::topics::{TopicInfo, TopicKind, dds_type_to_ros_type, topic_kind_from_dds_name};

/// Store the set of readers and writers (DDS endpoints) detected for a topic.
/// Can be used to determine whether a topic is publishable/subscribable:
/// - readers.len() > 0 => publishable.
/// - writers.len() > 0 => subscribable.
#[derive(Component)]
pub(crate) struct ReadersAndWriters {
    readers: HashSet<GUID>,
    writers: HashSet<GUID>,
}

// Helper enum to represent either a reader or a writer endpoint when updating the sets.
enum ReaderOrWriter {
    Reader(GUID),
    Writer(GUID),
}

impl ReadersAndWriters {
    pub fn new() -> Self {
        Self {
            readers: HashSet::new(),
            writers: HashSet::new(),
        }
    }

    fn add(&mut self, reader_or_writer: ReaderOrWriter) {
        match reader_or_writer {
            ReaderOrWriter::Reader(guid) => {
                self.readers.insert(guid);
            }
            ReaderOrWriter::Writer(guid) => {
                self.writers.insert(guid);
            }
        }
    }

    fn remove(&mut self, reader_or_writer: &ReaderOrWriter) -> bool {
        match reader_or_writer {
            ReaderOrWriter::Reader(guid) => self.readers.remove(guid),
            ReaderOrWriter::Writer(guid) => self.writers.remove(guid),
        }
    }

    pub(crate) fn is_publishable(&self) -> bool {
        !self.readers.is_empty()
    }

    pub(crate) fn is_subscribable(&self) -> bool {
        !self.writers.is_empty()
    }
}

// ---------------------------------------------------------------------------
// Systems
// ---------------------------------------------------------------------------
/// O(1) lookups for discovery bookkeeping: topic name -> entity for endpoint
/// arrivals, endpoint GUID -> entity for losses. Stale entries (entity gone)
/// are treated as absent and dropped lazily.
#[derive(Resource, Default)]
pub(crate) struct TopicIndex {
    by_name: HashMap<String, Entity>,
    by_guid: HashMap<GUID, Entity>,
}

/// Drain [`NodeEvent`]s to list topics and their readers/writers as they are discovered and lost.
/// Spawns one entity per topic, with components [`TopicInfo`] and [`ReadersAndWriters`].
/// When no more readers/writers are detected for a topic, its entity is despawned.
pub(crate) fn populate_topics(
    mut commands: Commands,
    ros_session: Res<RosSession>,
    mut index: ResMut<TopicIndex>,
    mut topics: Query<(Entity, &TopicInfo, &mut ReadersAndWriters)>,
) {
    // Update the set of readers/writers for each topic based on DDS discovery events.
    // If the topic is new, create a new entity with the raw DDS topic name and type.
    let update_or_create = |commands: &mut Commands,
                            index: &mut TopicIndex,
                            topics: &mut Query<(Entity, &TopicInfo, &mut ReadersAndWriters)>,
                            r_or_w: ReaderOrWriter,
                            endpoint: EndpointDescription| {
        let guid = match &r_or_w {
            ReaderOrWriter::Reader(guid) | ReaderOrWriter::Writer(guid) => *guid,
        };
        let existing = index
            .by_name
            .get(&endpoint.topic_name)
            .copied()
            // The entity may have been despawned externally; treat a stale
            // index entry as absent.
            .filter(|&entity| topics.get(entity).is_ok())
            // Index miss: fall back to a scan and backfill, so entities
            // spawned outside this system are still found.
            .or_else(|| {
                let found = topics
                    .iter()
                    .find(|(_, info, _)| info.topic_name == endpoint.topic_name)
                    .map(|(entity, _, _)| entity);
                if let Some(entity) = found {
                    index.by_name.insert(endpoint.topic_name.clone(), entity);
                }
                found
            });
        if let Some(entity) = existing {
            let (_, _, mut rw) = topics.get_mut(entity).expect("checked above");
            rw.add(r_or_w);
            index.by_guid.insert(guid, entity);
        } else {
            // Entity doesn't exist yet -- spawn it with the reader/writer
            // already set. We can't query a just-spawned entity (commands
            // are deferred), but its id is known and indexed immediately.
            let type_name = dds_type_to_ros_type(&endpoint.type_name);
            let kind = topic_kind_from_dds_name(&endpoint.topic_name);
            let topic_info = TopicInfo::new(&endpoint.topic_name, &type_name, kind);
            let mut rw = ReadersAndWriters::new();
            rw.add(r_or_w);
            let entity = commands.spawn((topic_info, rw)).id();
            index.by_name.insert(endpoint.topic_name.clone(), entity);
            index.by_guid.insert(guid, entity);
        }
    };

    // Remove the reader/writer from the topic's ReadersAndWriters.  If that was the last one, despawn the topic entity.
    let remove_and_cleanup = |commands: &mut Commands,
                              index: &mut TopicIndex,
                              topics: &mut Query<(Entity, &TopicInfo, &mut ReadersAndWriters)>,
                              r_or_w: ReaderOrWriter| {
        let guid = match &r_or_w {
            ReaderOrWriter::Reader(guid) | ReaderOrWriter::Writer(guid) => *guid,
        };
        let Some(entity) = index.by_guid.remove(&guid) else {
            return;
        };
        let Ok((entity, info, mut rw)) = topics.get_mut(entity) else {
            return;
        };
        if rw.remove(&r_or_w) && rw.readers.is_empty() && rw.writers.is_empty() {
            // That was the topic's last endpoint: drop the entity.
            index.by_name.remove(&info.topic_name);
            commands.entity(entity).despawn();
        }
    };

    while let Ok(event) = ros_session.event_receiver.try_recv() {
        let NodeEvent::DDS(dds_event) = event else {
            continue;
        };
        match dds_event {
            DomainParticipantStatusEvent::WriterDetected { writer } => {
                update_or_create(
                    &mut commands,
                    &mut index,
                    &mut topics,
                    ReaderOrWriter::Writer(writer.guid),
                    writer,
                );
            }
            DomainParticipantStatusEvent::ReaderDetected { reader } => {
                update_or_create(
                    &mut commands,
                    &mut index,
                    &mut topics,
                    ReaderOrWriter::Reader(reader.guid),
                    reader,
                );
            }
            DomainParticipantStatusEvent::WriterLost { guid, .. } => {
                remove_and_cleanup(
                    &mut commands,
                    &mut index,
                    &mut topics,
                    ReaderOrWriter::Writer(guid),
                );
            }
            DomainParticipantStatusEvent::ReaderLost { guid, .. } => {
                remove_and_cleanup(
                    &mut commands,
                    &mut index,
                    &mut topics,
                    ReaderOrWriter::Reader(guid),
                );
            }
            _ => {}
        }
    }
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    use std::time::{Duration, Instant};

    use bevy::MinimalPlugins;
    use ros2_client::{
        Context, ContextOptions, DEFAULT_SUBSCRIPTION_QOS, MessageTypeName, Name, NodeName,
        NodeOptions,
    };

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

    /// Build a fake [`EndpointDescription`] for a given DDS topic.
    fn fake_endpoint(topic_name: &str, type_name: &str) -> EndpointDescription {
        EndpointDescription {
            updated_time: chrono::Utc::now(),
            guid: GUID::GUID_UNKNOWN,
            topic_name: topic_name.to_owned(),
            type_name: type_name.to_owned(),
            qos: ros2_client::ros2::QosPolicyBuilder::new().build(),
        }
    }

    /// Build a [`DomainParticipantStatusEvent::WriterDetected`] for a given
    /// DDS topic name and type.
    fn writer_detected_event(topic_name: &str, type_name: &str) -> NodeEvent {
        NodeEvent::DDS(DomainParticipantStatusEvent::WriterDetected {
            writer: fake_endpoint(topic_name, type_name),
        })
    }

    /// Build a [`DomainParticipantStatusEvent::ReaderDetected`] for a given
    /// DDS topic name and type.
    fn reader_detected_event(topic_name: &str, type_name: &str) -> NodeEvent {
        NodeEvent::DDS(DomainParticipantStatusEvent::ReaderDetected {
            reader: fake_endpoint(topic_name, type_name),
        })
    }

    /// When a `WriterDetected` event arrives *before* discovery has spawned
    /// the entity, `populate_topics` should spawn a `TopicInfo` entity
    /// with `IsSubscribable` and the correct type/kind.
    #[test]
    fn writer_event_spawns_entity_when_missing() {
        let mut app = App::new();
        app.add_plugins(MinimalPlugins);

        // Create a channel to feed events synthetically.
        let (tx, rx) = async_channel::unbounded::<NodeEvent>();
        // We can't easily construct a full RosNode without a real DDS context,
        // so send the event directly and test the system logic.
        tx.send_blocking(writer_detected_event(
            "rt/chatter",
            "std_msgs::msg::dds_::String_",
        ))
        .unwrap();

        // Manually insert a minimal RosNode-like resource is not practical.
        // Instead, run the system function directly via the world.
        // We'll inline the system logic test by manually running it.
        // -- Use the system as a standalone function through Bevy scheduling --

        // Insert a throwaway ROS context and node, but use our mock event receiver.
        let domain_id = random_domain_id();
        let ctx = ros2_client::Context::with_options(
            ros2_client::ContextOptions::new().domain_id(domain_id),
        )
        .expect("create context");
        let node_name = NodeName::new("/", "test_node").expect("valid node name");
        let node = ctx
            .new_node(node_name, NodeOptions::new())
            .expect("create node");
        let mut ros_session = RosSession {
            ctx: Arc::new(Mutex::new(ctx)),
            node: Arc::new(Mutex::new(node)),
            event_receiver: rx.clone(),
        };
        ros_session.event_receiver = rx;
        app.insert_resource(ros_session);

        app.init_resource::<TopicIndex>();
        app.add_systems(Update, populate_topics);
        app.update();

        // The entity should have been spawned with TopicInfo + ReadersAndWriters.
        let world = app.world_mut();
        let mut query = world.query::<(&TopicInfo, Option<&ReadersAndWriters>)>();
        let results: Vec<_> = query.iter(world).collect();

        let matching = results
            .iter()
            .find(|(info, _)| info.topic_name == "rt/chatter");
        assert!(
            matching.is_some(),
            "Expected a TopicInfo for rt/chatter, found: {:?}",
            results
                .iter()
                .map(|(i, _)| &i.topic_name)
                .collect::<Vec<_>>()
        );
        let (info, rs_and_ws) = matching.unwrap();
        assert_eq!(info.type_name, "std_msgs/String");
        assert_eq!(info.kind, TopicKind::Normal("/chatter".into()));
        let rs_and_ws = rs_and_ws.expect("Expected ReadersAndWriters component");
        assert!(rs_and_ws.is_subscribable(), "Expected IsSubscribable");
        assert!(!rs_and_ws.is_publishable(), "Should not have IsPublishable");
    }

    /// When a `ReaderDetected` event arrives before discovery, the entity
    /// should be spawned with `IsPublishable`.
    #[test]
    fn reader_event_spawns_entity_when_missing() {
        let mut app = App::new();
        app.add_plugins(MinimalPlugins);

        let (tx, rx) = async_channel::unbounded::<NodeEvent>();
        tx.send_blocking(reader_detected_event(
            "rt/cmd_vel",
            "geometry_msgs::msg::dds_::Twist_",
        ))
        .unwrap();

        // Insert a throwaway ROS context and node, but use our mock event receiver.
        let domain_id = random_domain_id();
        let ctx = ros2_client::Context::with_options(
            ros2_client::ContextOptions::new().domain_id(domain_id),
        )
        .expect("create context");
        let node_name = NodeName::new("/", "test_node").expect("valid node name");
        let node = ctx
            .new_node(node_name, NodeOptions::new())
            .expect("create node");
        let mut ros_session = RosSession {
            ctx: Arc::new(Mutex::new(ctx)),
            node: Arc::new(Mutex::new(node)),
            event_receiver: rx.clone(),
        };
        ros_session.event_receiver = rx;
        app.insert_resource(ros_session);

        app.init_resource::<TopicIndex>();
        app.add_systems(Update, populate_topics);
        app.update();

        let world = app.world_mut();
        let mut query = world.query::<(&TopicInfo, Option<&ReadersAndWriters>)>();
        let results: Vec<_> = query.iter(world).collect();

        let matching = results
            .iter()
            .find(|(info, _)| info.topic_name == "rt/cmd_vel");
        assert!(matching.is_some(), "Expected a TopicInfo for rt/cmd_vel");
        let (info, rs_and_ws) = matching.unwrap();
        assert_eq!(info.type_name, "geometry_msgs/Twist");
        assert_eq!(info.kind, TopicKind::Normal("/cmd_vel".into()));
        let rs_and_ws = rs_and_ws.expect("Expected ReadersAndWriters component");
        assert!(rs_and_ws.is_publishable(), "Expected IsPublishable");
        assert!(
            !rs_and_ws.is_subscribable(),
            "Should not have IsSubscribable"
        );
    }

    /// When the entity already exists (spawned by discovery), the event
    /// should only add the marker component without spawning a duplicate.
    #[test]
    fn writer_event_adds_marker_to_existing_entity() {
        let mut app = App::new();
        app.add_plugins(MinimalPlugins);

        // Pre-spawn the entity as if discovery already ran.
        // We'll say that there is one reader (it is publishable) but no writers (not subscribable).
        let mut rs_and_ws = ReadersAndWriters::new();
        let fake_reader_guid = GUID::from_bytes(*b"fake_reader!read");
        rs_and_ws.add(ReaderOrWriter::Reader(fake_reader_guid));

        app.world_mut().spawn((
            TopicInfo::new(
                "rt/chatter",
                "std_msgs/String",
                TopicKind::Normal("/chatter".into()),
            ),
            rs_and_ws,
        ));

        let (tx, rx) = async_channel::unbounded::<NodeEvent>();
        tx.send_blocking(writer_detected_event(
            "rt/chatter",
            "std_msgs::msg::dds_::String_",
        ))
        .unwrap();

        // Insert a throwaway ROS context and node, but use our mock event receiver.
        let domain_id = random_domain_id();
        let ctx = ros2_client::Context::with_options(
            ros2_client::ContextOptions::new().domain_id(domain_id),
        )
        .expect("create context");
        let node_name = NodeName::new("/", "test_node").expect("valid node name");
        let node = ctx
            .new_node(node_name, NodeOptions::new())
            .expect("create node");
        let mut ros_session = RosSession {
            ctx: Arc::new(Mutex::new(ctx)),
            node: Arc::new(Mutex::new(node)),
            event_receiver: rx.clone(),
        };
        ros_session.event_receiver = rx;
        app.insert_resource(ros_session);

        app.init_resource::<TopicIndex>();
        app.add_systems(Update, populate_topics);
        app.update();
        // Should have exactly one entity for rt/chatter (no duplicate).
        let world = app.world_mut();
        let mut query = world.query::<(&TopicInfo, Option<&ReadersAndWriters>)>();
        let chatter_entities: Vec<_> = query
            .iter(world)
            .filter(|(info, _)| info.topic_name == "rt/chatter")
            .collect();
        assert_eq!(
            chatter_entities.len(),
            1,
            "Expected exactly one entity for rt/chatter, got {}",
            chatter_entities.len()
        );
        assert!(
            chatter_entities[0]
                .1
                .expect("missing readers and writers")
                .is_subscribable(),
            "Expected IsSubscribable on existing entity"
        );
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
        crate::require_dds_multicast!();
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
        let ros_plugin = RosPlugin::new(domain_id, "test_node").expect("create ROS plugin");
        app.add_plugins(ros_plugin);

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
