//! Native **Zenoh** transport for `rmw_zenoh` ROS 2 systems (feature `zenoh`).
//!
//! Like [`rosbridge`](crate::rosbridge) and [`ros_plugin`](crate::ros_plugin),
//! this is just another backend on the [`topics`](crate::topics) seam: it
//! discovers topics into [`TopicInfo`] entities and feeds
//! `/robot_description` + `/joint_states` into the scene. The point is that
//! the rest of the app is unchanged — the modularity is the ECS components,
//! not a transport trait.
//!
//! **Discovery** uses Zenoh liveliness tokens (`@ros2_lv/**`). For a
//! publisher/subscriber the token is
//! `@ros2_lv/<domain>/<session>/<node>/<entity>/<MP|MS>/<enclave>/<namespace>/<node_name>/<mangled_name>/<type>/<type_hash>/<qos>`
//! with `/` mangled to `%` in names. **Data** flows on the key
//! `<domain>/<mangled_name>/<type>/<type_hash>` as CDR — identical to DDS, so
//! our [`ros_msgs`](crate::ros_msgs) structs decode it directly.
//!
//! Verified against `docker/zenoh` (ROS 2 Jazzy + rmw_zenoh) in
//! `tests/zenoh_integration.rs`.

use std::collections::{BTreeMap, HashSet};
use std::sync::{Arc, Mutex};

use bevy::prelude::*;

use crate::ros_msgs;
use crate::scene::{JointPositions, PendingRobot, RobotHandle};
use crate::topics::{TopicInfo, TopicKind};

/// A topic discovered from a liveliness token.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ZenohTopic {
    /// ROS topic name, unmangled (e.g. `/robot_description`).
    pub name: String,
    /// ROS type, our short form (e.g. `std_msgs/String`).
    pub type_name: String,
    /// Full Zenoh data key (`<domain>/<mangled>/<type>/<hash>`).
    pub data_key: String,
}

/// Unmangle an rmw_zenoh name segment: `%` back to `/`.
fn unmangle(s: &str) -> String {
    s.replace('%', "/")
}

/// `std_msgs::msg::dds_::String_` -> `std_msgs/String`; passes through other
/// forms unchanged.
fn dds_type_to_ros(type_name: &str) -> String {
    let parts: Vec<&str> = type_name.split("::").collect();
    if parts.len() == 4 && parts[1] == "msg" && parts[2] == "dds_" {
        let ty = parts[3].strip_suffix('_').unwrap_or(parts[3]);
        return format!("{}/{}", parts[0], ty);
    }
    type_name.to_owned()
}

/// Parse a publisher/subscriber liveliness token into a [`ZenohTopic`].
/// Returns `None` for nodes, services, or malformed tokens.
pub fn parse_liveliness_token(key: &str) -> Option<ZenohTopic> {
    let parts: Vec<&str> = key.split('/').collect();
    // @ros2_lv/domain/session/node/entity/kind/enclave/ns/node_name/
    //   mangled_name/type/type_hash/qos  -> 13 segments. We index up to the
    //   type hash (parts[11]) and never read the trailing qos, so require at
    //   least 12 segments rather than the full 13.
    if parts.len() < 12 || parts[0] != "@ros2_lv" {
        return None;
    }
    let kind = parts[5];
    if kind != "MP" && kind != "MS" {
        return None; // nodes (NN) and services (SS/SC) are not topics
    }
    let domain = parts[1];
    let mangled = parts[9];
    let type_name = parts[10];
    let type_hash = parts[11];
    let name = unmangle(mangled);
    // The liveliness token mangles the topic to a single `%`-joined segment,
    // but the data key is hierarchical: real `/`, leading slash dropped
    // (`0/robot_description/<type>/<hash>`).
    let key_name = name.trim_start_matches('/');
    Some(ZenohTopic {
        name: name.clone(),
        type_name: dds_type_to_ros(type_name),
        data_key: format!("{domain}/{key_name}/{type_name}/{type_hash}"),
    })
}

/// Latest data received from Zenoh, drained by Bevy systems each frame.
#[derive(Default)]
struct ZenohShared {
    /// Discovered topics by name (deduped).
    topics: BTreeMap<String, ZenohTopic>,
    /// Latest `/robot_description` URDF, once.
    robot_description: Option<String>,
    /// Latest `/joint_states` (names, positions).
    joint_state: Option<(Vec<String>, Vec<f64>)>,
}

/// Bevy resource holding the shared state and keeping the worker alive.
#[derive(Resource, Clone)]
pub struct ZenohSession {
    shared: Arc<Mutex<ZenohShared>>,
    /// Kept so the tokio runtime/thread lives as long as the app.
    _worker: Arc<std::thread::JoinHandle<()>>,
}

/// Connects to a Zenoh router and mirrors an rmw_zenoh ROS 2 graph into the
/// ECS. Add it when `options.zenoh` is set.
pub struct ZenohPlugin {
    /// Router endpoint(s), e.g. `tcp/localhost:7447`.
    pub endpoints: Vec<String>,
}

impl Plugin for ZenohPlugin {
    fn build(&self, app: &mut App) {
        let shared = Arc::new(Mutex::new(ZenohShared::default()));
        let worker = spawn_worker(self.endpoints.clone(), shared.clone());
        app.insert_resource(ZenohSession {
            shared,
            _worker: Arc::new(worker),
        });
        app.add_systems(
            Update,
            (
                discover_topics,
                feed_robot_from_zenoh,
                feed_joints_from_zenoh,
            ),
        );
    }
}

/// Spawn the tokio runtime thread that owns the Zenoh session, liveliness
/// discovery, and per-topic data subscriptions.
fn spawn_worker(
    endpoints: Vec<String>,
    shared: Arc<Mutex<ZenohShared>>,
) -> std::thread::JoinHandle<()> {
    std::thread::Builder::new()
        .name("zenoh".into())
        .spawn(move || {
            let rt = match tokio::runtime::Builder::new_multi_thread()
                .enable_all()
                .build()
            {
                Ok(rt) => rt,
                Err(e) => {
                    tracing::error!("zenoh: tokio runtime: {e}");
                    return;
                }
            };
            rt.block_on(async move {
                if let Err(e) = run_session(endpoints, shared).await {
                    tracing::error!("zenoh: {e}");
                }
            });
        })
        .expect("spawn zenoh worker")
}

async fn run_session(
    endpoints: Vec<String>,
    shared: Arc<Mutex<ZenohShared>>,
) -> anyhow::Result<()> {
    let mut config = zenoh::Config::default();
    let json = serde_json::to_string(&endpoints)?;
    config
        .insert_json5("connect/endpoints", &json)
        .map_err(|e| anyhow::anyhow!("zenoh config: {e}"))?;
    let session = zenoh::open(config)
        .await
        .map_err(|e| anyhow::anyhow!("zenoh open: {e}"))?;
    tracing::info!("zenoh: connected to {endpoints:?}");

    let liveliness = session
        .liveliness()
        .declare_subscriber("@ros2_lv/**")
        .await
        .map_err(|e| anyhow::anyhow!("liveliness subscribe: {e}"))?;

    // Seed with already-present tokens.
    if let Ok(replies) = session.liveliness().get("@ros2_lv/**").await {
        while let Ok(reply) = replies.recv_async().await {
            if let Ok(s) = reply.result() {
                handle_token(&session, &shared, s.key_expr().as_str()).await;
            }
        }
    }

    // Then live updates.
    while let Ok(sample) = liveliness.recv_async().await {
        handle_token(&session, &shared, sample.key_expr().as_str()).await;
    }
    Ok(())
}

/// Discover a topic from a token and, for the topics we render, declare a
/// data subscriber that decodes CDR into the shared state.
async fn handle_token(session: &zenoh::Session, shared: &Arc<Mutex<ZenohShared>>, key: &str) {
    let Some(topic) = parse_liveliness_token(key) else {
        return;
    };
    {
        let mut s = shared.lock().unwrap_or_else(|p| p.into_inner());
        if s.topics.contains_key(&topic.name) {
            return; // already handled
        }
        s.topics.insert(topic.name.clone(), topic.clone());
    }

    // Subscribe to the two topics that drive the scene.
    match topic.name.as_str() {
        "/robot_description" => {
            subscribe_description(session, shared.clone(), topic.data_key).await;
        }
        "/joint_states" => {
            subscribe_joint_states(session, shared.clone(), topic.data_key).await;
        }
        _ => {}
    }
}

async fn subscribe_description(
    session: &zenoh::Session,
    shared: Arc<Mutex<ZenohShared>>,
    data_key: String,
) {
    // /robot_description is latched (TRANSIENT_LOCAL). rmw_zenoh implements
    // that with a zenoh-ext AdvancedPublisher cache, so we need an
    // AdvancedSubscriber with history to recover the retained value a plain
    // subscriber would miss (it also detects late publishers).
    use zenoh_ext::AdvancedSubscriberBuilderExt;
    let sub = match session
        .declare_subscriber(data_key)
        .history(zenoh_ext::HistoryConfig::default().detect_late_publishers())
        .await
    {
        Ok(sub) => sub,
        Err(e) => {
            tracing::error!("zenoh: robot_description subscribe: {e}");
            return;
        }
    };
    tokio::spawn(async move {
        while let Ok(sample) = sub.recv_async().await {
            store_description(&shared, &sample.payload().to_bytes());
        }
    });
}

fn store_description(shared: &Arc<Mutex<ZenohShared>>, bytes: &[u8]) {
    match cdr::deserialize::<ros_msgs::String>(bytes) {
        Ok(msg) => {
            shared
                .lock()
                .unwrap_or_else(|p| p.into_inner())
                .robot_description = Some(msg.data);
        }
        Err(e) => tracing::debug!("zenoh: decode robot_description: {e}"),
    }
}

async fn subscribe_joint_states(
    session: &zenoh::Session,
    shared: Arc<Mutex<ZenohShared>>,
    data_key: String,
) {
    let sub = match session.declare_subscriber(data_key).await {
        Ok(sub) => sub,
        Err(e) => {
            tracing::error!("zenoh: joint_states subscribe: {e}");
            return;
        }
    };
    tokio::spawn(async move {
        while let Ok(sample) = sub.recv_async().await {
            let bytes = sample.payload().to_bytes();
            match cdr::deserialize::<ros_msgs::JointState>(&bytes) {
                Ok(msg) => {
                    shared.lock().unwrap_or_else(|p| p.into_inner()).joint_state =
                        Some((msg.name, msg.position));
                }
                Err(e) => tracing::debug!("zenoh: decode joint_states: {e}"),
            }
        }
    });
}

/// Mirror discovered topics into [`TopicInfo`] entities (for the panel).
fn discover_topics(
    mut commands: Commands,
    session: Res<ZenohSession>,
    existing: Query<&TopicInfo>,
) {
    let known: HashSet<&str> = existing.iter().map(|t| t.topic_name.as_str()).collect();
    let shared = session.shared.lock().unwrap_or_else(|p| p.into_inner());
    for topic in shared.topics.values() {
        if !known.contains(topic.name.as_str()) {
            commands.spawn(TopicInfo::new(
                topic.name.clone(),
                topic.type_name.clone(),
                TopicKind::Normal(topic.name.clone()),
            ));
        }
    }
}

/// Parse `/robot_description` into a pending [`RobotModel`].
fn feed_robot_from_zenoh(
    session: Res<ZenohSession>,
    mut pending: ResMut<PendingRobot>,
    robots: Query<&RobotHandle>,
) {
    if pending.0.is_some() || !robots.is_empty() {
        return;
    }
    let xml = session
        .shared
        .lock()
        .unwrap_or_else(|p| p.into_inner())
        .robot_description
        .clone();
    let Some(xml) = xml else { return };
    match crate::robot::RobotModel::from_urdf_str(&xml) {
        Ok(model) => {
            tracing::info!("zenoh: received robot '{}'", model.name());
            pending.0 = Some(Arc::new(model));
        }
        Err(e) => tracing::error!("zenoh: bad /robot_description: {e}"),
    }
}

/// Feed `/joint_states` into [`JointPositions`].
fn feed_joints_from_zenoh(session: Res<ZenohSession>, mut joints: ResMut<JointPositions>) {
    let state = session
        .shared
        .lock()
        .unwrap_or_else(|p| p.into_inner())
        .joint_state
        .clone();
    if let Some((names, positions)) = state {
        for (name, position) in names.into_iter().zip(positions) {
            joints.positions.insert(name, position);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_publisher_token() {
        let key = "@ros2_lv/0/2e44/0/13/MP/%/%/robot_state_publisher/%robot_description/std_msgs::msg::dds_::String_/RIHS01_df668c/:1:,1:,:,:,,";
        let t = parse_liveliness_token(key).expect("parses");
        assert_eq!(t.name, "/robot_description");
        assert_eq!(t.type_name, "std_msgs/String");
        assert_eq!(
            t.data_key,
            "0/robot_description/std_msgs::msg::dds_::String_/RIHS01_df668c"
        );
    }

    #[test]
    fn parses_joint_states_token() {
        let key = "@ros2_lv/0/abc/0/11/MP/%/%/joint_state_publisher/%joint_states/sensor_msgs::msg::dds_::JointState_/RIHS01_a13ee3/::,10:,:,:,,";
        let t = parse_liveliness_token(key).expect("parses");
        assert_eq!(t.name, "/joint_states");
        assert_eq!(t.type_name, "sensor_msgs/JointState");
    }

    #[test]
    fn ignores_nodes_and_services() {
        assert!(
            parse_liveliness_token("@ros2_lv/0/abc/0/0/NN/%/%/robot_state_publisher").is_none()
        );
        let svc = "@ros2_lv/0/abc/0/2/SS/%/%/rsp/%rsp%get_parameters/rcl_interfaces::srv::dds_::GetParameters_/RIHS01_x/::,1000:,:,:,,";
        assert!(parse_liveliness_token(svc).is_none());
    }

    #[test]
    fn unmangles_nested_names() {
        let key = "@ros2_lv/0/abc/0/3/MS/%/%/n/%robot%left_arm%state/sensor_msgs::msg::dds_::JointState_/RIHS01_h/:,:,,";
        let t = parse_liveliness_token(key).expect("parses");
        assert_eq!(t.name, "/robot/left_arm/state");
    }
}
