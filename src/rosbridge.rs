//! rosbridge connection backend: ROS 2 over JSON/WebSocket.
//!
//! Speaks the [rosbridge v2 protocol](https://github.com/RobotWebTools/rosbridge_suite/blob/ros2/ROSBRIDGE_PROTOCOL.md)
//! through [`ewebsock`], which works both natively and on wasm — this is the
//! transport that lets the visualizer run in a browser against a robot
//! exposing `rosbridge_server` (the long-standing standard for web access
//! to ROS).
//!
//! It populates the same ECS surface as the DDS backend
//! ([`crate::topics`]): [`TopicInfo`] entities discovered via the `rosapi`
//! topics service, [`Subscription`]/[`TopicValue`] for incoming data
//! (rosbridge pushes JSON, which is already our reflected value format),
//! and [`Publisher`]/[`TopicEdit`] backed by `advertise`/`publish` ops.

use std::collections::HashMap;
use std::sync::mpsc;
use std::sync::{Arc, Mutex};

use bevy::prelude::*;
use serde_json::{Value, json};

use crate::messages::{DynPublisher, DynSubscription, MessageRegistry};
use crate::robot::RobotModel;
use crate::scene::JointPositions;
use crate::topics::{
    Publisher, Subscription, TopicEdit, TopicInfo, TopicKind, TopicValue, handle_publish_requests,
    poll_subscription_values,
};

/// Convert `pkg/Type` to the ROS 2 path form rosbridge expects
/// (`pkg/msg/Type`); passes through names that already carry an infix.
pub fn ros_type_to_path(type_name: &str) -> String {
    match type_name.split('/').collect::<Vec<_>>()[..] {
        [pkg, ty] => format!("{pkg}/msg/{ty}"),
        _ => type_name.to_owned(),
    }
}

/// Convert `pkg/msg/Type` (rosbridge/rosapi form) to our `pkg/Type` form.
pub fn ros_path_to_type(path: &str) -> String {
    match path.split('/').collect::<Vec<_>>()[..] {
        [pkg, "msg", ty] => format!("{pkg}/{ty}"),
        _ => path.to_owned(),
    }
}

/// One value slot per subscribed topic, filled by the socket pump and
/// drained through the standard [`Subscription`] component.
#[derive(Default)]
struct Inbox(Mutex<Option<Value>>);

/// [`DynSubscription`] backed by an [`Inbox`] the socket pump fills.
struct InboxSubscription(Arc<Inbox>);

impl DynSubscription for InboxSubscription {
    fn poll(&self) -> Option<Value> {
        self.0.0.lock().unwrap_or_else(|p| p.into_inner()).take()
    }
}

/// [`DynPublisher`] sending rosbridge `publish` ops.
struct BridgePublisher {
    outbox: Outbox,
    topic: String,
}

impl DynPublisher for BridgePublisher {
    fn publish(&self, value: &Value) -> Result<(), String> {
        self.outbox
            .send(json!({"op": "publish", "topic": self.topic, "msg": value}));
        Ok(())
    }
}

/// Sends protocol frames to the socket pump.
///
/// On wasm the WebSocket handle is `!Send`, so everything that must live in
/// ordinary (Send + Sync) ECS data — publishers, the session resource —
/// talks to the socket through this channel; [`RosbridgeIo`] drains it.
#[derive(Clone)]
struct Outbox(mpsc::Sender<Value>);

impl Outbox {
    fn send(&self, value: Value) {
        // Failure means the socket pump is gone; nothing useful to do.
        let _ = self.0.send(value);
    }
}

/// The socket itself: a non-Send resource owned by the main thread.
struct RosbridgeIo {
    sender: ewebsock::WsSender,
    receiver: ewebsock::WsReceiver,
    outbox_rx: mpsc::Receiver<Value>,
}

/// Connection state shared with ordinary systems.
#[derive(Resource)]
pub struct RosbridgeSession {
    outbox: Outbox,
    /// Inboxes for subscribed topics, keyed by topic name.
    inboxes: HashMap<String, Arc<Inbox>>,
    connected: bool,
    /// Periodic re-query of the topic list via rosapi.
    topics_poll: Timer,
}

/// Connects to a rosbridge server and mirrors its topics into the ECS.
pub struct RosbridgePlugin {
    /// WebSocket URL, e.g. `ws://localhost:9090`.
    pub url: String,
}

impl Plugin for RosbridgePlugin {
    fn build(&self, app: &mut App) {
        let (sender, receiver) = ewebsock::connect(&self.url, ewebsock::Options::default())
            .unwrap_or_else(|e| panic!("rosbridge: cannot start connection to {}: {e}", self.url));
        tracing::info!("rosbridge: connecting to {}", self.url);
        let (outbox_tx, outbox_rx) = mpsc::channel();
        app.insert_non_send_resource(RosbridgeIo {
            sender,
            receiver,
            outbox_rx,
        });
        app.insert_resource(RosbridgeSession {
            outbox: Outbox(outbox_tx),
            inboxes: HashMap::new(),
            connected: false,
            topics_poll: Timer::from_seconds(2.0, TimerMode::Repeating),
        });
        if !app.world().contains_resource::<MessageRegistry>() {
            app.init_resource::<MessageRegistry>();
        }
        app.add_systems(
            Update,
            (
                pump_socket,
                manage_topic_io,
                poll_subscription_values,
                handle_publish_requests,
                feed_robot_from_topics,
            )
                .chain(),
        );
    }
}

/// Drain WebSocket events: connection state, topic-list responses, and
/// pushed topic data into the matching [`Inbox`].
fn pump_socket(
    mut commands: Commands,
    mut io: NonSendMut<RosbridgeIo>,
    mut session: ResMut<RosbridgeSession>,
    time: Res<Time>,
    topics: Query<(Entity, &TopicInfo)>,
) {
    // Outgoing frames queued by publishers and management systems.
    while let Ok(frame) = io.outbox_rx.try_recv() {
        io.sender.send(ewebsock::WsMessage::Text(frame.to_string()));
    }

    while let Some(event) = io.receiver.try_recv() {
        match event {
            ewebsock::WsEvent::Opened => {
                tracing::info!("rosbridge: connected");
                session.connected = true;
                request_topics(&session.outbox);
            }
            ewebsock::WsEvent::Message(ewebsock::WsMessage::Text(text)) => {
                match serde_json::from_str::<Value>(&text) {
                    Ok(value) => handle_op(&mut commands, &mut session, &topics, value),
                    Err(e) => tracing::warn!("rosbridge: non-JSON frame: {e}"),
                }
            }
            ewebsock::WsEvent::Message(_) => {}
            ewebsock::WsEvent::Error(e) => {
                tracing::warn!("rosbridge: socket error: {e}");
            }
            ewebsock::WsEvent::Closed => {
                tracing::warn!("rosbridge: connection closed");
                session.connected = false;
            }
        }
    }

    if session.connected && session.topics_poll.tick(time.delta()).just_finished() {
        request_topics(&session.outbox);
        // Flush immediately so the request leaves this frame.
        while let Ok(frame) = io.outbox_rx.try_recv() {
            io.sender.send(ewebsock::WsMessage::Text(frame.to_string()));
        }
    }
}

fn request_topics(outbox: &Outbox) {
    outbox.send(json!({
        "op": "call_service",
        "service": "/rosapi/topics",
        "type": "rosapi_msgs/srv/Topics",
        "id": "ros-viz-rs:topics",
    }));
}

fn handle_op(
    commands: &mut Commands,
    session: &mut RosbridgeSession,
    topics: &Query<(Entity, &TopicInfo)>,
    value: Value,
) {
    match value.get("op").and_then(Value::as_str) {
        Some("publish") => {
            let Some(topic) = value.get("topic").and_then(Value::as_str) else {
                return;
            };
            if let (Some(inbox), Some(msg)) = (session.inboxes.get(topic), value.get("msg")) {
                *inbox.0.lock().unwrap_or_else(|p| p.into_inner()) = Some(msg.clone());
            }
        }
        Some("service_response")
            if value.get("id").and_then(Value::as_str) == Some("ros-viz-rs:topics") =>
        {
            let names: Vec<String> = value
                .pointer("/values/topics")
                .and_then(|v| serde_json::from_value(v.clone()).ok())
                .unwrap_or_default();
            let types: Vec<String> = value
                .pointer("/values/types")
                .and_then(|v| serde_json::from_value(v.clone()).ok())
                .unwrap_or_default();
            reconcile_topics(commands, topics, &names, &types);
        }
        Some("status") => {
            let level = value.get("level").and_then(Value::as_str).unwrap_or("?");
            let msg = value.get("msg").and_then(Value::as_str).unwrap_or("");
            tracing::debug!("rosbridge status [{level}]: {msg}");
        }
        _ => {}
    }
}

/// Mirror the rosapi topic list into [`TopicInfo`] entities.
fn reconcile_topics(
    commands: &mut Commands,
    topics: &Query<(Entity, &TopicInfo)>,
    names: &[String],
    types: &[String],
) {
    let existing: HashMap<&str, Entity> = topics
        .iter()
        .map(|(e, info)| (info.topic_name.as_str(), e))
        .collect();
    let mut seen = std::collections::HashSet::new();
    for (name, ty) in names.iter().zip(types.iter()) {
        seen.insert(name.as_str());
        if !existing.contains_key(name.as_str()) {
            commands.spawn(TopicInfo::new(
                name.clone(),
                ros_path_to_type(ty),
                TopicKind::Normal(name.clone()),
            ));
        }
    }
    for (name, entity) in existing {
        if !seen.contains(name) {
            commands.entity(entity).despawn();
        }
    }
}

/// Subscribe and advertise every discovered topic with a registered type,
/// attaching the standard I/O components the UI consumes.
fn manage_topic_io(
    mut commands: Commands,
    mut session: ResMut<RosbridgeSession>,
    registry: Res<MessageRegistry>,
    topics: Query<(Entity, &TopicInfo), Without<Subscription>>,
) {
    if !session.connected {
        return;
    }
    for (entity, info) in topics.iter() {
        if !registry.contains(&info.type_name) {
            continue;
        }
        let inbox = Arc::new(Inbox::default());
        session
            .inboxes
            .insert(info.topic_name.clone(), inbox.clone());
        session.outbox.send(json!({
            "op": "subscribe",
            "topic": info.topic_name,
            "type": ros_type_to_path(&info.type_name),
        }));
        session.outbox.send(json!({
            "op": "advertise",
            "topic": info.topic_name,
            "type": ros_type_to_path(&info.type_name),
        }));
        let default = registry
            .default_value(&info.type_name)
            .unwrap_or(Value::Null);
        commands.entity(entity).insert((
            Subscription(Box::new(InboxSubscription(inbox))),
            TopicValue(None),
            Publisher(Box::new(BridgePublisher {
                outbox: session.outbox.clone(),
                topic: info.topic_name.clone(),
            })),
            TopicEdit(default),
        ));
    }
}

/// Robot lifecycle over rosbridge: parse `/robot_description` values into a
/// pending [`RobotModel`] and feed `/joint_states` into [`JointPositions`].
fn feed_robot_from_topics(
    mut pending: ResMut<crate::scene::PendingRobot>,
    mut joints: ResMut<JointPositions>,
    robots: Query<&crate::scene::RobotHandle>,
    topics: Query<(&TopicInfo, &TopicValue)>,
) {
    for (info, value) in topics.iter() {
        let Some(value) = &value.0 else { continue };
        match info.topic_name.as_str() {
            "/robot_description" => {
                if pending.0.is_some() || !robots.is_empty() {
                    continue;
                }
                let Some(xml) = value.get("data").and_then(Value::as_str) else {
                    continue;
                };
                match RobotModel::from_urdf_str(xml) {
                    Ok(model) => {
                        tracing::info!("rosbridge: received robot '{}'", model.name());
                        pending.0 = Some(Arc::new(model));
                    }
                    Err(e) => tracing::error!("rosbridge: bad /robot_description: {e}"),
                }
            }
            "/joint_states" => {
                let names: Vec<String> = value
                    .get("name")
                    .and_then(|v| serde_json::from_value(v.clone()).ok())
                    .unwrap_or_default();
                let positions: Vec<f64> = value
                    .get("position")
                    .and_then(|v| serde_json::from_value(v.clone()).ok())
                    .unwrap_or_default();
                for (name, position) in names.into_iter().zip(positions) {
                    joints.positions.insert(name, position);
                }
            }
            _ => {}
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn type_name_conversions() {
        assert_eq!(ros_type_to_path("std_msgs/String"), "std_msgs/msg/String");
        assert_eq!(
            ros_type_to_path("sensor_msgs/msg/JointState"),
            "sensor_msgs/msg/JointState"
        );
        assert_eq!(ros_path_to_type("std_msgs/msg/String"), "std_msgs/String");
        assert_eq!(ros_path_to_type("weird"), "weird");
    }

    #[test]
    fn inbox_subscription_delivers_latest_once() {
        let inbox = Arc::new(Inbox::default());
        let subscription = InboxSubscription(inbox.clone());
        assert_eq!(subscription.poll(), None);
        *inbox.0.lock().unwrap() = Some(json!({"data": 1}));
        *inbox.0.lock().unwrap() = Some(json!({"data": 2}));
        assert_eq!(subscription.poll(), Some(json!({"data": 2})));
        assert_eq!(subscription.poll(), None);
    }
}
