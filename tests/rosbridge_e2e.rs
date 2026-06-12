//! End-to-end test of the rosbridge backend against a miniature in-process
//! rosbridge server (plain `tungstenite` over a TCP listener).
//!
//! The fake server implements just enough of the
//! [rosbridge v2 protocol](https://github.com/RobotWebTools/rosbridge_suite/blob/ros2/ROSBRIDGE_PROTOCOL.md):
//! it answers the `/rosapi/topics` service call with a robot's topics,
//! accepts `subscribe`/`advertise`, and pushes `/robot_description` and
//! `/joint_states` publishes. The app under test connects with
//! `--rosbridge`, renders headlessly, and the pixels prove the whole
//! JSON-over-WebSocket path: discovery → subscription → URDF → FK → GPU.

use std::net::TcpListener;
use std::sync::{Mutex, OnceLock};
use std::thread;
use std::time::Duration;

use serde_json::{Value, json};

use ros_viz_rs::options::Options;
use ros_viz_rs::vision;

const URDF: &str = include_str!("../test-data/urdf/two_link_planar.urdf");

/// One GPU app at a time per process.
fn gpu_lock() -> std::sync::MutexGuard<'static, ()> {
    static LOCK: OnceLock<Mutex<()>> = OnceLock::new();
    match LOCK.get_or_init(|| Mutex::new(())).lock() {
        Ok(guard) => guard,
        Err(poisoned) => poisoned.into_inner(),
    }
}

/// Serve one rosbridge client: answer the topic-list service, then stream
/// the robot description and joint states.
fn serve_client(stream: std::net::TcpStream) {
    stream
        .set_read_timeout(Some(Duration::from_millis(50)))
        .expect("set timeout");
    let mut ws = tungstenite::accept(stream).expect("websocket handshake");

    let topics = json!({
        "op": "service_response",
        "service": "/rosapi/topics",
        "id": "ros-viz-rs:topics",
        "result": true,
        "values": {
            "topics": ["/robot_description", "/joint_states"],
            "types": ["std_msgs/msg/String", "sensor_msgs/msg/JointState"],
        },
    });

    let mut subscribed_description = false;
    let mut subscribed_joints = false;
    let start = std::time::Instant::now();
    while start.elapsed() < Duration::from_secs(30) {
        match ws.read() {
            Ok(tungstenite::Message::Text(text)) => {
                let Ok(value) = serde_json::from_str::<Value>(&text) else {
                    continue;
                };
                match value.get("op").and_then(Value::as_str) {
                    Some("call_service")
                        if value.get("service").and_then(Value::as_str)
                            == Some("/rosapi/topics") =>
                    {
                        ws.send(tungstenite::Message::text(topics.to_string()))
                            .expect("send topics");
                    }
                    Some("subscribe") => match value.get("topic").and_then(Value::as_str) {
                        Some("/robot_description") => subscribed_description = true,
                        Some("/joint_states") => subscribed_joints = true,
                        _ => {}
                    },
                    _ => {}
                }
            }
            Ok(tungstenite::Message::Close(_)) => break,
            Ok(_) => {}
            Err(tungstenite::Error::Io(e))
                if e.kind() == std::io::ErrorKind::WouldBlock
                    || e.kind() == std::io::ErrorKind::TimedOut =>
            {
                // No client frame; push data for active subscriptions.
                if subscribed_description {
                    let frame = json!({
                        "op": "publish",
                        "topic": "/robot_description",
                        "msg": {"data": URDF},
                    });
                    ws.send(tungstenite::Message::text(frame.to_string())).ok();
                }
                if subscribed_joints {
                    let frame = json!({
                        "op": "publish",
                        "topic": "/joint_states",
                        "msg": {
                            "header": {"stamp": {"sec": 0, "nanosec": 0}, "frame_id": ""},
                            "name": ["joint1", "joint2"],
                            "position": [0.3, 1.2],
                            "velocity": [],
                            "effort": [],
                        },
                    });
                    ws.send(tungstenite::Message::text(frame.to_string())).ok();
                }
                thread::sleep(Duration::from_millis(50));
            }
            Err(_) => break,
        }
    }
}

#[test]
fn robot_renders_over_rosbridge() {
    let _gpu = gpu_lock();

    let listener = TcpListener::bind("127.0.0.1:0").expect("bind");
    let port = listener.local_addr().expect("addr").port();
    thread::spawn(move || {
        for stream in listener.incoming().flatten() {
            thread::spawn(move || serve_client(stream));
        }
    });

    let dir = tempfile::tempdir().expect("tempdir");
    let png = dir.path().join("rosbridge.png");
    let options = Options {
        rosbridge: Some(format!("ws://127.0.0.1:{port}")),
        snapshot_to: Some(png.clone()),
        width: 640,
        height: 480,
        ..Options::default()
    };
    ros_viz_rs::app::run(options).expect("app renders the rosbridge robot");

    let img = image::open(&png).expect("snapshot readable").to_rgba8();
    let background = *img.get_pixel(0, 0);
    let silhouette = vision::silhouette(&img, background, 12);
    assert!(
        silhouette.coverage > 0.002,
        "robot should be visible over rosbridge (coverage {:.4}%)",
        silhouette.coverage * 100.0
    );
}
