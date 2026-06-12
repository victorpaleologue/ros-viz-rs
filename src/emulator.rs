//! A fake robot publishing over real DDS, for tests and demos.
//!
//! [`Emulator`] owns a ROS 2 node that latches the URDF on
//! `/robot_description`, streams `/joint_states`, and applies any
//! `/joint_commands` it receives — the same surface as
//! `robot_state_publisher` + a joint-state source, so the visualizer can be
//! exercised end-to-end (transport → URDF → kinematics → pixels) without a
//! robot or a ROS installation.
//!
//! Joint motion comes from a [`JointScript`]: a function of elapsed seconds
//! returning joint positions. [`scripts`] provides ready-made ones.

use std::collections::BTreeMap;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use ros2_client::ros2::{
    Duration as RosDuration, QosPolicyBuilder,
    policy::{Durability, History, Reliability},
};

use crate::ros_msgs;
use crate::topics_io::{setup_typed_publisher, setup_typed_subscription};

pub use crate::demo::{JointScript, scripts};

/// Configuration for [`Emulator::spawn`].
pub struct EmulatorConfig {
    /// DDS domain to publish on.
    pub domain: u16,
    /// URDF to latch on `/robot_description`.
    pub urdf_xml: String,
    /// Initial joint positions.
    pub initial_joints: BTreeMap<String, f64>,
    /// Motion script, sampled at `rate_hz`; `None` keeps initial positions.
    pub script: Option<JointScript>,
    /// `/joint_states` publication rate.
    pub rate_hz: f64,
}

impl EmulatorConfig {
    pub fn new(domain: u16, urdf_xml: impl Into<String>) -> Self {
        Self {
            domain,
            urdf_xml: urdf_xml.into(),
            initial_joints: BTreeMap::new(),
            script: None,
            rate_hz: 30.0,
        }
    }

    pub fn with_script(mut self, script: JointScript) -> Self {
        self.script = Some(script);
        self
    }

    pub fn with_initial_joints(mut self, joints: impl IntoIterator<Item = (String, f64)>) -> Self {
        self.initial_joints.extend(joints);
        self
    }
}

/// A running emulated robot; stops when dropped.
pub struct Emulator {
    stop: Arc<AtomicBool>,
    thread: Option<JoinHandle<()>>,
    joints: Arc<Mutex<BTreeMap<String, f64>>>,
    /// The DDS context and node must outlive the endpoints: dropping the
    /// node tears down the machinery reliable/transient-local delivery
    /// rides on (best-effort topics keep flowing, which makes the failure
    /// mode delightfully subtle).
    _node: Arc<Mutex<ros2_client::Node>>,
    _ctx: ros2_client::Context,
}

impl Emulator {
    /// Start the emulated robot on its own node and background thread.
    pub fn spawn(config: EmulatorConfig) -> Result<Self, String> {
        let ctx = ros2_client::Context::with_options(
            ros2_client::ContextOptions::new().domain_id(config.domain),
        )
        .map_err(|e| format!("failed to create ROS context: {e:?}"))?;
        let node_name = ros2_client::NodeName::new("/", "ros_viz_emulator")
            .map_err(|e| format!("invalid node name: {e:?}"))?;
        let mut node = ctx
            .new_node(node_name, ros2_client::NodeOptions::new())
            .map_err(|e| format!("failed to create node: {e:?}"))?;

        let spinner = node
            .spinner()
            .map_err(|e| format!("failed to create spinner: {e:?}"))?;
        std::thread::Builder::new()
            .name("emulator_spinner".into())
            .spawn(move || {
                let _ = futures::executor::block_on(spinner.spin());
            })
            .map_err(|e| format!("failed to spawn spinner thread: {e}"))?;

        let node = Arc::new(Mutex::new(node));

        // Latched, like robot_state_publisher publishes the description:
        // late subscribers still receive the URDF.
        let latched = QosPolicyBuilder::new()
            .durability(Durability::TransientLocal)
            .history(History::KeepLast { depth: 1 })
            .reliability(Reliability::Reliable {
                max_blocking_time: RosDuration::from_secs(1),
            })
            .build();
        let description_pub =
            setup_typed_publisher::<ros_msgs::String>(&node, "/robot_description", Some(&latched))?;
        let joint_states_pub =
            setup_typed_publisher::<ros_msgs::JointState>(&node, "/joint_states", None)?;
        let commands_sub =
            setup_typed_subscription::<ros_msgs::JointState>(&node, "/joint_commands", None)?;

        description_pub
            .publish(ros_msgs::String {
                data: config.urdf_xml.clone(),
            })
            .map_err(|e| format!("failed to publish robot_description: {e:?}"))?;
        let urdf_xml = config.urdf_xml;

        let joints = Arc::new(Mutex::new(config.initial_joints));
        let stop = Arc::new(AtomicBool::new(false));

        let thread = {
            let joints = joints.clone();
            let stop = stop.clone();
            let mut script = config.script;
            let period = Duration::from_secs_f64(1.0 / config.rate_hz.max(0.1));
            std::thread::Builder::new()
                .name("emulator_loop".into())
                .spawn(move || {
                    let start = Instant::now();
                    let mut last_description = Instant::now();
                    while !stop.load(Ordering::Relaxed) {
                        // The description is latched (TransientLocal), but
                        // republishing every second also covers transports
                        // where historical delivery to late joiners is
                        // unreliable; receivers parse it only once.
                        if last_description.elapsed() > Duration::from_secs(1) {
                            last_description = Instant::now();
                            if let Err(e) = description_pub.publish(ros_msgs::String {
                                data: urdf_xml.clone(),
                            }) {
                                tracing::warn!("emulator description republish failed: {e:?}");
                            }
                        }
                        // External commands take effect first, scripts after,
                        // so a script owns the joints it animates.
                        while let Ok(Some((msg, _))) = commands_sub.take() {
                            let mut joints = joints.lock().unwrap_or_else(|p| p.into_inner());
                            for (name, pos) in msg.name.iter().zip(msg.position.iter()) {
                                joints.insert(name.clone(), *pos);
                            }
                        }
                        if let Some(script) = script.as_mut() {
                            let t = start.elapsed().as_secs_f64();
                            let mut joints = joints.lock().unwrap_or_else(|p| p.into_inner());
                            for (name, pos) in script(t) {
                                joints.insert(name, pos);
                            }
                        }
                        let (names, positions): (Vec<_>, Vec<_>) = {
                            let joints = joints.lock().unwrap_or_else(|p| p.into_inner());
                            joints.iter().map(|(n, p)| (n.clone(), *p)).unzip()
                        };
                        let msg = ros_msgs::JointState {
                            header: Default::default(),
                            name: names,
                            position: positions,
                            velocity: vec![],
                            effort: vec![],
                        };
                        if let Err(e) = joint_states_pub.publish(msg) {
                            tracing::debug!("emulator joint_states publish failed: {e:?}");
                        }
                        std::thread::sleep(period);
                    }
                    // Keep the description publisher alive until shutdown so
                    // TransientLocal delivery keeps working for late joiners.
                    drop(description_pub);
                })
                .map_err(|e| format!("failed to spawn emulator thread: {e}"))?
        };

        Ok(Self {
            stop,
            thread: Some(thread),
            joints,
            _node: node,
            _ctx: ctx,
        })
    }

    /// Current joint positions (sorted by name).
    pub fn joint_positions(&self) -> BTreeMap<String, f64> {
        self.joints
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .clone()
    }

    /// Stop publishing and join the background thread.
    pub fn stop(mut self) {
        self.shutdown();
    }

    fn shutdown(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(thread) = self.thread.take() {
            let _ = thread.join();
        }
    }
}

impl Drop for Emulator {
    fn drop(&mut self) {
        self.shutdown();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Random DDS domain in 1..=232, isolating tests from each other and
    /// from any real ROS system on domain 0.
    fn random_domain_id() -> u16 {
        use std::collections::hash_map::RandomState;
        use std::hash::{BuildHasher, Hasher};
        let hash = RandomState::new().build_hasher().finish();
        (hash % 232 + 1) as u16
    }

    const URDF: &str = include_str!("../test-data/urdf/two_link_planar.urdf");

    #[test]
    fn emulator_publishes_description_and_joint_states() {
        crate::require_dds_multicast!();
        let domain = random_domain_id();
        let emulator = Emulator::spawn(
            EmulatorConfig::new(domain, URDF)
                .with_initial_joints([("joint1".to_string(), 0.25)])
                .with_script(scripts::static_pose(vec![("joint2".into(), 0.5)])),
        )
        .expect("emulator starts");

        // Subscribe like the app does.
        let ctx = ros2_client::Context::with_options(
            ros2_client::ContextOptions::new().domain_id(domain),
        )
        .expect("context");
        let mut node = ctx
            .new_node(
                ros2_client::NodeName::new("/", "emulator_probe").unwrap(),
                ros2_client::NodeOptions::new(),
            )
            .expect("node");
        let spinner = node.spinner().expect("spinner");
        std::thread::spawn(move || {
            let _ = futures::executor::block_on(spinner.spin());
        });
        let node = Arc::new(Mutex::new(node));

        let latched = QosPolicyBuilder::new()
            .durability(Durability::TransientLocal)
            .history(History::KeepLast { depth: 1 })
            .reliability(Reliability::Reliable {
                max_blocking_time: RosDuration::from_secs(1),
            })
            .build();
        let description_sub = setup_typed_subscription::<ros_msgs::String>(
            &node,
            "/robot_description",
            Some(&latched),
        )
        .expect("description sub");
        let joints_sub =
            setup_typed_subscription::<ros_msgs::JointState>(&node, "/joint_states", None)
                .expect("joints sub");

        let deadline = Instant::now() + Duration::from_secs(30);
        let mut got_description = false;
        let mut got_joints = false;
        while Instant::now() < deadline && !(got_description && got_joints) {
            if let Ok(Some((msg, _))) = description_sub.take()
                && msg.data == URDF
            {
                got_description = true;
            }
            if let Ok(Some((msg, _))) = joints_sub.take() {
                let pairs: BTreeMap<_, _> =
                    msg.name.iter().cloned().zip(msg.position.clone()).collect();
                if pairs.get("joint1") == Some(&0.25) && pairs.get("joint2") == Some(&0.5) {
                    got_joints = true;
                }
            }
            std::thread::sleep(Duration::from_millis(50));
        }
        emulator.stop();
        assert!(got_description, "robot_description latched and received");
        assert!(got_joints, "joint_states received with scripted positions");
    }
}
