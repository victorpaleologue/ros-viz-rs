use ros2_client::{
    Context, DEFAULT_SUBSCRIPTION_QOS, Message, MessageTypeName, Name, Node, NodeName, NodeOptions,
    Subscription,
    builtin_interfaces::Time,
    ros2::{
        Duration as RosDuration, QosPolicyBuilder,
        policy::{Durability, History, Reliability},
    },
};
use serde::{Deserialize, Serialize};
use tracing::debug;

pub const TOPIC_ROBOT_DESCRIPTION: &str = "/robot_description";
pub const TOPIC_JOINT_STATES: &str = "/joint_states";
pub const TOPIC_JOINT_COMMANDS: &str = "/joint_commands";

#[derive(Debug, Clone)]
pub struct RosConfig {
    pub domain_id: u32,
    pub node_name: String,
    pub namespace: Option<String>,
}

impl RosConfig {
    pub fn new(domain_id: u32) -> Self {
        Self {
            domain_id,
            node_name: "ros_viz_rs".to_string(),
            namespace: None,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Header {
    pub stamp: Time,
    pub frame_id: String,
}

impl Default for Header {
    fn default() -> Self {
        Self {
            stamp: Time::from_nanos(0),
            frame_id: String::default(),
        }
    }
}

#[derive(Default, Debug, Clone, Serialize, Deserialize)]
pub struct JointStateMsg {
    pub header: Header,
    pub name: Vec<String>,
    pub position: Vec<f64>,
    pub velocity: Vec<f64>,
    pub effort: Vec<f64>,
}

impl Message for JointStateMsg {}

pub struct RosHandle {
    _node: Node,
    domain_id: u32,
    robot_description_sub: Subscription<String>,
    joint_state_sub: Subscription<JointStateMsg>,
}

impl std::fmt::Debug for RosHandle {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("RosHandle").finish()
    }
}

/// Establish ROS2 connectivity. When the `ros` feature is disabled, this returns an error so CI can run without ROS2.
pub fn connect(config: &RosConfig) -> anyhow::Result<RosHandle> {
    let ctx = Context::new()?;

    let node_name = NodeName::new(
        config.namespace.as_ref().map(|s| s.as_str()).unwrap_or("/"),
        &config.node_name,
    )?;
    let mut node = ctx.new_node(node_name, NodeOptions::new())?;

    // Use TransientLocal QoS for robot_description to receive latched messages
    // (messages published before we subscribed)
    let robot_description_qos = QosPolicyBuilder::new()
        .durability(Durability::TransientLocal)
        .history(History::KeepLast { depth: 1 })
        .reliability(Reliability::Reliable {
            max_blocking_time: RosDuration::from_secs(1),
        })
        .build();

    let robot_description_topic = node.create_topic(
        &Name::new("/", "robot_description")?,
        MessageTypeName::new("std_msgs", "String"),
        &robot_description_qos,
    )?;

    let joint_states_topic = node.create_topic(
        &Name::new("/", "joint_states")?,
        MessageTypeName::new("sensor_msgs", "JointState"),
        &DEFAULT_SUBSCRIPTION_QOS,
    )?;

    let robot_description_sub =
        node.create_subscription::<String>(&robot_description_topic, None)?;
    let joint_state_sub = node.create_subscription::<JointStateMsg>(&joint_states_topic, None)?;

    Ok(RosHandle {
        _node: node,
        domain_id: config.domain_id,
        robot_description_sub,
        joint_state_sub,
    })
}

impl RosHandle {
    pub fn domain_id(&self) -> u32 {
        self.domain_id
    }

    pub fn try_take_robot_description(&self) -> anyhow::Result<Option<String>> {
        match self.robot_description_sub.take() {
            Ok(Some((msg, _info))) => {
                debug!(
                    "received robot_description message, length: {} bytes",
                    msg.len()
                );
                Ok(Some(msg))
            }
            Ok(None) => Ok(None),
            Err(err) => Err(anyhow::anyhow!("read robot_description failed: {err:?}")),
        }
    }

    pub fn try_take_joint_states(&self) -> anyhow::Result<Option<JointStateMsg>> {
        match self.joint_state_sub.take() {
            Ok(Some((msg, _info))) => {
                debug!("received joint_states message");
                Ok(Some(msg))
            }
            Ok(None) => Ok(None),
            Err(err) => Err(anyhow::anyhow!("read joint_states failed: {err:?}")),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ros_connect() {
        let cfg = RosConfig::new(0);
        let handle = connect(&cfg).expect("connect ros");
        assert_eq!(handle.domain_id(), 0);
    }
}
