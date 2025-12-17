use crate::config::AppConfig;

#[cfg(feature = "ros")]
use ros2_client::{
    context::DEFAULT_PUBLISHER_QOS,
    context::DEFAULT_SUBSCRIPTION_QOS,
    Context, ContextOptions, Message, MessageTypeName, Subscription,
};
#[cfg(feature = "ros")]
use serde::{Deserialize, Serialize};
#[cfg(feature = "ros")]
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
    pub fn from_app(app: &AppConfig) -> Self {
        Self {
            domain_id: app.domain_id,
            node_name: "ros_viz_rs".to_string(),
            namespace: None,
        }
    }
}

#[cfg(not(feature = "ros"))]
#[derive(Debug)]
pub struct RosHandle;

#[cfg(feature = "ros")]
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct JointStateMsg {
    pub names: Vec<String>,
    pub positions: Vec<f64>,
}

#[cfg(feature = "ros")]
impl Message for JointStateMsg {}

#[cfg(feature = "ros")]
#[derive(Debug)]
pub struct RosHandle {
    ctx: Context,
    robot_description_pub: ros2_client::Publisher<String>,
    joint_state_pub: ros2_client::Publisher<JointStateMsg>,
    joint_command_sub: Subscription<JointStateMsg>,
}

/// Establish ROS2 connectivity. When the `ros` feature is disabled, this returns an error so CI can run without ROS2.
#[cfg(feature = "ros")]
pub fn connect(config: &RosConfig) -> anyhow::Result<RosHandle> {
    let ctx = Context::with_options(ContextOptions::new().domain_id(config.domain_id as u16))?;

    let robot_description_topic = ctx.create_topic(
        TOPIC_ROBOT_DESCRIPTION.to_string(),
        MessageTypeName::new("std_msgs", "String"),
        &DEFAULT_PUBLISHER_QOS,
    )?;

    let joint_states_topic = ctx.create_topic(
        TOPIC_JOINT_STATES.to_string(),
        MessageTypeName::new("sensor_msgs", "JointState"),
        &DEFAULT_PUBLISHER_QOS,
    )?;

    let joint_commands_topic = ctx.create_topic(
        TOPIC_JOINT_COMMANDS.to_string(),
        MessageTypeName::new("sensor_msgs", "JointState"),
        &DEFAULT_SUBSCRIPTION_QOS,
    )?;

    let robot_description_pub = ctx.create_publisher::<String>(&robot_description_topic, None)?;
    let joint_state_pub = ctx.create_publisher::<JointStateMsg>(&joint_states_topic, None)?;
    let joint_command_sub = ctx.create_subscription::<JointStateMsg>(&joint_commands_topic, None)?;

    Ok(RosHandle {
        ctx,
        robot_description_pub,
        joint_state_pub,
        joint_command_sub,
    })
}

/// Establish ROS2 connectivity. When the `ros` feature is disabled, this returns an error so CI can run without ROS2.
#[cfg(not(feature = "ros"))]
pub fn connect(_config: &RosConfig) -> anyhow::Result<RosHandle> {
    Err(anyhow::anyhow!(
        "Build with --features ros to enable ROS2 connectivity"
    ))
}

#[cfg(feature = "ros")]
impl RosHandle {
    pub fn publish_robot_description(&self, urdf_xml: &str) -> anyhow::Result<()> {
        self.robot_description_pub
            .publish(urdf_xml.to_string())
            .map_err(|e| anyhow::anyhow!("publish robot_description failed: {e:?}"))?;
        Ok(())
    }

    pub fn publish_joint_states(&self, msg: JointStateMsg) -> anyhow::Result<()> {
        self.joint_state_pub
            .publish(msg)
            .map_err(|e| anyhow::anyhow!("publish joint_states failed: {e:?}"))?;
        Ok(())
    }

    /// Attempts to take a joint command message if available.
    pub fn try_take_joint_commands(&self) -> anyhow::Result<Option<JointStateMsg>> {
        match self.joint_command_sub.take() {
            Ok(Some((msg, _info))) => {
                debug!("received joint command message");
                Ok(Some(msg))
            }
            Ok(None) => Ok(None),
            Err(err) => Err(anyhow::anyhow!("read joint_commands failed: {err:?}")),
        }
    }

    pub(crate) fn joint_command_publisher(
        &self,
    ) -> anyhow::Result<ros2_client::Publisher<JointStateMsg>> {
        let topic = self.ctx.create_topic(
            TOPIC_JOINT_COMMANDS.to_string(),
            MessageTypeName::new("sensor_msgs", "JointState"),
            &DEFAULT_PUBLISHER_QOS,
        )?;
        self.ctx
            .create_publisher::<JointStateMsg>(&topic, None)
            .map_err(|e| anyhow::anyhow!("create joint_commands publisher failed: {e:?}"))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stub_connect_errors_without_feature() {
        let cfg = RosConfig::from_app(&AppConfig::new(0));
        let err = connect(&cfg).unwrap_err();
        assert!(
            err.to_string().contains("ros"),
            "error should mention ros feature"
        );
    }
}

#[cfg(all(test, feature = "ros"))]
mod feature_tests {
    use super::*;
    use std::thread::sleep;
    use std::time::{Duration, Instant};

    #[test]
    fn loopback_joint_commands() {
        let cfg = RosConfig::from_app(&AppConfig::new(0));
        let handle = connect(&cfg).expect("connect ros");
        let publisher = handle
            .joint_command_publisher()
            .expect("create joint_commands publisher");

        let msg = JointStateMsg {
            names: vec!["shoulder".to_string()],
            positions: vec![1.0],
        };
        publisher.publish(msg).expect("publish joint_commands");

        let start = Instant::now();
        let mut received = None;
        while start.elapsed() < Duration::from_millis(500) {
            match handle.try_take_joint_commands() {
                Ok(Some(cmds)) => {
                    received = Some(cmds);
                    break;
                }
                Ok(None) => sleep(Duration::from_millis(10)),
                Err(err) => panic!("read joint_commands failed: {err:?}"),
            }
        }

        let msg = received.expect("expected joint_commands message");
        assert_eq!(msg.names, vec!["shoulder".to_string()]);
        assert_eq!(msg.positions, vec![1.0]);
    }
}
