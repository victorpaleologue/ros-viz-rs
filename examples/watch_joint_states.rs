use std::env;

use anyhow::{Error, Result};
use bevy::tasks::futures_lite::StreamExt;
use ros2_client::builtin_interfaces::Time;
use ros2_client::{
    Context, MessageTypeName, Name, NodeName, NodeOptions,
    ros2::{QosPolicies, policy::Durability},
};
use ros2_client::{ContextOptions, DEFAULT_SUBSCRIPTION_QOS};
use serde::{Deserialize, Serialize};

trait MessageType {
    fn message_type_name() -> MessageTypeName;
    const MESSAGE_TYPE_STR: &'static str;
}

/// Associate message type to ROS2 message structs.
///
/// This macro takes care of the common implementation details for
/// ROS2 message types, including the `message_type()` method.
macro_rules! impl_message_type {
    ($package:expr, $struct_name:ident) => {
        impl MessageType for $struct_name {
            fn message_type_name() -> MessageTypeName {
                MessageTypeName::new($package, stringify!($struct_name))
            }
            const MESSAGE_TYPE_STR: &'static str = concat!($package, "/", stringify!($struct_name));
        }
    };
}

/// Header message from std_msgs
///
/// http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Header.html
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Header {
    pub stamp: Time,
    pub frame_id: std::string::String,
}
impl_message_type!("std_msgs", Header);

// ====== sensor_msgs ======

/// JointState message from sensor_msgs
///
/// http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/JointState.html
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct JointState {
    pub header: Header,
    pub name: Vec<std::string::String>,
    pub position: Vec<f64>,
    pub velocity: Vec<f64>,
    pub effort: Vec<f64>,
}
impl_message_type!("sensor_msgs", JointState);

#[tokio::main]
async fn main() -> Result<()> {
    // Get domain ID from environment or default to 0
    let domain_id: u16 = env::var("ROS_DOMAIN_ID")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(0);
    println!("Connecting to ROS2 domain {}", domain_id);

    // Create ROS2 context and node
    let context_options = ContextOptions::new().domain_id(domain_id);
    let ctx = Context::with_options(context_options)?;
    let node_name = NodeName::new("/", "joint_states_watcher")?;
    let mut node = ctx.new_node(node_name, NodeOptions::new())?;

    println!("Subscribing to /joint_states...");
    // The /joint_states topic.
    let topic = node.create_topic(
        &Name::new("/", "joint_states")?,
        JointState::message_type_name(),
        &DEFAULT_SUBSCRIPTION_QOS,
    )?;

    // Create subscription
    let subscription = node.create_subscription::<JointState>(&topic, None)?;

    let node_spinner = node.spinner()?;

    println!("Waiting for /joint_states messages...");
    println!("Press Ctrl+C to exit\n");

    let spinning = node_spinner.spin();
    tokio::pin!(spinning);

    let as_stream = subscription.async_stream();
    tokio::pin!(as_stream);

    loop {
        tokio::select! {
            // _ = &mut spinning => return Err(Error::msg("ROS node spin stopped")),
            // Try to take a message
            Some(res) = as_stream.next() => {
                match res {
                    Ok((msg, _info)) => println!("Robot State: \n{msg:?}"),
                    Err(err) => return Err(Error::msg(format!(
                        "Error reading /robot_description: {err:?}"
                    ))),
                }
            }
        }
    }
}
