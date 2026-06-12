//! Message type definitions for common ROS 2 messages.
//!
//! Provides Rust structs corresponding to standard ROS 2 message types
//! together with a [`MessageType`] trait that maps each struct to its
//! package-qualified type name (e.g. `"std_msgs/String"`).

use ros2_client::builtin_interfaces::Time;
use serde::{Deserialize, Serialize};
use std::fmt::Debug;

// ---------------------------------------------------------------------------
// Trait
// ---------------------------------------------------------------------------

/// A type that can be used as a ROS 2 message.
pub trait MessageType:
    Clone + Debug + Send + Sync + Serialize + serde::de::DeserializeOwned + 'static
{
    /// The ROS 2 message type name in `"package/Type"` format.
    const MESSAGE_TYPE_STR: &'static str;

    /// Build a [`ros2_client::MessageTypeName`] for DDS topic creation.
    fn message_type_name() -> ros2_client::MessageTypeName;
}

/// Implement [`MessageType`] for a struct defined in this module.
macro_rules! impl_message_type {
    ($package:expr, $struct_name:ident) => {
        impl MessageType for $struct_name {
            const MESSAGE_TYPE_STR: &'static str = concat!($package, "/", stringify!($struct_name));
            fn message_type_name() -> ros2_client::MessageTypeName {
                ros2_client::MessageTypeName::new($package, stringify!($struct_name))
            }
        }
    };
}

// ====== std_msgs =========================================================

/// `std_msgs/String`
#[derive(Default, Debug, Clone, Serialize, Deserialize)]
pub struct String {
    pub data: std::string::String,
}
impl_message_type!("std_msgs", String);

/// `std_msgs/Bool`
#[derive(Default, Debug, Clone, Serialize, Deserialize)]
pub struct Bool {
    pub data: bool,
}
impl_message_type!("std_msgs", Bool);

/// `std_msgs/Int32`
#[derive(Default, Debug, Clone, Serialize, Deserialize)]
pub struct Int32 {
    pub data: i32,
}
impl_message_type!("std_msgs", Int32);

/// `std_msgs/UInt32`
#[derive(Default, Debug, Clone, Serialize, Deserialize)]
pub struct UInt32 {
    pub data: u32,
}
impl_message_type!("std_msgs", UInt32);

/// `std_msgs/Float64`
#[derive(Default, Debug, Clone, Serialize, Deserialize)]
pub struct Float64 {
    pub data: f64,
}
impl_message_type!("std_msgs", Float64);

/// `std_msgs/Empty`
#[derive(Default, Debug, Clone, Serialize, Deserialize)]
pub struct Empty {}
impl_message_type!("std_msgs", Empty);

/// `std_msgs/Header`
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Header {
    pub stamp: Time,
    pub frame_id: std::string::String,
}
impl_message_type!("std_msgs", Header);

impl Default for Header {
    fn default() -> Self {
        Self {
            stamp: Time::from_nanos(0),
            frame_id: std::string::String::default(),
        }
    }
}

/// `std_msgs/MultiArrayDimension`
#[derive(Default, Debug, Clone, Serialize, Deserialize)]
pub struct MultiArrayDimension {
    pub label: std::string::String,
    pub size: u32,
    pub stride: u32,
}
impl_message_type!("std_msgs", MultiArrayDimension);

/// `std_msgs/MultiArrayLayout`
#[derive(Default, Debug, Clone, Serialize, Deserialize)]
pub struct MultiArrayLayout {
    pub dim: Vec<MultiArrayDimension>,
    pub data_offset: u32,
}
impl_message_type!("std_msgs", MultiArrayLayout);

/// `std_msgs/Float64MultiArray`
#[derive(Default, Debug, Clone, Serialize, Deserialize)]
pub struct Float64MultiArray {
    pub layout: MultiArrayLayout,
    pub data: Vec<f64>,
}
impl_message_type!("std_msgs", Float64MultiArray);

// ====== sensor_msgs ======================================================

/// `sensor_msgs/JointState`
#[derive(Default, Debug, Clone, Serialize, Deserialize)]
pub struct JointState {
    pub header: Header,
    pub name: Vec<std::string::String>,
    pub position: Vec<f64>,
    pub velocity: Vec<f64>,
    pub effort: Vec<f64>,
}
impl_message_type!("sensor_msgs", JointState);

// ====== geometry_msgs ====================================================

/// `geometry_msgs/Point`
#[derive(Default, Debug, Clone, Serialize, Deserialize)]
pub struct Point {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}
impl_message_type!("geometry_msgs", Point);

/// `geometry_msgs/Quaternion`
#[derive(Default, Debug, Clone, Serialize, Deserialize)]
pub struct Quaternion {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub w: f64,
}
impl_message_type!("geometry_msgs", Quaternion);

/// `geometry_msgs/Pose`
#[derive(Default, Debug, Clone, Serialize, Deserialize)]
pub struct Pose {
    pub position: Point,
    pub orientation: Quaternion,
}
impl_message_type!("geometry_msgs", Pose);

/// `geometry_msgs/PoseStamped`
#[derive(Default, Debug, Clone, Serialize, Deserialize)]
pub struct PoseStamped {
    pub header: Header,
    pub pose: Pose,
}
impl_message_type!("geometry_msgs", PoseStamped);

/// `geometry_msgs/Vector3`
#[derive(Default, Debug, Clone, Serialize, Deserialize)]
pub struct Vector3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}
impl_message_type!("geometry_msgs", Vector3);

/// `geometry_msgs/Twist`
#[derive(Default, Debug, Clone, Serialize, Deserialize)]
pub struct Twist {
    pub linear: Vector3,
    pub angular: Vector3,
}
impl_message_type!("geometry_msgs", Twist);
