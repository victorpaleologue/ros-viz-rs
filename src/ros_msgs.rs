//! Message type definitions for common ROS 2 messages.
//!
//! Provides Rust structs corresponding to standard ROS 2 message types
//! together with a [`MessageType`] trait that maps each struct to its
//! package-qualified type name (e.g. `"std_msgs/String"`).
//!
//! Field names, types and **order** match the official `.msg` definitions
//! from <https://github.com/ros2/common_interfaces> (rolling branch) and
//! <https://github.com/ros2/geometry2> (`tf2_msgs`); order matters because
//! CDR encodes struct fields sequentially.
//!
//! Fixed-size arrays (e.g. `float64[36]` covariance) are represented as Rust
//! arrays, which serde serializes as tuples — exactly the CDR encoding of a
//! fixed array (no length prefix), unlike `Vec<T>`, which serde serializes as
//! a sequence (length-prefixed). Arrays longer than 32 elements need
//! [`serde_big_array::BigArray`] because serde only derives `Deserialize`
//! for arrays up to 32 elements.

use serde::{Deserialize, Serialize};
use serde_big_array::BigArray;
use std::fmt::Debug;

// ---------------------------------------------------------------------------
// Trait
// ---------------------------------------------------------------------------

/// A type that can be used as a ROS 2 message.
///
/// Transport-free: DDS-specific name types are derived from
/// [`PACKAGE`](Self::PACKAGE)/[`TYPE_NAME`](Self::TYPE_NAME) by the `ros2`
/// backend, and rosbridge uses [`MESSAGE_TYPE_STR`](Self::MESSAGE_TYPE_STR)
/// directly.
pub trait MessageType:
    Clone + Debug + Send + Sync + Serialize + serde::de::DeserializeOwned + 'static
{
    /// The ROS package the message belongs to, e.g. `"std_msgs"`.
    const PACKAGE: &'static str;

    /// The bare type name, e.g. `"String"`.
    const TYPE_NAME: &'static str;

    /// The ROS 2 message type name in `"package/Type"` format.
    const MESSAGE_TYPE_STR: &'static str;

    /// Build a [`ros2_client::MessageTypeName`] for DDS topic creation.
    #[cfg(feature = "ros2")]
    fn message_type_name() -> ros2_client::MessageTypeName {
        ros2_client::MessageTypeName::new(Self::PACKAGE, Self::TYPE_NAME)
    }
}

/// Implement [`MessageType`] for a struct defined in this module.
macro_rules! impl_message_type {
    ($package:expr, $struct_name:ident) => {
        impl MessageType for $struct_name {
            const PACKAGE: &'static str = $package;
            const TYPE_NAME: &'static str = stringify!($struct_name);
            const MESSAGE_TYPE_STR: &'static str = concat!($package, "/", stringify!($struct_name));
        }
    };
}

// ====== builtin_interfaces ===============================================
// https://github.com/ros2/rcl_interfaces/tree/rolling/builtin_interfaces

/// `builtin_interfaces/Time` — same wire layout as
/// `ros2_client::builtin_interfaces::Time` (int32 sec, uint32 nanosec),
/// defined here so messages stay transport-free.
#[derive(Default, Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct Time {
    pub sec: i32,
    pub nanosec: u32,
}

impl Time {
    /// Build from nanoseconds since the UNIX epoch.
    pub fn from_nanos(nanos: i64) -> Self {
        Self {
            sec: (nanos / 1_000_000_000) as i32,
            nanosec: (nanos % 1_000_000_000) as u32,
        }
    }
}
impl_message_type!("builtin_interfaces", Time);

// ====== std_msgs =========================================================
// https://github.com/ros2/common_interfaces/tree/rolling/std_msgs/msg

/// `std_msgs/String`
#[derive(Default, Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct String {
    pub data: std::string::String,
}
impl_message_type!("std_msgs", String);

/// `std_msgs/Bool`
#[derive(Default, Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Bool {
    pub data: bool,
}
impl_message_type!("std_msgs", Bool);

/// `std_msgs/Char` — ROS `char` is an unsigned 8-bit integer.
#[derive(Default, Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Char {
    pub data: u8,
}
impl_message_type!("std_msgs", Char);

/// `std_msgs/Byte`
#[derive(Default, Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Byte {
    pub data: u8,
}
impl_message_type!("std_msgs", Byte);

/// `std_msgs/Int8`
#[derive(Default, Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Int8 {
    pub data: i8,
}
impl_message_type!("std_msgs", Int8);

/// `std_msgs/Int16`
#[derive(Default, Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Int16 {
    pub data: i16,
}
impl_message_type!("std_msgs", Int16);

/// `std_msgs/Int32`
#[derive(Default, Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Int32 {
    pub data: i32,
}
impl_message_type!("std_msgs", Int32);

/// `std_msgs/Int64`
#[derive(Default, Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Int64 {
    pub data: i64,
}
impl_message_type!("std_msgs", Int64);

/// `std_msgs/UInt8`
#[derive(Default, Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct UInt8 {
    pub data: u8,
}
impl_message_type!("std_msgs", UInt8);

/// `std_msgs/UInt16`
#[derive(Default, Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct UInt16 {
    pub data: u16,
}
impl_message_type!("std_msgs", UInt16);

/// `std_msgs/UInt32`
#[derive(Default, Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct UInt32 {
    pub data: u32,
}
impl_message_type!("std_msgs", UInt32);

/// `std_msgs/UInt64`
#[derive(Default, Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct UInt64 {
    pub data: u64,
}
impl_message_type!("std_msgs", UInt64);

/// `std_msgs/Float32`
#[derive(Default, Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Float32 {
    pub data: f32,
}
impl_message_type!("std_msgs", Float32);

/// `std_msgs/Float64`
#[derive(Default, Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Float64 {
    pub data: f64,
}
impl_message_type!("std_msgs", Float64);

/// `std_msgs/Empty`
#[derive(Default, Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Empty {}
impl_message_type!("std_msgs", Empty);

/// `std_msgs/ColorRGBA`
#[derive(Default, Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ColorRGBA {
    pub r: f32,
    pub g: f32,
    pub b: f32,
    pub a: f32,
}
impl_message_type!("std_msgs", ColorRGBA);

/// `std_msgs/Header`
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
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
#[derive(Default, Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MultiArrayDimension {
    pub label: std::string::String,
    pub size: u32,
    pub stride: u32,
}
impl_message_type!("std_msgs", MultiArrayDimension);

/// `std_msgs/MultiArrayLayout`
#[derive(Default, Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MultiArrayLayout {
    pub dim: Vec<MultiArrayDimension>,
    pub data_offset: u32,
}
impl_message_type!("std_msgs", MultiArrayLayout);

/// `std_msgs/Float64MultiArray`
#[derive(Default, Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Float64MultiArray {
    pub layout: MultiArrayLayout,
    pub data: Vec<f64>,
}
impl_message_type!("std_msgs", Float64MultiArray);

// ====== geometry_msgs ====================================================
// https://github.com/ros2/common_interfaces/tree/rolling/geometry_msgs/msg

/// `geometry_msgs/Point`
#[derive(Default, Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Point {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}
impl_message_type!("geometry_msgs", Point);

/// `geometry_msgs/Point32`
#[derive(Default, Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Point32 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}
impl_message_type!("geometry_msgs", Point32);

/// `geometry_msgs/Vector3`
#[derive(Default, Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Vector3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}
impl_message_type!("geometry_msgs", Vector3);

/// `geometry_msgs/Quaternion`
///
/// Defaults to the identity rotation (`w = 1`), matching the field default
/// `float64 w 1` in the official definition.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Quaternion {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub w: f64,
}
impl_message_type!("geometry_msgs", Quaternion);

impl Default for Quaternion {
    fn default() -> Self {
        Self {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 1.0,
        }
    }
}

/// `geometry_msgs/Pose`
#[derive(Default, Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Pose {
    pub position: Point,
    pub orientation: Quaternion,
}
impl_message_type!("geometry_msgs", Pose);

/// `geometry_msgs/Pose2D` (deprecated upstream, but still commonly used)
#[derive(Default, Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Pose2D {
    pub x: f64,
    pub y: f64,
    pub theta: f64,
}
impl_message_type!("geometry_msgs", Pose2D);

/// `geometry_msgs/PoseStamped`
#[derive(Default, Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PoseStamped {
    pub header: Header,
    pub pose: Pose,
}
impl_message_type!("geometry_msgs", PoseStamped);

/// `geometry_msgs/PoseArray`
#[derive(Default, Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PoseArray {
    pub header: Header,
    pub poses: Vec<Pose>,
}
impl_message_type!("geometry_msgs", PoseArray);

/// `geometry_msgs/PoseWithCovariance`
///
/// `covariance` is `float64[36]`: a fixed array, hence `[f64; 36]` and not
/// `Vec<f64>` (CDR sequences carry a length prefix that fixed arrays lack).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PoseWithCovariance {
    pub pose: Pose,
    #[serde(with = "BigArray")]
    pub covariance: [f64; 36],
}
impl_message_type!("geometry_msgs", PoseWithCovariance);

impl Default for PoseWithCovariance {
    fn default() -> Self {
        Self {
            pose: Pose::default(),
            covariance: [0.0; 36],
        }
    }
}

/// `geometry_msgs/Twist`
#[derive(Default, Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Twist {
    pub linear: Vector3,
    pub angular: Vector3,
}
impl_message_type!("geometry_msgs", Twist);

/// `geometry_msgs/TwistStamped`
#[derive(Default, Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TwistStamped {
    pub header: Header,
    pub twist: Twist,
}
impl_message_type!("geometry_msgs", TwistStamped);

/// `geometry_msgs/TwistWithCovariance`
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TwistWithCovariance {
    pub twist: Twist,
    #[serde(with = "BigArray")]
    pub covariance: [f64; 36],
}
impl_message_type!("geometry_msgs", TwistWithCovariance);

impl Default for TwistWithCovariance {
    fn default() -> Self {
        Self {
            twist: Twist::default(),
            covariance: [0.0; 36],
        }
    }
}

/// `geometry_msgs/Transform`
#[derive(Default, Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Transform {
    pub translation: Vector3,
    pub rotation: Quaternion,
}
impl_message_type!("geometry_msgs", Transform);

/// `geometry_msgs/TransformStamped`
#[derive(Default, Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TransformStamped {
    pub header: Header,
    pub child_frame_id: std::string::String,
    pub transform: Transform,
}
impl_message_type!("geometry_msgs", TransformStamped);

/// `geometry_msgs/Accel`
#[derive(Default, Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Accel {
    pub linear: Vector3,
    pub angular: Vector3,
}
impl_message_type!("geometry_msgs", Accel);

/// `geometry_msgs/Wrench`
#[derive(Default, Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Wrench {
    pub force: Vector3,
    pub torque: Vector3,
}
impl_message_type!("geometry_msgs", Wrench);

/// `geometry_msgs/WrenchStamped`
#[derive(Default, Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct WrenchStamped {
    pub header: Header,
    pub wrench: Wrench,
}
impl_message_type!("geometry_msgs", WrenchStamped);

// ====== sensor_msgs ======================================================
// https://github.com/ros2/common_interfaces/tree/rolling/sensor_msgs/msg

/// `sensor_msgs/JointState`
#[derive(Default, Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct JointState {
    pub header: Header,
    pub name: Vec<std::string::String>,
    pub position: Vec<f64>,
    pub velocity: Vec<f64>,
    pub effort: Vec<f64>,
}
impl_message_type!("sensor_msgs", JointState);

/// `sensor_msgs/Imu`
#[derive(Default, Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Imu {
    pub header: Header,
    pub orientation: Quaternion,
    pub orientation_covariance: [f64; 9],
    pub angular_velocity: Vector3,
    pub angular_velocity_covariance: [f64; 9],
    pub linear_acceleration: Vector3,
    pub linear_acceleration_covariance: [f64; 9],
}
impl_message_type!("sensor_msgs", Imu);

/// `sensor_msgs/Temperature`
#[derive(Default, Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Temperature {
    pub header: Header,
    pub temperature: f64,
    pub variance: f64,
}
impl_message_type!("sensor_msgs", Temperature);

/// `sensor_msgs/RelativeHumidity`
#[derive(Default, Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RelativeHumidity {
    pub header: Header,
    pub relative_humidity: f64,
    pub variance: f64,
}
impl_message_type!("sensor_msgs", RelativeHumidity);

/// `sensor_msgs/FluidPressure`
#[derive(Default, Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FluidPressure {
    pub header: Header,
    pub fluid_pressure: f64,
    pub variance: f64,
}
impl_message_type!("sensor_msgs", FluidPressure);

/// `sensor_msgs/Illuminance`
#[derive(Default, Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Illuminance {
    pub header: Header,
    pub illuminance: f64,
    pub variance: f64,
}
impl_message_type!("sensor_msgs", Illuminance);

/// `sensor_msgs/BatteryState`
#[derive(Default, Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct BatteryState {
    pub header: Header,
    pub voltage: f32,
    pub temperature: f32,
    pub current: f32,
    pub charge: f32,
    pub capacity: f32,
    pub design_capacity: f32,
    pub percentage: f32,
    pub power_supply_status: u8,
    pub power_supply_health: u8,
    pub power_supply_technology: u8,
    pub present: bool,
    pub cell_voltage: Vec<f32>,
    pub cell_temperature: Vec<f32>,
    pub location: std::string::String,
    pub serial_number: std::string::String,
}
impl_message_type!("sensor_msgs", BatteryState);

/// `sensor_msgs/Range`
///
/// The `variance` field was added upstream in 2022 (present from ROS 2 Iron
/// on); peers running Humble or older use the previous wire format.
#[derive(Default, Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Range {
    pub header: Header,
    pub radiation_type: u8,
    pub field_of_view: f32,
    pub min_range: f32,
    pub max_range: f32,
    pub range: f32,
    pub variance: f32,
}
impl_message_type!("sensor_msgs", Range);

/// `sensor_msgs/MagneticField`
#[derive(Default, Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MagneticField {
    pub header: Header,
    pub magnetic_field: Vector3,
    pub magnetic_field_covariance: [f64; 9],
}
impl_message_type!("sensor_msgs", MagneticField);

/// `sensor_msgs/NavSatStatus`
#[derive(Default, Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct NavSatStatus {
    pub status: i8,
    pub service: u16,
}
impl_message_type!("sensor_msgs", NavSatStatus);

/// `sensor_msgs/NavSatFix`
#[derive(Default, Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct NavSatFix {
    pub header: Header,
    pub status: NavSatStatus,
    pub latitude: f64,
    pub longitude: f64,
    pub altitude: f64,
    pub position_covariance: [f64; 9],
    pub position_covariance_type: u8,
}
impl_message_type!("sensor_msgs", NavSatFix);

/// `sensor_msgs/LaserScan`
#[derive(Default, Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct LaserScan {
    pub header: Header,
    pub angle_min: f32,
    pub angle_max: f32,
    pub angle_increment: f32,
    pub time_increment: f32,
    pub scan_time: f32,
    pub range_min: f32,
    pub range_max: f32,
    pub ranges: Vec<f32>,
    pub intensities: Vec<f32>,
}
impl_message_type!("sensor_msgs", LaserScan);

/// `sensor_msgs/Joy`
#[derive(Default, Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Joy {
    pub header: Header,
    pub axes: Vec<f32>,
    pub buttons: Vec<i32>,
}
impl_message_type!("sensor_msgs", Joy);

// ====== nav_msgs =========================================================
// https://github.com/ros2/common_interfaces/tree/rolling/nav_msgs/msg

/// `nav_msgs/Odometry`
#[derive(Default, Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Odometry {
    pub header: Header,
    pub child_frame_id: std::string::String,
    pub pose: PoseWithCovariance,
    pub twist: TwistWithCovariance,
}
impl_message_type!("nav_msgs", Odometry);

/// `nav_msgs/Path`
#[derive(Default, Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Path {
    pub header: Header,
    pub poses: Vec<PoseStamped>,
}
impl_message_type!("nav_msgs", Path);

// ====== tf2_msgs =========================================================
// https://github.com/ros2/geometry2/tree/rolling/tf2_msgs/msg

/// `tf2_msgs/TFMessage`
#[derive(Default, Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TFMessage {
    pub transforms: Vec<TransformStamped>,
}
impl_message_type!("tf2_msgs", TFMessage);

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    #[cfg(feature = "ros2")]
    use rustdds::serialization::{
        RepresentationIdentifier, deserialize_from_cdr_with_rep_id, to_writer_with_rep_id,
    };

    /// Serialize a message to CDR (little-endian, the rustdds default) and
    /// deserialize it back; used to prove wire-format self-consistency.
    #[cfg(feature = "ros2")]
    fn cdr_roundtrip<T: MessageType + PartialEq>(msg: &T) -> Vec<u8> {
        let mut bytes = Vec::new();
        to_writer_with_rep_id(&mut bytes, msg, RepresentationIdentifier::CDR_LE)
            .expect("CDR serialization failed");
        let (decoded, consumed): (T, usize) =
            deserialize_from_cdr_with_rep_id(&bytes, RepresentationIdentifier::CDR_LE)
                .expect("CDR deserialization failed");
        assert_eq!(&decoded, msg, "CDR roundtrip altered the message");
        assert_eq!(consumed, bytes.len(), "CDR roundtrip left trailing bytes");
        bytes
    }

    fn sample_header() -> Header {
        Header {
            stamp: Time::from_nanos(1_700_000_000_123_456_789),
            frame_id: "base_link".into(),
        }
    }

    #[test]
    fn message_type_names() {
        assert_eq!(String::MESSAGE_TYPE_STR, "std_msgs/String");
        assert_eq!(Odometry::MESSAGE_TYPE_STR, "nav_msgs/Odometry");
        assert_eq!(TFMessage::MESSAGE_TYPE_STR, "tf2_msgs/TFMessage");
        assert_eq!(JointState::MESSAGE_TYPE_STR, "sensor_msgs/JointState");
    }

    #[test]
    fn quaternion_default_is_identity() {
        let q = Quaternion::default();
        assert_eq!((q.x, q.y, q.z, q.w), (0.0, 0.0, 0.0, 1.0));
    }

    /// `float64[36]` covariance must encode as a fixed array: exactly
    /// 36 × 8 bytes, with **no** 4-byte sequence length prefix.
    ///
    /// TwistWithCovariance is 6 f64 (twist) + 36 f64 (covariance):
    /// 42 × 8 = 336 bytes. A `Vec<f64>` encoding would add a length prefix
    /// and padding, producing a different size.
    #[cfg(feature = "ros2")]
    #[test]
    fn covariance_encodes_as_fixed_array() {
        let mut msg = TwistWithCovariance::default();
        for (i, c) in msg.covariance.iter_mut().enumerate() {
            *c = i as f64 * 0.5;
        }
        let bytes = cdr_roundtrip(&msg);
        assert_eq!(bytes.len(), 42 * 8, "unexpected CDR size for fixed arrays");
    }

    /// Same check for `float64[9]` (Imu-style covariance) through
    /// MagneticField, accounting for its Header explicitly.
    #[cfg(feature = "ros2")]
    #[test]
    fn nine_element_covariance_encodes_as_fixed_array() {
        // Imu without header noise: serialize the covariance-bearing part
        // through MagneticField and account for the header explicitly.
        let msg = MagneticField {
            header: Header {
                stamp: Time::from_nanos(0),
                frame_id: "".into(),
            },
            magnetic_field: Vector3 {
                x: 1.0,
                y: 2.0,
                z: 3.0,
            },
            magnetic_field_covariance: [0.1; 9],
        };
        let bytes = cdr_roundtrip(&msg);
        // Header: Time (i32 + u32 = 8) + string (4-byte length + 1 NUL,
        // padded to 8 for the following f64) = 16; then 3 f64 + 9 f64 = 96.
        assert_eq!(bytes.len(), 16 + 96);
    }

    #[cfg(feature = "ros2")]
    #[test]
    fn cdr_roundtrip_odometry() {
        let mut msg = Odometry {
            header: sample_header(),
            child_frame_id: "odom".into(),
            ..Default::default()
        };
        msg.pose.pose.position = Point {
            x: 1.5,
            y: -2.25,
            z: 0.125,
        };
        for (i, c) in msg.pose.covariance.iter_mut().enumerate() {
            *c = i as f64;
        }
        msg.twist.twist.linear.x = 0.7;
        for (i, c) in msg.twist.covariance.iter_mut().enumerate() {
            *c = -(i as f64);
        }
        cdr_roundtrip(&msg);
    }

    #[cfg(feature = "ros2")]
    #[test]
    fn cdr_roundtrip_tf_message() {
        let msg = TFMessage {
            transforms: vec![
                TransformStamped {
                    header: sample_header(),
                    child_frame_id: "child_a".into(),
                    transform: Transform {
                        translation: Vector3 {
                            x: 0.1,
                            y: 0.2,
                            z: 0.3,
                        },
                        rotation: Quaternion::default(),
                    },
                },
                TransformStamped {
                    header: sample_header(),
                    child_frame_id: "child_b".into(),
                    transform: Transform::default(),
                },
            ],
        };
        cdr_roundtrip(&msg);
    }

    #[cfg(feature = "ros2")]
    #[test]
    fn cdr_roundtrip_joint_state() {
        let msg = JointState {
            header: sample_header(),
            name: vec!["shoulder".into(), "elbow".into()],
            position: vec![0.5, -1.25],
            velocity: vec![0.0, 0.1],
            effort: vec![],
        };
        cdr_roundtrip(&msg);
    }

    #[cfg(feature = "ros2")]
    #[test]
    fn cdr_roundtrip_misc_types() {
        cdr_roundtrip(&String {
            data: "hello".into(),
        });
        cdr_roundtrip(&Bool { data: true });
        cdr_roundtrip(&Empty {});
        cdr_roundtrip(&ColorRGBA {
            r: 0.1,
            g: 0.2,
            b: 0.3,
            a: 1.0,
        });
        cdr_roundtrip(&Imu {
            header: sample_header(),
            orientation_covariance: [1.0; 9],
            ..Default::default()
        });
        cdr_roundtrip(&NavSatFix {
            header: sample_header(),
            status: NavSatStatus {
                status: -1,
                service: 3,
            },
            latitude: 48.85,
            longitude: 2.35,
            altitude: 35.0,
            position_covariance: [0.0; 9],
            position_covariance_type: 2,
        });
        cdr_roundtrip(&LaserScan {
            header: sample_header(),
            angle_min: -1.57,
            angle_max: 1.57,
            ranges: vec![1.0, 2.0, 3.0],
            intensities: vec![],
            ..Default::default()
        });
        cdr_roundtrip(&BatteryState {
            header: sample_header(),
            voltage: 12.6,
            percentage: 0.85,
            present: true,
            cell_voltage: vec![4.2, 4.2, 4.2],
            location: "slot0".into(),
            ..Default::default()
        });
    }

    /// JSON reflection roundtrip: typed -> serde_json::Value -> typed.
    #[test]
    fn json_value_roundtrip_odometry() {
        let mut msg = Odometry::default();
        msg.pose.covariance[35] = 4.5;
        let value = serde_json::to_value(&msg).expect("to_value");
        let back: Odometry = serde_json::from_value(value).expect("from_value");
        assert_eq!(back, msg);
    }
}
