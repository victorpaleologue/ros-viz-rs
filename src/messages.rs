//! Registry of ROS 2 message types for reflection-based topic I/O.
//!
//! The [`MessageRegistry`] resource maps a ROS type name (e.g.
//! `"std_msgs/String"`) to a vtable of monomorphized functions that can:
//!
//! - create a type-erased subscription ([`DynSubscription`]) whose
//!   [`poll`](DynSubscription::poll) drains received messages and reflects the
//!   latest one into a lossless [`ron::Value`] tree,
//! - create a type-erased publisher ([`DynPublisher`]) whose
//!   [`publish`](DynPublisher::publish) converts a [`ron::Value`] back
//!   into the typed struct and sends it,
//! - produce a default [`ron::Value`] to seed edit buffers.
//!
//! Supporting a new message type therefore takes two steps: define the serde
//! struct in [`crate::ros_msgs`] and add one line to the
//! `standard_messages!` invocation in [`MessageRegistry::standard`].

use std::collections::HashMap;
#[cfg(feature = "ros2")]
use std::sync::{Arc, Mutex};

use bevy::prelude::Resource;
use ron::Value;
#[cfg(feature = "ros2")]
use ros2_client::Node;
#[cfg(feature = "ros2")]
use rustdds::QosPolicies;

use crate::ros_msgs::{self, MessageType};
#[cfg(feature = "ros2")]
use crate::topics_io::{setup_typed_publisher, setup_typed_subscription};

/// Reflect a typed message into a lossless [RON value](ron::Value) tree.
///
/// Unlike JSON, RON keeps the distinction between `i64`/`u64`/`f64` and can
/// represent non-finite floats (`NaN`/`inf`), which ROS messages carry
/// (covariances, laser ranges). We go through RON's text form because
/// `ron::Value` has no direct `to_value`; the round-trip is lossless.
pub(crate) fn to_value<T: serde::Serialize>(msg: &T) -> Result<Value, String> {
    let text = ron::to_string(msg).map_err(|e| e.to_string())?;
    ron::from_str::<Value>(&text).map_err(|e| e.to_string())
}

// ---------------------------------------------------------------------------
// Type-erased subscription / publisher
// ---------------------------------------------------------------------------

/// A type-erased ROS 2 subscription producing reflected values.
pub trait DynSubscription: Send + Sync {
    /// Drain all pending messages and return the latest one reflected as a
    /// [`ron::Value`], or `None` if nothing new arrived.
    fn poll(&self) -> Option<Value>;
}

/// A type-erased ROS 2 publisher consuming reflected values.
pub trait DynPublisher: Send + Sync {
    /// Convert `value` to the typed message and publish it.
    fn publish(&self, value: &Value) -> Result<(), String>;
}

/// Typed [`DynSubscription`] implementation wrapping a
/// [`ros2_client::Subscription`].
#[cfg(feature = "ros2")]
struct TypedSubscription<T: MessageType>(ros2_client::Subscription<T>);

#[cfg(feature = "ros2")]
impl<T: MessageType> DynSubscription for TypedSubscription<T> {
    fn poll(&self) -> Option<Value> {
        let mut latest = None;
        while let Ok(Some((msg, _info))) = self.0.take() {
            latest = Some(msg);
        }
        let msg = latest?;
        match to_value(&msg) {
            Ok(value) => Some(value),
            Err(e) => {
                tracing::error!("Failed to reflect {}: {e}", T::MESSAGE_TYPE_STR);
                None
            }
        }
    }
}

/// Typed [`DynPublisher`] implementation wrapping a
/// [`ros2_client::Publisher`].
#[cfg(feature = "ros2")]
struct TypedPublisher<T: MessageType>(ros2_client::Publisher<T>);

#[cfg(feature = "ros2")]
impl<T: MessageType> DynPublisher for TypedPublisher<T> {
    fn publish(&self, value: &Value) -> Result<(), String> {
        let msg: T = value.clone().into_rust().map_err(|e| {
            format!(
                "value does not match message type {}: {e}",
                T::MESSAGE_TYPE_STR
            )
        })?;
        self.0
            .publish(msg)
            .map_err(|e| format!("publish failed: {e:?}"))
    }
}

// ---------------------------------------------------------------------------
// Registry
// ---------------------------------------------------------------------------

/// Factory signature for type-erased subscriptions.
#[cfg(feature = "ros2")]
pub type SubscribeFn =
    fn(&Arc<Mutex<Node>>, &str, Option<&QosPolicies>) -> Result<Box<dyn DynSubscription>, String>;

/// Factory signature for type-erased publishers.
#[cfg(feature = "ros2")]
pub type MakePublisherFn =
    fn(&Arc<Mutex<Node>>, &str, Option<&QosPolicies>) -> Result<Box<dyn DynPublisher>, String>;

/// Per-type vtable of monomorphized entry points.
struct MessageVtable {
    #[cfg(feature = "ros2")]
    subscribe: SubscribeFn,
    #[cfg(feature = "ros2")]
    make_publisher: MakePublisherFn,
    default_value: fn() -> Value,
}

/// Monomorphized [`MessageVtable::subscribe`] entry.
#[cfg(feature = "ros2")]
fn subscribe_erased<T: MessageType>(
    node: &Arc<Mutex<Node>>,
    topic: &str,
    qos: Option<&QosPolicies>,
) -> Result<Box<dyn DynSubscription>, String> {
    let subscription = setup_typed_subscription::<T>(node, topic, qos)?;
    Ok(Box::new(TypedSubscription(subscription)))
}

/// Monomorphized [`MessageVtable::make_publisher`] entry.
#[cfg(feature = "ros2")]
fn make_publisher_erased<T: MessageType>(
    node: &Arc<Mutex<Node>>,
    topic: &str,
    qos: Option<&QosPolicies>,
) -> Result<Box<dyn DynPublisher>, String> {
    let publisher = setup_typed_publisher::<T>(node, topic, qos)?;
    Ok(Box::new(TypedPublisher(publisher)))
}

/// Monomorphized [`MessageVtable::default_value`] entry.
fn default_value_erased<T: MessageType + Default>() -> Value {
    to_value(&T::default()).unwrap_or(Value::Unit)
}

/// Register every standard message type on `$registry`.
///
/// Adding support for a new message = define the struct in
/// [`crate::ros_msgs`] + add one line here.
macro_rules! standard_messages {
    ($registry:ident) => {
        standard_messages!(@register $registry,
            // std_msgs
            ros_msgs::Bool,
            ros_msgs::Byte,
            ros_msgs::Char,
            ros_msgs::ColorRGBA,
            ros_msgs::Empty,
            ros_msgs::Float32,
            ros_msgs::Float64,
            ros_msgs::Float64MultiArray,
            ros_msgs::Header,
            ros_msgs::Int8,
            ros_msgs::Int16,
            ros_msgs::Int32,
            ros_msgs::Int64,
            ros_msgs::String,
            ros_msgs::UInt8,
            ros_msgs::UInt16,
            ros_msgs::UInt32,
            ros_msgs::UInt64,
            // geometry_msgs
            ros_msgs::Accel,
            ros_msgs::Point,
            ros_msgs::Point32,
            ros_msgs::Pose,
            ros_msgs::Pose2D,
            ros_msgs::PoseArray,
            ros_msgs::PoseStamped,
            ros_msgs::PoseWithCovariance,
            ros_msgs::Quaternion,
            ros_msgs::Transform,
            ros_msgs::TransformStamped,
            ros_msgs::Twist,
            ros_msgs::TwistStamped,
            ros_msgs::TwistWithCovariance,
            ros_msgs::Vector3,
            ros_msgs::Wrench,
            ros_msgs::WrenchStamped,
            // sensor_msgs
            ros_msgs::BatteryState,
            ros_msgs::FluidPressure,
            ros_msgs::Illuminance,
            ros_msgs::Imu,
            ros_msgs::JointState,
            ros_msgs::Joy,
            ros_msgs::LaserScan,
            ros_msgs::MagneticField,
            ros_msgs::NavSatFix,
            ros_msgs::Range,
            ros_msgs::RelativeHumidity,
            ros_msgs::Temperature,
            // nav_msgs
            ros_msgs::Odometry,
            ros_msgs::Path,
            // tf2_msgs
            ros_msgs::TFMessage,
        );
    };
    (@register $registry:ident, $($t:ty),* $(,)?) => {
        $( $registry.register::<$t>(); )*
    };
}

/// Bevy resource mapping ROS type names to message vtables.
///
/// The [`Default`] implementation returns [`MessageRegistry::standard`],
/// i.e. all standard message types known to this crate.
#[derive(Resource)]
pub struct MessageRegistry {
    entries: HashMap<&'static str, MessageVtable>,
}

impl Default for MessageRegistry {
    fn default() -> Self {
        Self::standard()
    }
}

impl MessageRegistry {
    /// Create an empty registry.
    pub fn empty() -> Self {
        Self {
            entries: HashMap::new(),
        }
    }

    /// Register a message type under its [`MessageType::MESSAGE_TYPE_STR`].
    pub fn register<T: MessageType + Default>(&mut self) {
        self.entries.insert(
            T::MESSAGE_TYPE_STR,
            MessageVtable {
                #[cfg(feature = "ros2")]
                subscribe: subscribe_erased::<T>,
                #[cfg(feature = "ros2")]
                make_publisher: make_publisher_erased::<T>,
                default_value: default_value_erased::<T>,
            },
        );
    }

    /// Whether `type_name` (e.g. `"std_msgs/String"`) is registered.
    pub fn contains(&self, type_name: &str) -> bool {
        self.entries.contains_key(type_name)
    }

    /// Iterate over the registered ROS type names.
    pub fn type_names(&self) -> impl Iterator<Item = &'static str> + '_ {
        self.entries.keys().copied()
    }

    /// Create a type-erased subscription on `topic` for `type_name`.
    #[cfg(feature = "ros2")]
    pub fn subscribe(
        &self,
        type_name: &str,
        node: &Arc<Mutex<Node>>,
        topic: &str,
        qos: Option<&QosPolicies>,
    ) -> Result<Box<dyn DynSubscription>, String> {
        let vtable = self
            .entries
            .get(type_name)
            .ok_or_else(|| format!("message type '{type_name}' is not registered"))?;
        (vtable.subscribe)(node, topic, qos)
    }

    /// Create a type-erased publisher on `topic` for `type_name`.
    #[cfg(feature = "ros2")]
    pub fn make_publisher(
        &self,
        type_name: &str,
        node: &Arc<Mutex<Node>>,
        topic: &str,
        qos: Option<&QosPolicies>,
    ) -> Result<Box<dyn DynPublisher>, String> {
        let vtable = self
            .entries
            .get(type_name)
            .ok_or_else(|| format!("message type '{type_name}' is not registered"))?;
        (vtable.make_publisher)(node, topic, qos)
    }

    /// Default value of `type_name` reflected as a [`ron::Value`];
    /// `None` if the type is not registered.
    pub fn default_value(&self, type_name: &str) -> Option<Value> {
        self.entries
            .get(type_name)
            .map(|vtable| (vtable.default_value)())
    }

    /// Registry pre-populated with all standard message types of this crate.
    pub fn standard() -> Self {
        let mut registry = Self::empty();
        standard_messages!(registry);
        registry
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;
    // Only the DDS round-trip helpers poll with a timeout.
    #[cfg(feature = "ros2")]
    use std::time::{Duration, Instant};

    /// Build a lossless [`ron::Value`] from a `serde_json!` literal, for test
    /// ergonomics — the seam carries `ron::Value`, but literals read best
    /// written with `json!`.
    fn rv(j: serde_json::Value) -> Value {
        ron::from_str(&ron::to_string(&j).unwrap()).unwrap()
    }

    #[cfg(feature = "ros2")]
    /// Pick a random DDS domain ID in 1..=232 to isolate tests from each
    /// other and from any running ROS 2 system on domain 0.
    fn random_domain_id() -> u16 {
        use std::collections::hash_map::RandomState;
        use std::hash::{BuildHasher, Hasher};
        let hash = RandomState::new().build_hasher().finish();
        (hash % 232 + 1) as u16
    }

    #[cfg(feature = "ros2")]
    /// Create a throwaway ROS node on a random domain.
    fn test_node(name: &str) -> Arc<Mutex<Node>> {
        let domain_id = random_domain_id();
        let ctx = ros2_client::Context::with_options(
            ros2_client::ContextOptions::new().domain_id(domain_id),
        )
        .expect("create context");
        let node_name = ros2_client::NodeName::new("/", name).expect("valid node name");
        let node = ctx
            .new_node(node_name, ros2_client::NodeOptions::new())
            .expect("create node");
        Arc::new(Mutex::new(node))
    }

    #[test]
    fn standard_registry_contains_expected_types() {
        let registry = MessageRegistry::standard();
        for type_name in [
            "std_msgs/Bool",
            "std_msgs/String",
            "std_msgs/Empty",
            "std_msgs/Char",
            "std_msgs/Int8",
            "std_msgs/Int16",
            "std_msgs/Int32",
            "std_msgs/Int64",
            "std_msgs/UInt8",
            "std_msgs/UInt16",
            "std_msgs/UInt32",
            "std_msgs/UInt64",
            "std_msgs/Float32",
            "std_msgs/Float64",
            "std_msgs/ColorRGBA",
            "std_msgs/Header",
            "geometry_msgs/Vector3",
            "geometry_msgs/Point",
            "geometry_msgs/Point32",
            "geometry_msgs/Quaternion",
            "geometry_msgs/Pose",
            "geometry_msgs/Pose2D",
            "geometry_msgs/PoseStamped",
            "geometry_msgs/PoseArray",
            "geometry_msgs/Twist",
            "geometry_msgs/TwistStamped",
            "geometry_msgs/Transform",
            "geometry_msgs/TransformStamped",
            "geometry_msgs/Accel",
            "geometry_msgs/Wrench",
            "geometry_msgs/WrenchStamped",
            "sensor_msgs/JointState",
            "sensor_msgs/Imu",
            "sensor_msgs/Temperature",
            "sensor_msgs/RelativeHumidity",
            "sensor_msgs/FluidPressure",
            "sensor_msgs/Illuminance",
            "sensor_msgs/BatteryState",
            "sensor_msgs/Range",
            "sensor_msgs/MagneticField",
            "sensor_msgs/NavSatFix",
            "sensor_msgs/LaserScan",
            "sensor_msgs/Joy",
            "nav_msgs/Odometry",
            "nav_msgs/Path",
            "tf2_msgs/TFMessage",
        ] {
            assert!(registry.contains(type_name), "missing {type_name}");
        }
        assert!(!registry.contains("made_up_msgs/Nope"));
    }

    #[test]
    fn default_value_round_trips_to_typed() {
        let registry = MessageRegistry::standard();

        let joint_state = registry.default_value("sensor_msgs/JointState").unwrap();
        let typed: ros_msgs::JointState = joint_state.into_rust().expect("JointState");
        assert_eq!(typed, ros_msgs::JointState::default());

        let odometry = registry.default_value("nav_msgs/Odometry").unwrap();
        let typed: ros_msgs::Odometry = odometry.into_rust().expect("Odometry");
        // covariance is a float64[36]; into_rust only succeeds when the
        // reflected tree carries all 36 elements.
        assert_eq!(typed.pose.covariance.len(), 36);
        assert_eq!(typed, ros_msgs::Odometry::default());

        let tf = registry.default_value("tf2_msgs/TFMessage").unwrap();
        let typed: ros_msgs::TFMessage = tf.into_rust().expect("TFMessage");
        assert_eq!(typed, ros_msgs::TFMessage::default());

        assert_eq!(registry.default_value("made_up_msgs/Nope"), None);
    }

    #[test]
    fn default_value_quaternion_is_identity() {
        let registry = MessageRegistry::standard();
        let q = registry.default_value("geometry_msgs/Quaternion").unwrap();
        assert_eq!(q, rv(json!({"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0})));
    }

    #[test]
    fn typed_to_value_to_typed_round_trip() {
        let msg = ros_msgs::JointState {
            header: ros_msgs::Header::default(),
            name: vec!["a".into(), "b".into()],
            position: vec![1.0, 2.0],
            velocity: vec![],
            effort: vec![],
        };
        // Exercise the actual reflection seam (`to_value` -> `into_rust`),
        // not a bare serde_json round trip.
        let value = to_value(&msg).unwrap();
        let names: Vec<String> = match &value {
            Value::Map(m) => m
                .get(&Value::String("name".into()))
                .and_then(|v| v.clone().into_rust().ok())
                .unwrap_or_default(),
            _ => Vec::new(),
        };
        assert_eq!(names, vec!["a".to_string(), "b".to_string()]);
        let back: ros_msgs::JointState = value.into_rust().unwrap();
        assert_eq!(back, msg);
    }

    /// The reason the value tree is RON rather than JSON: non-finite floats
    /// and full-width integers survive reflection losslessly. The same values
    /// through `serde_json` would be clobbered (`NaN`/`±inf` → `null`,
    /// `u64`/`i64` past 2^53 → lossy `f64`).
    #[test]
    fn reflection_preserves_non_finite_floats_and_wide_ints() {
        // Non-finite f64 round-trips through the reflected value.
        for probe in [f64::INFINITY, f64::NEG_INFINITY, f64::NAN] {
            let value = to_value(&ros_msgs::Float64 { data: probe }).unwrap();
            assert_ne!(value, Value::Unit, "non-finite float must reflect");
            let back: ros_msgs::Float64 = value.into_rust().expect("Float64 round-trip");
            if probe.is_nan() {
                assert!(back.data.is_nan(), "NaN must survive reflection");
            } else {
                assert_eq!(back.data, probe);
            }
        }

        // A u64 beyond f64's 2^53 exact-integer range survives unrounded —
        // serde_json would coerce it to a lossy f64.
        let wide = (1u64 << 53) + 1;
        let value = to_value(&ros_msgs::UInt64 { data: wide }).unwrap();
        let back: ros_msgs::UInt64 = value.into_rust().expect("UInt64 round-trip");
        assert_eq!(back.data, wide);
    }

    #[cfg(feature = "ros2")]
    #[test]
    fn unregistered_type_errors() {
        let registry = MessageRegistry::standard();
        let node = test_node("unregistered_type_node");
        let err = registry
            .subscribe("made_up_msgs/Nope", &node, "/nope", None)
            .err()
            .expect("subscribe must fail");
        assert!(err.contains("not registered"), "got: {err}");
        let err = registry
            .make_publisher("made_up_msgs/Nope", &node, "/nope", None)
            .err()
            .expect("make_publisher must fail");
        assert!(err.contains("not registered"), "got: {err}");
    }

    #[cfg(feature = "ros2")]
    #[test]
    fn publisher_rejects_mismatched_value() {
        let registry = MessageRegistry::standard();
        let node = test_node("mismatch_node");
        let publisher = registry
            .make_publisher("geometry_msgs/Twist", &node, "/mismatch_twist", None)
            .expect("make_publisher");
        let err = publisher
            .publish(&rv(json!({"data": "not a twist"})))
            .expect_err("must fail");
        assert!(err.contains("geometry_msgs/Twist"), "got: {err}");
    }

    /// Publish a reflected value and poll it back through the registry on a
    /// private DDS domain. Returns the received value.
    #[cfg(feature = "ros2")]
    fn pub_sub_round_trip(type_name: &str, topic: &str, value: Value) -> Value {
        let registry = MessageRegistry::standard();
        let node = test_node("registry_roundtrip_node");

        let subscription = registry
            .subscribe(type_name, &node, topic, None)
            .expect("subscribe");
        let publisher = registry
            .make_publisher(type_name, &node, topic, None)
            .expect("make_publisher");

        // Discovery between endpoints of the same node still takes a moment;
        // keep publishing until the value comes back (or time out).
        let deadline = Instant::now() + Duration::from_secs(10);
        loop {
            publisher.publish(&value).expect("publish");
            std::thread::sleep(Duration::from_millis(50));
            if let Some(received) = subscription.poll() {
                return received;
            }
            assert!(
                Instant::now() < deadline,
                "no {type_name} message received on {topic} within 10s"
            );
        }
    }

    #[cfg(feature = "ros2")]
    #[test]
    fn dds_round_trip_string() {
        crate::require_dds_multicast!();
        let sent = rv(json!({"data": "hello reflection"}));
        let received = pub_sub_round_trip("std_msgs/String", "/registry_test_string", sent.clone());
        assert_eq!(received, sent);
    }

    #[cfg(feature = "ros2")]
    #[test]
    fn dds_round_trip_twist() {
        crate::require_dds_multicast!();
        let sent = rv(json!({
            "linear": {"x": 1.5, "y": 0.0, "z": -0.5},
            "angular": {"x": 0.0, "y": 0.0, "z": 3.25},
        }));
        let received =
            pub_sub_round_trip("geometry_msgs/Twist", "/registry_test_twist", sent.clone());
        assert_eq!(received, sent);
    }

    #[cfg(feature = "ros2")]
    #[test]
    fn dds_round_trip_joint_state() {
        crate::require_dds_multicast!();
        let sent = rv(json!({
            "header": {"stamp": {"sec": 12, "nanosec": 34}, "frame_id": "base"},
            "name": ["shoulder", "elbow"],
            "position": [0.5, -1.25],
            "velocity": [],
            "effort": [],
        }));
        let received = pub_sub_round_trip(
            "sensor_msgs/JointState",
            "/registry_test_joint_state",
            sent.clone(),
        );
        assert_eq!(received, sent);
    }
}
