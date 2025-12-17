use crate::config::AppConfig;

pub const TOPIC_ROBOT_DESCRIPTION: &str = "/robot_description";
pub const TOPIC_JOINT_STATES: &str = "/joint_states";

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

#[derive(Debug)]
pub struct RosHandle;

/// Establish ROS2 connectivity. When the `ros` feature is disabled, this returns an error so CI can run without ROS2.
#[cfg(feature = "ros")]
pub fn connect(_config: &RosConfig) -> anyhow::Result<RosHandle> {
    Err(anyhow::anyhow!("ROS connectivity not implemented yet"))
}

/// Establish ROS2 connectivity. When the `ros` feature is disabled, this returns an error so CI can run without ROS2.
#[cfg(not(feature = "ros"))]
pub fn connect(_config: &RosConfig) -> anyhow::Result<RosHandle> {
    Err(anyhow::anyhow!(
        "Build with --features ros to enable ROS2 connectivity"
    ))
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
