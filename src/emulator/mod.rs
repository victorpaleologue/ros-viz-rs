use crate::config::AppConfig;

#[derive(Debug, Clone)]
pub struct EmulatorConfig {
    pub domain_id: u32,
    pub robot_name: String,
}

impl EmulatorConfig {
    pub fn from_app(app: &AppConfig, robot_name: impl Into<String>) -> Self {
        Self {
            domain_id: app.domain_id,
            robot_name: robot_name.into(),
        }
    }
}

#[derive(Debug)]
pub struct Emulator;

/// Placeholder for a ROS2-backed emulator that will publish `/robot_description` and `/joint_states`.
pub fn start(_config: &EmulatorConfig) -> anyhow::Result<Emulator> {
    Err(anyhow::anyhow!("Emulator not implemented yet"))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn emulator_stub_returns_error() {
        let cfg = EmulatorConfig::from_app(&AppConfig::new(0), "dummy");
        let err = start(&cfg).unwrap_err();
        assert!(err.to_string().contains("Emulator"));
    }
}
