use clap::Parser;
use std::path::PathBuf;

/// Command-line interface. Uses clap with env fallback; see https://docs.rs/clap/latest/clap/_tutorial/index.html
#[derive(Debug, Clone, Parser)]
#[command(
    name = "ros-viz-rs",
    version,
    about = "Bevy-based ROS2 robot visualizer"
)]
pub struct CliArgs {
    /// ROS domain to join. Overrides ROS_DOMAIN_ID when provided.
    #[arg(long, env = "ROS_DOMAIN_ID", value_name = "ID")]
    pub domain: Option<u32>,

    /// Run without creating a native window; used for automated rendering/tests.
    #[arg(long, default_value_t = false)]
    pub headless: bool,

    /// Optional output image path to save a rendered frame.
    #[arg(long, value_name = "PATH")]
    pub output_image: Option<PathBuf>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AppConfig {
    pub domain_id: u32,
    pub headless: bool,
    pub output_image: Option<PathBuf>,
}

impl AppConfig {
    pub const DEFAULT_DOMAIN_ID: u32 = 0;

    pub fn new(domain_id: u32) -> Self {
        Self {
            domain_id,
            headless: false,
            output_image: None,
        }
    }
}

impl CliArgs {
    pub fn into_config(self) -> AppConfig {
        let domain = self.domain.unwrap_or(AppConfig::DEFAULT_DOMAIN_ID);
        AppConfig {
            domain_id: domain,
            headless: self.headless,
            output_image: self.output_image,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::{Mutex, OnceLock};

    static ENV_LOCK: OnceLock<Mutex<()>> = OnceLock::new();

    fn env_lock() -> &'static Mutex<()> {
        ENV_LOCK.get_or_init(|| Mutex::new(()))
    }

    fn with_domain_env<T, F: FnOnce() -> T>(value: Option<&str>, f: F) -> T {
        let _guard = env_lock().lock().expect("lock poisoned");
        // Safety: mutations are serialized by the mutex to avoid concurrent env races in tests.
        match value {
            Some(v) => unsafe { std::env::set_var("ROS_DOMAIN_ID", v) },
            None => unsafe { std::env::remove_var("ROS_DOMAIN_ID") },
        }
        let result = f();
        unsafe { std::env::remove_var("ROS_DOMAIN_ID") };
        result
    }

    #[test]
    fn defaults_to_zero_when_not_set() {
        let config = with_domain_env(None, || CliArgs::parse_from(["ros-viz-rs"]).into_config());
        assert_eq!(config.domain_id, AppConfig::DEFAULT_DOMAIN_ID);
        assert!(!config.headless);
        assert!(config.output_image.is_none());
    }

    #[test]
    fn reads_from_env_when_no_flag() {
        let config = with_domain_env(Some("42"), || {
            CliArgs::parse_from(["ros-viz-rs"]).into_config()
        });
        assert_eq!(config.domain_id, 42);
        assert!(!config.headless);
        assert!(config.output_image.is_none());
    }

    #[test]
    fn cli_flag_overrides_env() {
        let config = with_domain_env(Some("42"), || {
            CliArgs::parse_from(["ros-viz-rs", "--domain", "7"]).into_config()
        });
        assert_eq!(config.domain_id, 7);
    }

    #[test]
    fn parses_headless_flag() {
        let cfg = CliArgs::parse_from(["ros-viz-rs", "--headless"]).into_config();
        assert!(cfg.headless);
    }

    #[test]
    fn parses_output_image_path() {
        let cfg = CliArgs::parse_from(["ros-viz-rs", "--output-image", "out.png"]).into_config();
        assert_eq!(
            cfg.output_image.as_deref(),
            Some(std::path::Path::new("out.png"))
        );
    }
}
