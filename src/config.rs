use bevy::prelude::Resource;
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

    /// Render width in pixels (windowed or headless capture).
    #[arg(long, default_value_t = RenderConfig::DEFAULT_WIDTH, value_name = "PX")]
    pub width: u32,

    /// Render height in pixels (windowed or headless capture).
    #[arg(long, default_value_t = RenderConfig::DEFAULT_HEIGHT, value_name = "PX")]
    pub height: u32,
}

#[derive(Debug, Clone, PartialEq, Eq, Resource)]
pub struct AppConfig {
    pub domain_id: u32,
    pub headless: bool,
    pub output_image: Option<PathBuf>,
    pub render: RenderConfig,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RenderConfig {
    pub width: u32,
    pub height: u32,
}

impl AppConfig {
    pub const DEFAULT_DOMAIN_ID: u32 = 0;

    pub fn new(domain_id: u32) -> Self {
        Self {
            domain_id,
            headless: false,
            output_image: None,
            render: RenderConfig::default(),
        }
    }
}

impl RenderConfig {
    pub const DEFAULT_WIDTH: u32 = 800;
    pub const DEFAULT_HEIGHT: u32 = 600;

    pub fn new(width: u32, height: u32) -> Self {
        Self { width, height }
    }
}

impl Default for RenderConfig {
    fn default() -> Self {
        Self {
            width: Self::DEFAULT_WIDTH,
            height: Self::DEFAULT_HEIGHT,
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
            render: RenderConfig::new(self.width, self.height),
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
        assert_eq!(config.render.width, RenderConfig::DEFAULT_WIDTH);
        assert_eq!(config.render.height, RenderConfig::DEFAULT_HEIGHT);
    }

    #[test]
    fn reads_from_env_when_no_flag() {
        let config = with_domain_env(Some("42"), || {
            CliArgs::parse_from(["ros-viz-rs"]).into_config()
        });
        assert_eq!(config.domain_id, 42);
        assert!(!config.headless);
        assert!(config.output_image.is_none());
        assert_eq!(config.render.width, RenderConfig::DEFAULT_WIDTH);
        assert_eq!(config.render.height, RenderConfig::DEFAULT_HEIGHT);
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

    #[test]
    fn parses_resolution() {
        let cfg =
            CliArgs::parse_from(["ros-viz-rs", "--width", "1024", "--height", "576"]).into_config();
        assert_eq!(cfg.render.width, 1024);
        assert_eq!(cfg.render.height, 576);
    }
}
