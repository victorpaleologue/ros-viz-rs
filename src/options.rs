use std::path::PathBuf;

use bevy::prelude::*;

use clap::Parser;

const DEFAULT_ROS_DOMAIN_ID: u16 = 0;
const DEFAULT_WIDTH: u32 = 800;
const DEFAULT_HEIGHT: u32 = 600;

/// Command-line interface. Uses clap with env fallback; see https://docs.rs/clap/latest/clap/_tutorial/index.html
#[derive(Debug, Default, Clone, Parser, Resource, PartialEq, Eq)]
#[command(
    name = "ros-viz-rs",
    version,
    about = "Bevy-based ROS2 robot visualizer"
)]
pub struct Options {
    /// ROS domain to join. Overrides ROS_DOMAIN_ID when provided.
    #[arg(long, env = "ROS_DOMAIN_ID", value_name = "ID", default_value_t = DEFAULT_ROS_DOMAIN_ID)]
    pub domain: u16,

    /// URDF visualization. If set, ROS is not used.
    #[arg(long, value_name = "PATH")]
    pub urdf: Option<PathBuf>,

    /// Run without creating a native window and save the rendered frame to the given path.
    #[arg(long, value_name = "PATH")]
    pub snapshot_to: Option<PathBuf>,

    /// Render width in pixels (windowed or headless capture).
    #[arg(long, default_value_t = DEFAULT_WIDTH, value_name = "PX")]
    pub width: u32,

    /// Render height in pixels (windowed or headless capture).
    #[arg(long, default_value_t = DEFAULT_HEIGHT, value_name = "PX")]
    pub height: u32,
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
        let options = with_domain_env(None, || Options::parse_from(["ros-viz-rs"]));
        assert_eq!(options.domain, 0);
        assert!(options.urdf.is_none());
        assert!(options.snapshot_to.is_none());
        assert_eq!(options.width, DEFAULT_WIDTH);
        assert_eq!(options.height, DEFAULT_HEIGHT);
    }

    #[test]
    fn reads_from_env_when_no_flag() {
        let options = with_domain_env(Some("42"), || Options::parse_from(["ros-viz-rs"]));
        assert_eq!(options.domain, 42);
        assert!(options.urdf.is_none());
        assert!(options.snapshot_to.is_none());
        assert_eq!(options.width, DEFAULT_WIDTH);
        assert_eq!(options.height, DEFAULT_HEIGHT);
    }

    #[test]
    fn cli_flag_overrides_env() {
        let options = with_domain_env(Some("42"), || {
            Options::parse_from(["ros-viz-rs", "--domain", "7"])
        });
        assert_eq!(options.domain, 7);
    }

    #[test]
    fn parses_output_image_path() {
        let cfg = Options::parse_from(["ros-viz-rs", "--snapshot-to", "out.png"]);
        assert_eq!(
            cfg.snapshot_to.as_deref(),
            Some(std::path::Path::new("out.png"))
        );
    }

    #[test]
    fn parses_resolution() {
        let cfg = Options::parse_from(["ros-viz-rs", "--width", "1024", "--height", "576"]);
        assert_eq!(cfg.width, 1024);
        assert_eq!(cfg.height, 576);
    }
}
