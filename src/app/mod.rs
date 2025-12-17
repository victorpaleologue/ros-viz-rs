use crate::config::AppConfig;
use bevy::MinimalPlugins;
use bevy::app::App;
use tracing_subscriber::EnvFilter;

/// Initialize tracing and launch the (future) Bevy-based app.
/// Tracing setup mirrors https://docs.rs/tracing-subscriber/latest/tracing_subscriber/fmt/index.html
pub fn run(config: AppConfig) -> anyhow::Result<()> {
    init_tracing();
    tracing::info!(
        domain_id = config.domain_id,
        headless = config.headless,
        output_image = ?config.output_image,
        "Starting ros-viz-rs application"
    );

    // TODO: Bevy app bootstrap and ROS2 graph wiring will be added here.
    Ok(())
}

/// Build a Bevy `App` configured for headless or windowed rendering.
/// Rendering plugins are intentionally minimal for now to keep CI stable; full stacks will be added later.
pub fn build_app(config: &AppConfig) -> App {
    let mut app = App::new();
    app.insert_resource(config.clone());

    if config.headless {
        app.add_plugins(MinimalPlugins);
    } else {
        // Placeholder: use MinimalPlugins until windowed rendering is wired; DefaultPlugins will come later.
        app.add_plugins(MinimalPlugins);
    }

    app
}

fn init_tracing() {
    // Allow overrides with RUST_LOG; default to info for early scaffolding.
    let env_filter = EnvFilter::try_from_default_env().unwrap_or_else(|_| EnvFilter::new("info"));
    let _ = tracing_subscriber::fmt()
        .with_env_filter(env_filter)
        .try_init();
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn builds_headless_app() {
        let mut cfg = AppConfig::new(0);
        cfg.headless = true;
        let app = build_app(&cfg);
        let stored = app.world().get_resource::<AppConfig>().cloned();
        assert_eq!(stored, Some(cfg));
    }

    #[test]
    fn builds_windowed_app_placeholder() {
        let cfg = AppConfig::new(0);
        let app = build_app(&cfg);
        let stored = app.world().get_resource::<AppConfig>().cloned();
        assert_eq!(stored, Some(cfg));
    }
}
