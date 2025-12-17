use crate::config::AppConfig;
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

fn init_tracing() {
    // Allow overrides with RUST_LOG; default to info for early scaffolding.
    let env_filter = EnvFilter::try_from_default_env().unwrap_or_else(|_| EnvFilter::new("info"));
    let _ = tracing_subscriber::fmt()
        .with_env_filter(env_filter)
        .try_init();
}
