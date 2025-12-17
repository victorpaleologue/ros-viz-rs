use clap::Parser;
use ros_viz_rs::{CliArgs, app};

fn main() -> anyhow::Result<()> {
    let cli = CliArgs::parse();
    let config = cli.into_config();
    app::run(config)
}
