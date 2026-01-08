use clap::Parser;

use ros_viz_rs::{app, options::Options};

fn main() -> anyhow::Result<()> {
    let options = Options::parse();
    app::run(options)
}
