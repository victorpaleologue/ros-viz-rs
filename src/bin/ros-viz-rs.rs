//! `ros-viz-rs` alias binary — identical to the `ros-viz` binary (`src/main.rs`).
//!
//! Kept as a separate source file (rather than a second `[[bin]]` pointing at
//! `src/main.rs`) so the same file isn't claimed by two build targets, which
//! Cargo warns about. See the `[[bin]]` notes in `Cargo.toml`.

use clap::Parser;

use ros_viz_rs::{app, options::Options};

fn main() -> anyhow::Result<()> {
    let options = Options::parse();
    app::run(options)
}
