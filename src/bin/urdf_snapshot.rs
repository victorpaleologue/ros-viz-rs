//! Helper binary for taking URDF snapshots in tests
//! This is separate from the example to ensure it runs on the main thread

use anyhow::Result;
use ros_viz_rs::urdf::parse_urdf;
use ros_viz_rs::visualization::create_urdf_view_app;
use std::env;
use std::fs;

fn main() -> Result<()> {
    let args: Vec<String> = env::args().collect();

    if args.len() != 3 {
        eprintln!("Usage: {} <urdf-file> <output-png>", args[0]);
        std::process::exit(1);
    }

    let urdf_path = &args[1];
    let output_path = &args[2];

    // Load and parse URDF
    let urdf_xml = fs::read_to_string(urdf_path)?;
    let scene = parse_urdf(&urdf_xml)?;

    // Create app with snapshot export
    let window_title = format!("Snapshot: {}", urdf_path);
    let mut app = create_urdf_view_app(scene, window_title, Some(output_path.into()));

    // Run the app (it will exit after capturing the screenshot)
    app.run();

    Ok(())
}
