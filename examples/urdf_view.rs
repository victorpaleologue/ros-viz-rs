use anyhow::Result;
use clap::Parser;
use ros_viz_rs::urdf::parse_urdf;
use ros_viz_rs::visualization::create_urdf_view_app;
use std::fs;
use std::path::PathBuf;

#[derive(Parser, Debug)]
#[command(name = "urdf_view")]
#[command(about = "View and optionally snapshot URDF robot models", long_about = None)]
struct Args {
    /// Path to the URDF file to visualize
    urdf_file: PathBuf,

    /// Export a snapshot to the specified PNG file and exit
    #[arg(long = "export-snapshot", value_name = "PATH")]
    export_snapshot: Option<PathBuf>,
}

fn main() -> Result<()> {
    let args = Args::parse();

    let urdf_xml = fs::read_to_string(&args.urdf_file)?;

    println!("Loading URDF from: {}", args.urdf_file.display());
    let scene = parse_urdf(&urdf_xml)?;
    println!(
        "✓ Parsed URDF: {} links, {} joints",
        scene.links.len(),
        scene.joints.len()
    );

    if let Some(ref output_path) = args.export_snapshot {
        println!("Will export snapshot to: {}", output_path.display());
    } else {
        println!("Running in interactive mode (no snapshot)");
    }

    let window_title = format!(
        "URDF View: {}",
        args.urdf_file.file_name().unwrap().to_string_lossy()
    );

    let mut app = create_urdf_view_app(scene, window_title, args.export_snapshot);
    app.run();

    Ok(())
}
