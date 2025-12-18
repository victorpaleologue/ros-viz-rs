use anyhow::Result;
use bevy::prelude::*;
use bevy::render::view::screenshot::{Screenshot, save_to_disk};
use clap::Parser;
use ros_viz_rs::urdf::parse_urdf;
use ros_viz_rs::visualization::{setup_camera, setup_lighting};
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

    let has_export = args.export_snapshot.is_some();
    
    if let Some(ref output_path) = args.export_snapshot {
        println!("Will export snapshot to: {}", output_path.display());
    } else {
        println!("Running in interactive mode (no snapshot)");
    }

    let window_title = format!(
        "URDF View: {}",
        args.urdf_file.file_name().unwrap().to_string_lossy()
    );

    let mut app = App::new();
    app.add_plugins(DefaultPlugins.set(WindowPlugin {
        primary_window: Some(Window {
            title: window_title,
            resolution: (800.0, 600.0).into(),
            ..default()
        }),
        ..default()
    }))
    .insert_resource(ClearColor(Color::srgb(0.2, 0.2, 0.25)))
    .insert_resource(UrdfViewConfig {
        scene,
        export_snapshot: args.export_snapshot,
        frame_count: 0,
        screenshot_requested: false,
    })
    .add_systems(Startup, setup_scene);

    // Only add exit system if we're exporting a snapshot
    if has_export {
        app.add_systems(Update, capture_and_exit);
    }

    app.run();

    Ok(())
}

#[derive(Resource)]
struct UrdfViewConfig {
    scene: ros_viz_rs::urdf::UrdfScene,
    export_snapshot: Option<PathBuf>,
    frame_count: u32,
    screenshot_requested: bool,
}

fn setup_scene(
    mut commands: Commands,
    config: Res<UrdfViewConfig>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    setup_camera(&mut commands);
    setup_lighting(&mut commands);
    ros_viz_rs::visualization::spawn_robot_from_urdf(
        &mut commands,
        &config.scene,
        &mut meshes,
        &mut materials,
    );

    println!("✓ Scene setup complete");
}

fn capture_and_exit(
    mut commands: Commands,
    mut config: ResMut<UrdfViewConfig>,
    mut app_exit: EventWriter<AppExit>,
) {
    config.frame_count += 1;

    // Wait for rendering to stabilize
    if config.frame_count < 10 {
        return;
    }

    // Take screenshot on frame 10
    if config.frame_count == 10 && !config.screenshot_requested {
        if let Some(output_path) = config.export_snapshot.clone() {
            commands
                .spawn(Screenshot::primary_window())
                .observe(save_to_disk(output_path.clone()));
            config.screenshot_requested = true;
            println!("✓ Requested screenshot to: {}", output_path.display());
        }
    }

    // Exit after screenshot
    if config.frame_count >= 15 && config.screenshot_requested {
        app_exit.send(AppExit::Success);
    }
}
