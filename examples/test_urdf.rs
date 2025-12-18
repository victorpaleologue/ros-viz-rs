use anyhow::Result;
use bevy::prelude::*;
use bevy::render::view::screenshot::{Screenshot, save_to_disk};
use ros_viz_rs::urdf::parse_urdf;
use ros_viz_rs::visualization::{setup_camera, setup_lighting};
use std::env;
use std::fs;
use std::path::{Path, PathBuf};

fn main() -> Result<()> {
    let args: Vec<String> = env::args().collect();
    if args.len() < 2 {
        eprintln!("Usage: {} <urdf_file> [output_image]", args[0]);
        eprintln!(
            "Example: cargo run --example test_urdf test-data/urdf/simple_arm.urdf simple_arm.png"
        );
        eprintln!("If output_image is omitted, saves to <urdf_name>.png in current directory");
        std::process::exit(1);
    }

    let urdf_path = &args[1];
    let urdf_xml = fs::read_to_string(urdf_path)?;

    println!("Loading URDF from: {}", urdf_path);
    let scene = parse_urdf(&urdf_xml)?;
    println!(
        "✓ Parsed URDF: {} links, {} joints",
        scene.links.len(),
        scene.joints.len()
    );

    // Determine output path
    let output_path = if args.len() >= 3 {
        PathBuf::from(&args[2])
    } else {
        // Default to current directory with URDF name
        let stem = Path::new(urdf_path).file_stem().unwrap().to_string_lossy();
        PathBuf::from(format!("{}.png", stem))
    };
    println!("Will export to: {}", output_path.display());

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: format!(
                    "URDF Test: {}",
                    Path::new(urdf_path).file_name().unwrap().to_string_lossy()
                ),
                resolution: (800.0, 600.0).into(),
                ..default()
            }),
            ..default()
        }))
        .insert_resource(ClearColor(Color::srgb(0.2, 0.2, 0.25)))
        .insert_resource(UrdfTestConfig {
            scene,
            output_path,
            frame_count: 0,
            screenshot_requested: false,
        })
        .add_systems(Startup, setup_scene)
        .add_systems(Update, capture_and_exit)
        .run();

    Ok(())
}

#[derive(Resource)]
struct UrdfTestConfig {
    scene: ros_viz_rs::urdf::UrdfScene,
    output_path: PathBuf,
    frame_count: u32,
    screenshot_requested: bool,
}

fn setup_scene(
    mut commands: Commands,
    config: Res<UrdfTestConfig>,
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
    mut config: ResMut<UrdfTestConfig>,
    mut app_exit: EventWriter<AppExit>,
) {
    config.frame_count += 1;

    // Wait for rendering to stabilize
    if config.frame_count < 10 {
        return;
    }

    // Take screenshot on frame 10
    if config.frame_count == 10 && !config.screenshot_requested {
        let output_path = config.output_path.clone();
        commands
            .spawn(Screenshot::primary_window())
            .observe(save_to_disk(output_path.clone()));
        config.screenshot_requested = true;
        println!("✓ Requested screenshot to: {}", output_path.display());
    }

    // Exit after screenshot
    if config.frame_count >= 15 && config.screenshot_requested {
        app_exit.send(AppExit::Success);
    }
}
