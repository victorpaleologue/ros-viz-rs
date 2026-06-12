//! View a URDF file, optionally posing joints and exporting a snapshot.
//!
//! ```bash
//! cargo run --example urdf_view test-data/urdf/nao_robot.urdf
//! cargo run --example urdf_view robot.urdf --package my_pkg=/path/to/pkg \
//!     --joint HeadYaw=0.5 --export-snapshot out.png
//! ```

use std::path::PathBuf;
use std::sync::Arc;

use anyhow::{Context, Result};
use bevy::prelude::*;
use bevy::render::view::screenshot::{Screenshot, save_to_disk};
use clap::Parser;

use ros_viz_rs::robot::RobotModel;
use ros_viz_rs::robot::mesh::MeshResolver;
use ros_viz_rs::scene::{JointPositions, RobotScenePlugin, spawn_robot, spawn_viewing_rig};

#[derive(Parser, Debug)]
#[command(name = "urdf_view", about = "View and snapshot URDF robot models")]
struct Args {
    /// Path to the URDF file to visualize
    urdf_file: PathBuf,

    /// Export a snapshot to the given PNG file and exit
    #[arg(long, value_name = "PATH")]
    export_snapshot: Option<PathBuf>,

    /// Map a ROS package to a directory for package:// mesh URIs,
    /// as `name=path`. Repeatable.
    #[arg(long, value_name = "NAME=PATH")]
    package: Vec<String>,

    /// Set a joint position in radians, as `joint=value`. Repeatable.
    #[arg(long, value_name = "JOINT=RAD")]
    joint: Vec<String>,
}

#[derive(Resource)]
struct ViewSetup {
    model: Arc<RobotModel>,
    resolver: MeshResolver,
}

#[derive(Resource)]
struct SnapshotRequest {
    path: PathBuf,
    frames: u32,
    requested: bool,
}

fn main() -> Result<()> {
    let args = Args::parse();

    let model = Arc::new(RobotModel::from_urdf_file(&args.urdf_file)?);
    println!(
        "Loaded {}: {} links, {} joints",
        model.name(),
        model.urdf.links.len(),
        model.urdf.joints.len()
    );

    let mut resolver = MeshResolver::for_urdf_file(&args.urdf_file);
    for spec in &args.package {
        let (name, path) = spec
            .split_once('=')
            .with_context(|| format!("expected NAME=PATH, got '{spec}'"))?;
        resolver = resolver.with_package(name, path);
    }

    let mut joints = JointPositions::default();
    for spec in &args.joint {
        let (name, value) = spec
            .split_once('=')
            .with_context(|| format!("expected JOINT=RAD, got '{spec}'"))?;
        joints
            .positions
            .insert(name.to_string(), value.parse::<f64>()?);
    }

    let snapshot = args.export_snapshot.is_some();
    let mut app = App::new();
    app.add_plugins(DefaultPlugins.set(WindowPlugin {
        primary_window: Some(Window {
            title: format!("ros-viz-rs: {}", model.name()),
            resolution: (1024u32, 768u32).into(),
            visible: !snapshot,
            ..default()
        }),
        ..default()
    }))
    .insert_resource(ClearColor(Color::srgb(0.13, 0.14, 0.17)))
    .insert_resource(ViewSetup { model, resolver })
    .insert_resource(joints)
    .add_plugins(RobotScenePlugin)
    .add_systems(Startup, setup);

    if let Some(path) = args.export_snapshot {
        app.insert_resource(SnapshotRequest {
            path,
            frames: 0,
            requested: false,
        });
        app.add_systems(Update, capture_then_exit);
    }

    app.run();
    Ok(())
}

fn setup(
    mut commands: Commands,
    setup: Res<ViewSetup>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    spawn_viewing_rig(&mut commands);
    spawn_robot(
        &mut commands,
        &mut meshes,
        &mut materials,
        setup.model.clone(),
        &setup.resolver,
    );
}

/// Let rendering settle for a few frames, take a window screenshot, exit.
fn capture_then_exit(
    mut commands: Commands,
    mut request: ResMut<SnapshotRequest>,
    mut exit: MessageWriter<AppExit>,
) {
    request.frames += 1;
    if request.frames == 10 && !request.requested {
        commands
            .spawn(Screenshot::primary_window())
            .observe(save_to_disk(request.path.clone()));
        request.requested = true;
    }
    if request.frames >= 15 && request.requested {
        exit.write(AppExit::Success);
    }
}
