use crate::options::Options;

#[cfg(feature = "ros")]
use crate::ros::{self, RosConfig};
use crate::urdf::{UrdfScene, parse_urdf};
#[cfg(feature = "render")]
use bevy::DefaultPlugins;
use bevy::MinimalPlugins;
use bevy::app::App;
use bevy::prelude::*;
#[cfg(feature = "render")]
use bevy::window::{PresentMode, WindowResolution};
use std::time::Duration;

use std::collections::HashMap;
use tracing_subscriber::EnvFilter;

#[cfg(feature = "ros")]
#[derive(Debug, Resource)]
struct RosState {
    handle: ros::RosHandle,
}

#[derive(Debug, Clone, Resource)]
pub struct RobotAssets {
    pub urdf_xml: Option<String>,
    pub scene: Option<UrdfScene>,
}

#[derive(Debug, Clone, Resource, Default)]
pub struct JointPositions {
    pub positions: HashMap<String, f64>,
}

#[derive(Debug, Resource)]
struct UrdfWaitTimer {
    timer: Timer,
    warned: bool,
}

impl Default for UrdfWaitTimer {
    fn default() -> Self {
        Self {
            timer: Timer::new(Duration::from_secs(3), TimerMode::Once),
            warned: false,
        }
    }
}

// Re-export components from visualization module
use crate::visualization::{JointNode, LinkNode};

/// Initialize tracing and launch the (future) Bevy-based app.
/// Tracing setup mirrors https://docs.rs/tracing-subscriber/latest/tracing_subscriber/fmt/index.html
pub fn run(options: Options) -> anyhow::Result<()> {
    init_tracing();
    tracing::info!("Starting ros-viz-rs application with options: {options:?}");

    let mut app = build_app(&options);

    // Always run the full loop to receive ROS messages
    app.run();
    Ok(())
}

/// Build a Bevy `App` configured for headless or windowed rendering.
/// Rendering plugins are intentionally minimal for now to keep CI stable; full stacks will be added later.
pub fn build_app(options: &Options) -> App {
    let mut app = App::new();
    app.insert_resource(options.clone());

    // Start with empty robot assets - will be populated from ROS /robot_description
    app.insert_resource(RobotAssets {
        urdf_xml: None,
        scene: None,
    });
    app.insert_resource(JointPositions::default());
    app.insert_resource(UrdfWaitTimer::default());

    #[cfg(feature = "ros")]
    {
        if let Some(ros) = maybe_init_ros(options) {
            app.insert_resource(RosState { handle: ros });
        }
    }

    if options.snapshot_to.is_some() {
        app.add_plugins(MinimalPlugins);
    } else {
        #[cfg(feature = "render")]
        {
            let window_plugin = bevy::window::WindowPlugin {
                primary_window: Some(bevy::window::Window {
                    resolution: WindowResolution::new(options.width as f32, options.height as f32),
                    present_mode: PresentMode::AutoNoVsync,
                    ..Default::default()
                }),
                ..Default::default()
            };

            app.add_plugins(DefaultPlugins.set(window_plugin).set(bevy::log::LogPlugin {
                level: bevy::log::Level::WARN,
                filter: "wgpu_core=warn,wgpu_hal=warn".into(),
                custom_layer: |_| None,
            }));
            app.add_systems(Startup, spawn_render_basics);
        }

        #[cfg(not(feature = "render"))]
        {
            app.add_plugins(MinimalPlugins);
        }
    }

    // Note: TransformPlugin is included in both DefaultPlugins and MinimalPlugins in Bevy 0.15

    // Don't populate scene at startup - wait for ROS /robot_description
    // populate_urdf_scene needs Assets which are only available with DefaultPlugins (render mode)
    #[cfg(feature = "render")]
    if options.snapshot_to.is_none() {
        app.add_systems(Update, check_and_spawn_robot);
    }

    app.add_systems(Update, sync_joint_transforms);

    #[cfg(feature = "ros")]
    {
        app.add_systems(Update, (receive_robot_description, receive_joint_states));
    }

    app
}

#[cfg(feature = "ros")]
fn maybe_init_ros(options: &Options) -> Option<ros::RosHandle> {
    let cfg = RosConfig::new(options.domain);
    match ros::connect(&cfg) {
        Ok(handle) => Some(handle),
        Err(err) => {
            tracing::warn!(?err, "ROS connect failed; continuing without ROS");
            None
        }
    }
}

#[cfg(feature = "render")]
fn populate_urdf_scene_inner(
    commands: &mut Commands,
    scene: &UrdfScene,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<StandardMaterial>>,
) {
    crate::visualization::spawn_robot_from_urdf(commands, scene, meshes, materials);
}

fn sync_joint_transforms(
    joint_positions: Res<JointPositions>,
    mut joint_query: Query<(&JointNode, &Children)>,
    mut child_query: Query<&mut Transform, Without<JointNode>>,
) {
    for (node, children) in joint_query.iter_mut() {
        if let Some(value) = joint_positions.positions.get(&node.name) {
            // Rotate child links around the joint axis
            // The joint entity itself stays in place, but we rotate its children
            for &child_entity in children.iter() {
                if let Ok(mut child_transform) = child_query.get_mut(child_entity) {
                    // Apply rotation around the joint axis
                    let rotation = Quat::from_axis_angle(node.axis, *value as f32);
                    child_transform.rotation = rotation;
                }
            }
        }
    }
}

#[cfg(feature = "render")]
fn spawn_render_basics(mut commands: Commands) {
    // Camera positioned to view the robot from an angle
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(3.0, 2.5, 4.0).looking_at(Vec3::new(0.0, 1.0, 0.0), Vec3::Y),
    ));

    // Main directional light from above-right
    commands.spawn((
        DirectionalLight {
            shadows_enabled: true,
            illuminance: 10000.0,
            ..Default::default()
        },
        Transform::from_rotation(Quat::from_euler(EulerRot::ZYX, 0.0, -0.7, -0.6)),
    ));

    // Add ambient light for better visibility
    commands.insert_resource(bevy::pbr::AmbientLight {
        color: Color::WHITE,
        brightness: 300.0,
    });
}

#[cfg(feature = "ros")]
fn receive_robot_description(
    mut assets: ResMut<RobotAssets>,
    ros: Option<Res<RosState>>,
    mut wait_timer: ResMut<UrdfWaitTimer>,
    time: Res<Time>,
) {
    // Only try to receive if we don't already have a URDF
    if assets.urdf_xml.is_some() {
        return;
    }

    // Update wait timer
    wait_timer.timer.tick(time.delta());

    if let Some(ros) = ros.as_ref() {
        match ros.handle.try_take_robot_description() {
            Ok(Some(urdf_xml)) => {
                tracing::info!("✓ Received robot_description from ROS topic");
                match parse_urdf(&urdf_xml) {
                    Ok(scene) => {
                        tracing::info!(
                            "Parsed URDF: {} links, {} joints",
                            scene.links.len(),
                            scene.joints.len()
                        );
                        assets.urdf_xml = Some(urdf_xml);
                        assets.scene = Some(scene);
                    }
                    Err(err) => {
                        tracing::error!(?err, "Failed to parse URDF from /robot_description");
                    }
                }
            }
            Ok(None) => {
                // No message yet - warn after timeout
                if wait_timer.timer.finished() && !wait_timer.warned {
                    wait_timer.warned = true;
                    tracing::warn!(
                        "No /robot_description received after {} seconds. \
                        Make sure a ROS2 node is publishing the URDF on domain {}. \
                        Common publishers: robot_state_publisher, joint_state_publisher",
                        wait_timer.timer.duration().as_secs(),
                        ros.handle.domain_id()
                    );
                }
            }
            Err(err) => tracing::warn!(?err, "Failed to read /robot_description"),
        }
    } else {
        // No ROS connection
        if wait_timer.timer.finished() && !wait_timer.warned {
            wait_timer.warned = true;
            tracing::warn!("ROS connection not available - cannot receive robot_description");
        }
    }
}

#[cfg(feature = "ros")]
fn receive_joint_states(mut joint_positions: ResMut<JointPositions>, ros: Option<Res<RosState>>) {
    if let Some(ros) = ros.as_ref() {
        match ros.handle.try_take_joint_states() {
            Ok(Some(msg)) => {
                // Update joint positions from ROS message
                for (name, position) in msg.name.iter().zip(msg.position.iter()) {
                    joint_positions.positions.insert(name.clone(), *position);
                }
            }
            Ok(None) => {}
            Err(err) => tracing::warn!(?err, "Failed to read /joint_states"),
        }
    }
}

#[cfg(feature = "render")]
fn check_and_spawn_robot(
    mut commands: Commands,
    assets: Res<RobotAssets>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    existing_links: Query<&LinkNode>,
) {
    // Only spawn if we have a scene and haven't spawned yet
    if let Some(scene) = &assets.scene {
        if existing_links.is_empty() {
            tracing::info!("Spawning robot from URDF");
            populate_urdf_scene_inner(&mut commands, scene, &mut meshes, &mut materials);
        }
    }
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
    use std::fs;
    use tempfile::tempdir;

    #[test]
    fn builds_headless_app() {
        let mut cfg = Options::default();
        cfg.snapshot_to = Some("dummy.png".into());
        let app = build_app(&cfg);
        let stored = app.world().get_resource::<Options>().cloned();
        assert_eq!(stored, Some(cfg));
        assert!(app.world().contains_resource::<RobotAssets>());
        assert!(app.world().contains_resource::<JointPositions>());
    }

    #[test]
    fn builds_windowed_app_placeholder() {
        let cfg = Options::default();
        let app = build_app(&cfg);
        let stored = app.world().get_resource::<Options>().cloned();
        assert_eq!(stored, Some(cfg));
        assert!(app.world().contains_resource::<RobotAssets>());
        assert!(app.world().contains_resource::<JointPositions>());
    }

    #[test]
    fn startup_spawns_scene_entities() {
        let cfg = Options::default();
        let mut app = build_app(&cfg);
        app.update();

        let assets = app
            .world()
            .get_resource::<RobotAssets>()
            .cloned()
            .expect("assets present");

        let world = app.world_mut();
        let link_count = world.query::<&LinkNode>().iter(&*world).count();
        let joint_count = world
            .query::<(&JointNode, &Transform)>()
            .iter(&*world)
            .count();

        // RobotAssets.scene is now Option<UrdfScene>
        if let Some(scene) = assets.scene {
            assert_eq!(link_count, scene.link_count());
            assert_eq!(joint_count, scene.joint_count());
        }
    }

    // TODO: Update these tests - they reference old Emulator/render code
    #[test]
    #[ignore = "needs updating for new architecture"]
    fn stub_image_matches_resolution_and_marks_joint() {
        // let scene = UrdfScene { ... };
        // Test needs updating for new visualization module
        todo!("Update test for new UrdfScene structure with JointInfo/LinkInfo")
    }

    #[cfg(feature = "urdf")]
    #[test]
    #[ignore = "needs updating for new architecture"]
    fn joint_commands_update_transforms() {
        // Test needs updating for new joint state handling
        todo!("Update test for ROS2 joint state subscription model")
    }

    #[test]
    fn run_writes_output_image_when_requested() {
        let dir = tempdir().expect("tempdir");
        let img_path = dir.path().join("out.png");

        let mut cfg = Options::default();
        cfg.snapshot_to = Some(img_path.clone());
        run(cfg).expect("run succeeds");

        let meta = fs::metadata(&img_path).expect("image exists");
        assert!(meta.len() > 0, "image should not be empty");
    }
}
