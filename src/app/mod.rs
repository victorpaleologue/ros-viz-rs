use crate::config::AppConfig;
#[cfg(feature = "ros")]
use crate::ros::{self, RosConfig};
use crate::urdf::{parse_urdf, UrdfScene};
use bevy::app::App;
use bevy::prelude::*;
use bevy::MinimalPlugins;
use std::time::Duration;
#[cfg(feature = "render")]
use bevy::window::{PresentMode, WindowResolution};
#[cfg(feature = "render")]
use bevy::DefaultPlugins;

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

#[derive(Debug, Component)]
struct LinkNode;

#[derive(Debug, Component)]
struct JointNode {
    name: String,
    axis: Vec3,  // Rotation axis from URDF
}

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

    let mut app = build_app(&config);

    // Always run the full loop to receive ROS messages
    app.run();
    Ok(())
}

/// Build a Bevy `App` configured for headless or windowed rendering.
/// Rendering plugins are intentionally minimal for now to keep CI stable; full stacks will be added later.
pub fn build_app(config: &AppConfig) -> App {
    let mut app = App::new();
    app.insert_resource(config.clone());

    // Start with empty robot assets - will be populated from ROS /robot_description
    app.insert_resource(RobotAssets { urdf_xml: None, scene: None });
    app.insert_resource(JointPositions::default());
    app.insert_resource(UrdfWaitTimer::default());

    #[cfg(feature = "ros")]
    {
        if let Some(ros) = maybe_init_ros(config) {
            app.insert_resource(RosState { handle: ros });
        }
    }

    if config.headless {
        app.add_plugins(MinimalPlugins);
    } else {
        #[cfg(feature = "render")]
        {
            let window_plugin = bevy::window::WindowPlugin {
                primary_window: Some(bevy::window::Window {
                    resolution: WindowResolution::new(
                        config.render.width as f32,
                        config.render.height as f32,
                    ),
                    present_mode: PresentMode::AutoNoVsync,
                    ..Default::default()
                }),
                ..Default::default()
            };

            app.add_plugins(
                DefaultPlugins
                    .set(window_plugin)
                    .set(bevy::log::LogPlugin {
                        level: bevy::log::Level::WARN,
                        filter: "wgpu_core=warn,wgpu_hal=warn".into(),
                        custom_layer: |_| None,
                    }),
            );
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
    if !config.headless {
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
fn maybe_init_ros(config: &AppConfig) -> Option<ros::RosHandle> {
    let cfg = RosConfig::from_app(config);
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
    use std::collections::HashMap;
    use std::f32::consts::PI;

    // Create link meshes - boxes to represent rigid bodies
    let link_mesh = meshes.add(Cuboid::new(1.2, 0.15, 0.15));
    let link_material = materials.add(StandardMaterial {
        base_color: Color::srgb(0.2, 0.6, 0.8),
        metallic: 0.3,
        perceptual_roughness: 0.5,
        ..Default::default()
    });

    // Create joint mesh - cylinder (default aligned with Y axis)
    let joint_mesh = meshes.add(Cylinder::new(0.08, 0.3));  // radius, height
    let joint_material = materials.add(StandardMaterial {
        base_color: Color::srgb(0.9, 0.3, 0.2),
        metallic: 0.5,
        perceptual_roughness: 0.3,
        ..Default::default()
    });

    // Build kinematic tree with parent-child relationships
    let mut link_entities: HashMap<String, Entity> = HashMap::new();

    // Find root link (link with no parent joint)
    let child_links: std::collections::HashSet<_> = scene.joints
        .iter()
        .map(|j| j.child.as_str())
        .collect();

    let root_link = scene.links
        .iter()
        .find(|l| !child_links.contains(l.name.as_str()))
        .map(|l| l.name.as_str())
        .unwrap_or("base_link");

    // Spawn root link at origin
    if let Some(root_info) = scene.links.iter().find(|l| l.name == root_link) {
        let entity = commands.spawn((
            LinkNode,
            Mesh3d(link_mesh.clone()),
            MeshMaterial3d(link_material.clone()),
            Transform::from_xyz(0.0, 0.0, 0.0),
            Name::new(format!("link:{}", root_info.name)),
        )).id();
        link_entities.insert(root_info.name.clone(), entity);
    }

    // Spawn joints and their child links
    for joint_info in &scene.joints {
        // Get parent link entity (should exist)
        let parent_entity = match link_entities.get(&joint_info.parent) {
            Some(e) => *e,
            None => continue, // Skip if parent not found
        };

        // Calculate joint transform from URDF origin
        let origin_pos = Vec3::new(
            joint_info.origin_xyz[0] as f32,
            joint_info.origin_xyz[1] as f32,
            joint_info.origin_xyz[2] as f32,
        );

        let origin_rot = Quat::from_euler(
            EulerRot::XYZ,
            joint_info.origin_rpy[0] as f32,
            joint_info.origin_rpy[1] as f32,
            joint_info.origin_rpy[2] as f32,
        );

        // Calculate rotation to align cylinder (default Y-axis) with joint axis
        let axis = Vec3::new(
            joint_info.axis[0] as f32,
            joint_info.axis[1] as f32,
            joint_info.axis[2] as f32,
        ).normalize();

        let default_axis = Vec3::Y;
        let axis_rotation = if (axis - default_axis).length() < 0.01 {
            Quat::IDENTITY
        } else if (axis + default_axis).length() < 0.01 {
            Quat::from_rotation_x(PI)
        } else {
            Quat::from_rotation_arc(default_axis, axis)
        };

        // Spawn joint as child of parent link
        let joint_entity = commands.spawn((
            JointNode {
                name: joint_info.name.clone(),
                axis,  // Store axis for animation
            },
            Mesh3d(joint_mesh.clone()),
            MeshMaterial3d(joint_material.clone()),
            Transform::from_translation(origin_pos + Vec3::Y * 0.6)  // Offset up along link
                .with_rotation(origin_rot * axis_rotation),
            Name::new(format!("joint:{}", joint_info.name)),
        )).id();

        commands.entity(parent_entity).add_child(joint_entity);

        // Spawn child link as child of joint
        if let Some(child_info) = scene.links.iter().find(|l| l.name == joint_info.child) {
            let child_entity = commands.spawn((
                LinkNode,
                Mesh3d(link_mesh.clone()),
                MeshMaterial3d(link_material.clone()),
                Transform::from_xyz(0.0, 0.4, 0.0),  // Offset from joint
                Name::new(format!("link:{}", child_info.name)),
            )).id();

            commands.entity(joint_entity).add_child(child_entity);
            link_entities.insert(child_info.name.clone(), child_entity);
        }
    }
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
        Transform::from_rotation(Quat::from_euler(
            EulerRot::ZYX,
            0.0,
            -0.7,
            -0.6,
        )),
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
                        tracing::info!("Parsed URDF: {} links, {} joints", scene.links.len(), scene.joints.len());
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
fn receive_joint_states(
    mut joint_positions: ResMut<JointPositions>,
    ros: Option<Res<RosState>>,
) {
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
        let mut cfg = AppConfig::new(0);
        cfg.headless = true;
        let app = build_app(&cfg);
        let stored = app.world().get_resource::<AppConfig>().cloned();
        assert_eq!(stored, Some(cfg));
        assert!(app.world().contains_resource::<RobotAssets>());
        assert!(app.world().contains_resource::<JointPositions>());
    }

    #[test]
    fn builds_windowed_app_placeholder() {
        let cfg = AppConfig::new(0);
        let app = build_app(&cfg);
        let stored = app.world().get_resource::<AppConfig>().cloned();
        assert_eq!(stored, Some(cfg));
        assert!(app.world().contains_resource::<RobotAssets>());
        assert!(app.world().contains_resource::<JointPositions>());
    }

    #[test]
    fn startup_spawns_scene_entities() {
        let cfg = AppConfig::new(0);
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

        assert_eq!(link_count, assets.scene.link_count());
        assert_eq!(joint_count, assets.scene.joint_count());
    }

    #[test]
    fn stub_image_matches_resolution_and_marks_joint() {
        let scene = UrdfScene {
            joints: vec!["j1".to_string()],
            links: vec!["l1".to_string()],
        };
        let assets = RobotAssets {
            urdf_xml: "<robot></robot>".to_string(),
            scene,
        };
        let emulator = Emulator::start(
            EmulatorConfig::from_app(&AppConfig::new(0), "dummy", "<robot></robot>"),
            BTreeMap::new(),
        );

        emulator.apply_joint_commands(vec![("j1".to_string(), 1.0)]);
        let img = render_stub_image(&assets, &emulator, &RenderConfig::new(64, 48));
        assert_eq!(img.width(), 64);
        assert_eq!(img.height(), 48);
        let top_left = img.get_pixel(0, 0);
        let sample_bar = img.get_pixel(1, 10);
        assert_ne!(top_left, sample_bar, "joint bar should differ from marker");
    }

    #[cfg(feature = "urdf")]
    #[test]
    fn joint_commands_update_transforms() {
        let cfg = AppConfig::new(0);
        let mut app = build_app(&cfg);
        app.update(); // run startup systems

        {
            let emulator = app
                .world()
                .get_resource::<Emulator>()
                .cloned()
                .expect("emulator present");
            emulator.apply_joint_commands(vec![("shoulder".to_string(), 1.25)]);
        }

        app.update(); // run sync system

        let world = app.world_mut();
        let mut query = world.query::<(&JointNode, &Transform)>();
        let mut found = false;
        for (node, transform) in query.iter(&world) {
            if node.name == "shoulder" {
                found = true;
                assert!((transform.translation.x - 1.25).abs() < 1e-6);
            }
        }
        assert!(found);
    }

    #[test]
    fn run_writes_output_image_when_requested() {
        let dir = tempdir().expect("tempdir");
        let img_path = dir.path().join("out.png");

        let mut cfg = AppConfig::new(0);
        cfg.output_image = Some(img_path.clone());
        run(cfg).expect("run succeeds");

        let meta = fs::metadata(&img_path).expect("image exists");
        assert!(meta.len() > 0, "image should not be empty");
    }
}
