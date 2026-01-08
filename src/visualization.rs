use crate::urdf::UrdfScene;
use bevy::prelude::*;
use bevy::render::view::screenshot::{Screenshot, save_to_disk};
use std::collections::HashMap;
use std::path::PathBuf;

#[derive(Component)]
pub struct LinkNode {
    pub name: String,
}

#[derive(Component)]
pub struct JointNode {
    pub name: String,
    pub axis: Vec3,
}

#[derive(Resource)]
pub struct RobotKinematicChain {
    pub chain: k::Chain<f32>,
    pub link_to_joint_map: HashMap<String, String>,
}

/// Standard robot visualization geometry configuration
pub struct RobotGeometry {
    pub link_size: (f32, f32, f32),
    pub joint_radius: f32,
    pub joint_height: f32,
}

impl Default for RobotGeometry {
    fn default() -> Self {
        Self {
            link_size: (1.2, 0.15, 0.15),
            joint_radius: 0.08,
            joint_height: 0.3,
        }
    }
}

/// Create standard materials for robot visualization
pub fn create_robot_materials(
    materials: &mut ResMut<Assets<StandardMaterial>>,
) -> (Handle<StandardMaterial>, Handle<StandardMaterial>) {
    let link_material = materials.add(StandardMaterial {
        base_color: Color::srgb(0.3, 0.5, 0.8),
        metallic: 0.3,
        perceptual_roughness: 0.5,
        ..default()
    });

    let joint_material = materials.add(StandardMaterial {
        base_color: Color::srgb(0.9, 0.3, 0.2),
        metallic: 0.5,
        perceptual_roughness: 0.3,
        ..default()
    });

    (link_material, joint_material)
}

/// Spawn a robot from a URDF scene into the Bevy world
pub fn spawn_robot_from_urdf(
    commands: &mut Commands,
    scene: &UrdfScene,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<StandardMaterial>>,
) {
    spawn_robot_from_urdf_with_geometry(
        commands,
        scene,
        meshes,
        materials,
        RobotGeometry::default(),
    )
}

/// Spawn a robot from a URDF scene with custom geometry
pub fn spawn_robot_from_urdf_with_geometry(
    commands: &mut Commands,
    scene: &UrdfScene,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<StandardMaterial>>,
    geometry: RobotGeometry,
) {
    use std::f32::consts::PI;

    // For now, spawn without k crate - just use the parent-child hierarchy from URDF
    // TODO: Integrate k crate properly by parsing URDF XML directly to urdf_rs::Robot

    let (link_material, joint_material) = create_robot_materials(materials);

    // Create link mesh - elongated box to represent rigid bodies
    let link_mesh = meshes.add(Cuboid::new(
        geometry.link_size.0,
        geometry.link_size.1,
        geometry.link_size.2,
    ));

    // Create joint mesh - cylinder (default aligned with Y axis)
    let joint_mesh = meshes.add(Cylinder::new(geometry.joint_radius, geometry.joint_height));

    let mut link_entities: HashMap<String, Entity> = HashMap::new();

    // Find root link (link with no parent joint)
    let child_links: std::collections::HashSet<_> =
        scene.joints.iter().map(|j| j.child.as_str()).collect();

    let root_link = scene
        .links
        .iter()
        .find(|l| !child_links.contains(l.name.as_str()))
        .map(|l| l.name.as_str())
        .unwrap_or("base_link");

    // Spawn root link at origin
    if let Some(root_info) = scene.links.iter().find(|l| l.name == root_link) {
        let entity = commands
            .spawn((
                LinkNode {
                    name: root_info.name.clone(),
                },
                Mesh3d(link_mesh.clone()),
                MeshMaterial3d(link_material.clone()),
                Transform::from_xyz(0.0, 0.0, 0.0),
                Name::new(format!("link:{}", root_info.name)),
            ))
            .id();
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
        )
        .normalize();

        let default_axis = Vec3::Y;
        let axis_rotation = if (axis - default_axis).length() < 0.01 {
            Quat::IDENTITY
        } else if (axis + default_axis).length() < 0.01 {
            Quat::from_rotation_x(PI)
        } else {
            Quat::from_rotation_arc(default_axis, axis)
        };

        // Spawn joint as child of parent link at the URDF origin position
        let joint_entity = commands
            .spawn((
                JointNode {
                    name: joint_info.name.clone(),
                    axis,
                },
                Mesh3d(joint_mesh.clone()),
                MeshMaterial3d(joint_material.clone()),
                Transform::from_translation(origin_pos).with_rotation(origin_rot * axis_rotation),
                Name::new(format!("joint:{}", joint_info.name)),
            ))
            .id();

        commands.entity(parent_entity).add_child(joint_entity);

        // Spawn child link as child of joint (at joint's local origin)
        if let Some(child_info) = scene.links.iter().find(|l| l.name == joint_info.child) {
            let child_entity = commands
                .spawn((
                    LinkNode {
                        name: child_info.name.clone(),
                    },
                    Mesh3d(link_mesh.clone()),
                    MeshMaterial3d(link_material.clone()),
                    Transform::from_xyz(0.0, 0.0, 0.0), // At joint origin
                    Name::new(format!("link:{}", child_info.name)),
                ))
                .id();

            commands.entity(joint_entity).add_child(child_entity);
            link_entities.insert(child_info.name.clone(), child_entity);
        }
    }
}

/// Setup standard lighting for robot visualization
pub fn setup_lighting(commands: &mut Commands) {
    commands.spawn((
        DirectionalLight {
            illuminance: 15000.0,
            shadows_enabled: false,
            ..default()
        },
        Transform::from_xyz(4.0, 8.0, 4.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    commands.spawn((
        PointLight {
            intensity: 5000.0,
            ..default()
        },
        Transform::from_xyz(-2.0, 3.0, 2.0),
    ));

    commands.insert_resource(AmbientLight {
        color: Color::WHITE,
        brightness: 300.0,
    });
}

/// Setup standard camera for robot visualization
pub fn setup_camera(commands: &mut Commands) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(2.5, 2.5, 2.5).looking_at(Vec3::new(0.0, 0.5, 0.0), Vec3::Y),
    ));
}

/// Configuration for rendering a URDF to an image
#[derive(Resource)]
pub struct SnapshotConfig {
    pub output_path: PathBuf,
    pub frame_count: u32,
    pub screenshot_requested: bool,
}

impl SnapshotConfig {
    pub fn new(output_path: PathBuf) -> Self {
        Self {
            output_path,
            frame_count: 0,
            screenshot_requested: false,
        }
    }
}

/// System to capture a screenshot and exit after rendering stabilizes
pub fn capture_and_exit_system(
    mut commands: Commands,
    mut config: ResMut<SnapshotConfig>,
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
            .observe(save_to_disk(output_path));
        config.screenshot_requested = true;
    }

    // Exit after screenshot
    if config.frame_count >= 15 && config.screenshot_requested {
        app_exit.send(AppExit::Success);
    }
}

/// Create a Bevy app configured to render a URDF scene and optionally export a snapshot
pub fn create_urdf_view_app(
    scene: UrdfScene,
    window_title: String,
    snapshot_output: Option<PathBuf>,
) -> App {
    let mut app = App::new();

    // For snapshot mode, create an invisible window
    // For interactive mode, create a visible window
    let is_snapshot = snapshot_output.is_some();

    app.add_plugins(DefaultPlugins.set(WindowPlugin {
        primary_window: Some(Window {
            title: window_title,
            resolution: (800.0, 600.0).into(),
            visible: !is_snapshot,     // Hide window for snapshots
            decorations: !is_snapshot, // No decorations for snapshots
            ..default()
        }),
        ..default()
    }))
    .insert_resource(ClearColor(Color::srgb(0.2, 0.2, 0.25)))
    .insert_resource(UrdfViewScene { scene })
    .add_systems(Startup, setup_urdf_view_scene);

    if let Some(output_path) = snapshot_output {
        app.insert_resource(SnapshotConfig::new(output_path))
            .add_systems(Update, capture_and_exit_system);
    }

    app
}

#[derive(Resource)]
struct UrdfViewScene {
    scene: UrdfScene,
}

fn setup_urdf_view_scene(
    mut commands: Commands,
    scene: Res<UrdfViewScene>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    setup_camera(&mut commands);
    setup_lighting(&mut commands);
    spawn_robot_from_urdf(&mut commands, &scene.scene, &mut meshes, &mut materials);
}
