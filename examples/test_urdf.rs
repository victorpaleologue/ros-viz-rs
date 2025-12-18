use anyhow::Result;
use bevy::prelude::*;
use bevy::render::camera::RenderTarget;
use bevy::render::render_resource::{
    Extent3d, TextureDescriptor, TextureDimension, TextureFormat, TextureUsages,
};
use ros_viz_rs::urdf::parse_urdf;
use std::env;
use std::fs;
use std::path::{Path, PathBuf};

fn main() -> Result<()> {
    let args: Vec<String> = env::args().collect();
    if args.len() < 2 {
        eprintln!("Usage: {} <urdf_file>", args[0]);
        eprintln!("Example: cargo run --example test_urdf test-data/urdf/simple_arm.urdf");
        std::process::exit(1);
    }

    let urdf_path = &args[1];
    let urdf_xml = fs::read_to_string(urdf_path)?;
    
    println!("Loading URDF from: {}", urdf_path);
    let scene = parse_urdf(&urdf_xml)?;
    println!("✓ Parsed URDF: {} links, {} joints", scene.links.len(), scene.joints.len());

    // Create output path
    let output_path = get_output_path(urdf_path);
    println!("Will export to: {}", output_path.display());

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: format!("URDF Test: {}", Path::new(urdf_path).file_name().unwrap().to_string_lossy()),
                resolution: (800.0, 600.0).into(),
                ..default()
            }),
            ..default()
        }))
        .insert_resource(UrdfTestConfig {
            scene,
            output_path,
            frame_count: 0,
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
}

fn setup_scene(
    mut commands: Commands,
    config: Res<UrdfTestConfig>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut images: ResMut<Assets<Image>>,
) {
    // Create render target for image export
    let size = Extent3d {
        width: 800,
        height: 600,
        ..default()
    };

    let mut image = Image {
        texture_descriptor: TextureDescriptor {
            label: None,
            size,
            dimension: TextureDimension::D2,
            format: TextureFormat::Rgba8UnormSrgb,
            mip_level_count: 1,
            sample_count: 1,
            usage: TextureUsages::COPY_SRC
                | TextureUsages::RENDER_ATTACHMENT
                | TextureUsages::TEXTURE_BINDING,
            view_formats: &[],
        },
        ..default()
    };
    image.resize(size);
    let image_handle = images.add(image);

    // Camera with render target
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(3.0, 3.0, 3.0).looking_at(Vec3::ZERO, Vec3::Y),
        Camera {
            target: RenderTarget::Image(image_handle.clone()),
            ..default()
        },
        RenderImageHandle(image_handle),
    ));

    // Lighting
    commands.spawn((
        DirectionalLight {
            illuminance: 10000.0,
            shadows_enabled: true,
            ..default()
        },
        Transform::from_xyz(4.0, 8.0, 4.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    commands.spawn((
        PointLight {
            intensity: 2000.0,
            ..default()
        },
        Transform::from_xyz(-3.0, 3.0, 3.0),
    ));

    // Spawn robot from URDF
    spawn_robot_from_urdf(&mut commands, &config.scene, &mut meshes, &mut materials);

    println!("✓ Scene setup complete");
}

#[derive(Component)]
struct RenderImageHandle(Handle<Image>);

#[derive(Component)]
struct LinkNode {
    name: String,
}

#[derive(Component)]
struct JointNode {
    name: String,
    axis: Vec3,
}

fn spawn_robot_from_urdf(
    commands: &mut Commands,
    scene: &ros_viz_rs::urdf::UrdfScene,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<StandardMaterial>>,
) {
    use std::collections::HashMap;

    let mut link_entities: HashMap<String, Entity> = HashMap::new();

    // Create link material
    let link_material = materials.add(StandardMaterial {
        base_color: Color::srgb(0.3, 0.6, 0.8),
        metallic: 0.3,
        perceptual_roughness: 0.5,
        ..default()
    });

    let joint_material = materials.add(StandardMaterial {
        base_color: Color::srgb(0.8, 0.3, 0.3),
        metallic: 0.5,
        perceptual_roughness: 0.3,
        ..default()
    });

    // Spawn all links
    for link in &scene.links {
        let mesh = meshes.add(Cuboid::new(0.1, 0.1, 0.1));
        
        let entity = commands
            .spawn((
                Mesh3d(mesh),
                MeshMaterial3d(link_material.clone()),
                Transform::default(),
                LinkNode {
                    name: link.name.clone(),
                },
            ))
            .id();

        link_entities.insert(link.name.clone(), entity);
    }

    // Spawn joints and establish hierarchy
    for joint in &scene.joints {
        if let (Some(&parent_entity), Some(&child_entity)) = (
            link_entities.get(&joint.parent),
            link_entities.get(&joint.child),
        ) {
            // Create joint visualization (small cylinder along axis)
            let joint_length = 0.15;
            let joint_radius = 0.03;
            let mesh = meshes.add(Cylinder::new(joint_radius, joint_length));

            // Calculate rotation to align cylinder with joint axis
            let default_axis = Vec3::Y;
            let joint_axis = Vec3::new(
                joint.axis[0] as f32,
                joint.axis[1] as f32,
                joint.axis[2] as f32,
            );
            let rotation = if joint_axis.dot(default_axis).abs() > 0.999 {
                Quat::IDENTITY
            } else {
                Quat::from_rotation_arc(default_axis, joint_axis)
            };

            // Apply URDF origin transform
            let origin_translation = Vec3::new(
                joint.origin_xyz[0] as f32,
                joint.origin_xyz[1] as f32,
                joint.origin_xyz[2] as f32,
            );
            let origin_rotation = Quat::from_euler(
                EulerRot::XYZ,
                joint.origin_rpy[0] as f32,
                joint.origin_rpy[1] as f32,
                joint.origin_rpy[2] as f32,
            );

            let joint_entity = commands
                .spawn((
                    Mesh3d(mesh),
                    MeshMaterial3d(joint_material.clone()),
                    Transform {
                        translation: origin_translation,
                        rotation: origin_rotation * rotation,
                        ..default()
                    },
                    JointNode {
                        name: joint.name.clone(),
                        axis: joint_axis,
                    },
                ))
                .id();

            // Set up parent-child relationships
            commands.entity(parent_entity).add_child(joint_entity);
            commands.entity(joint_entity).add_child(child_entity);

            // Apply origin to child link
            commands.entity(child_entity).insert(Transform {
                translation: Vec3::ZERO,
                rotation: Quat::IDENTITY,
                scale: Vec3::ONE,
            });
        }
    }

    println!("✓ Spawned {} links and {} joints", scene.links.len(), scene.joints.len());
}

fn capture_and_exit(
    mut config: ResMut<UrdfTestConfig>,
    images: Res<Assets<Image>>,
    query: Query<&RenderImageHandle>,
    mut app_exit: EventWriter<AppExit>,
) {
    config.frame_count += 1;

    // Wait a few frames for rendering to stabilize
    if config.frame_count < 5 {
        return;
    }

    // Capture on frame 5
    if config.frame_count == 5 {
        if let Ok(render_image) = query.get_single() {
            if let Some(image) = images.get(&render_image.0) {
                match save_image(&config.output_path, image) {
                    Ok(_) => println!("✓ Saved image to: {}", config.output_path.display()),
                    Err(e) => eprintln!("✗ Failed to save image: {}", e),
                }
            }
        }
    }

    // Exit after a few more frames
    if config.frame_count >= 10 {
        app_exit.send(AppExit::Success);
    }
}

fn save_image(path: &Path, image: &Image) -> Result<()> {
    let dynamic_image = image::DynamicImage::ImageRgba8(
        image::RgbaImage::from_raw(
            image.texture_descriptor.size.width,
            image.texture_descriptor.size.height,
            image.data.clone(),
        )
        .ok_or_else(|| anyhow::anyhow!("Failed to create image from raw data"))?,
    );

    dynamic_image.save(path)?;
    Ok(())
}

fn get_output_path(urdf_path: &str) -> PathBuf {
    let stem = Path::new(urdf_path)
        .file_stem()
        .unwrap()
        .to_string_lossy();

    // Try temp directory first
    if let Ok(temp_dir) = env::temp_dir().canonicalize() {
        return temp_dir.join(format!("{}_test.png", stem));
    }

    // Fallback to .test_outputs/
    let output_dir = PathBuf::from(".test_outputs");
    fs::create_dir_all(&output_dir).ok();
    output_dir.join(format!("{}_test.png", stem))
}
