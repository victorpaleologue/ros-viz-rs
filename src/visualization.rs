use crate::urdf::UrdfScene;
use bevy::prelude::*;
use std::collections::HashMap;

#[derive(Component)]
pub struct LinkNode;

#[derive(Component)]
pub struct JointNode {
    pub name: String,
    pub axis: Vec3,
}

/// Spawn a robot from a URDF scene into the Bevy world
pub fn spawn_robot_from_urdf(
    commands: &mut Commands,
    scene: &UrdfScene,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<StandardMaterial>>,
) {
    let mut link_entities: HashMap<String, Entity> = HashMap::new();

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

    // Spawn all links (scaled up 5x for visibility)
    for link in &scene.links {
        let mesh = meshes.add(Cuboid::new(0.5, 0.5, 0.5));

        let entity = commands
            .spawn((
                Mesh3d(mesh),
                MeshMaterial3d(link_material.clone()),
                Transform::default(),
                LinkNode,
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
            let joint_axis = Vec3::new(
                joint.axis[0] as f32,
                joint.axis[1] as f32,
                joint.axis[2] as f32,
            );

            // Create joint visualization (cylinder along axis, scaled up 5x)
            let joint_length = 0.75;
            let joint_radius = 0.15;
            let mesh = meshes.add(Cylinder::new(joint_radius, joint_length));

            // Calculate rotation to align cylinder with joint axis
            let default_axis = Vec3::Y;
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
