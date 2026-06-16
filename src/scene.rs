//! Bevy scene construction and animation for a [`RobotModel`].
//!
//! [`spawn_robot`] creates one entity per URDF link under a robot root
//! entity, each link carrying its `<visual>` geometry as child entities
//! (primitives or loaded meshes). [`RobotScenePlugin`] keeps link transforms
//! in sync with [`JointPositions`] through forward kinematics — entities are
//! laid out flat under the root and receive *world* (robot-frame) transforms
//! directly from the kinematic chain, mirroring how RViz applies TF.
//!
//! ROS uses Z-up, X-forward coordinates (REP-103); Bevy is Y-up. The robot
//! root entity carries the rotation between the two, so everything inside
//! stays in ROS coordinates.

use std::collections::HashMap;
use std::f32::consts::FRAC_PI_2;
use std::sync::{Arc, Mutex};

use bevy::asset::RenderAssetUsages;
use bevy::mesh::{Indices, PrimitiveTopology};
use bevy::prelude::*;

use crate::robot::RobotModel;
use crate::robot::mesh::{LoadedMesh, MeshResolver};

/// Latest joint positions, keyed by joint name (fed from `/joint_states`,
/// UI sliders, or test scripts).
#[derive(Debug, Clone, Resource, Default)]
pub struct JointPositions {
    pub positions: HashMap<String, f64>,
}

/// A robot model received from a connection backend, waiting to be
/// spawned into the scene (consumed by the app's spawn system).
#[derive(Resource, Default)]
pub struct PendingRobot(pub Option<Arc<RobotModel>>);

/// Marks a robot root entity and owns its kinematic model.
#[derive(Component, Clone)]
pub struct RobotHandle(pub(crate) Arc<RobotModel>);

impl RobotHandle {
    /// The robot model driving this scene robot.
    pub fn model(&self) -> &RobotModel {
        &self.0
    }
}

/// Marks a link entity; transform is written by forward kinematics.
#[derive(Component, Debug)]
pub struct LinkEntity {
    pub name: String,
}

/// Marks a camera that should be repositioned to frame newly spawned robots.
#[derive(Component, Debug, Default)]
pub struct AutoFrameCamera;

/// Mesh files supplied at runtime (e.g. uploaded in the browser), merged
/// into every robot's [`MeshResolver`] when it spawns. Lets users provide
/// license-bound meshes (NAO) without hosting them.
#[derive(Resource, Default)]
pub struct MeshBlobs(pub HashMap<String, Vec<u8>>);

impl MeshBlobs {
    /// Build a resolver seeded with these blobs on top of `base`.
    pub fn apply(&self, mut base: MeshResolver) -> MeshResolver {
        base.blobs
            .extend(self.0.iter().map(|(k, v)| (k.clone(), v.clone())));
        base
    }
}

/// Queue of uploaded meshes awaiting insertion into [`MeshBlobs`]. A plain
/// static so the wasm upload entry point (`crate::web::add_mesh`) can reach
/// the running app without an `App` handle; drained by [`drain_uploaded_meshes`].
static UPLOAD_QUEUE: Mutex<Vec<(String, Vec<u8>)>> = Mutex::new(Vec::new());

/// Hand an uploaded mesh (name + bytes) to the running app. Picked up on the
/// next frame, which reloads the current robot so the mesh appears.
pub fn queue_mesh_blob(name: String, bytes: Vec<u8>) {
    UPLOAD_QUEUE
        .lock()
        .unwrap_or_else(|p| p.into_inner())
        .push((name, bytes));
}

/// Drain newly uploaded meshes into [`MeshBlobs`] and, if any arrived,
/// despawn the current robot so the spawn systems rebuild it with the new
/// meshes (both the demo and connection spawn paths re-run when no robot
/// is present).
fn drain_uploaded_meshes(
    mut commands: Commands,
    mut blobs: ResMut<MeshBlobs>,
    robots: Query<(Entity, &RobotHandle)>,
    mut pending: ResMut<PendingRobot>,
) {
    let uploaded: Vec<(String, Vec<u8>)> =
        std::mem::take(&mut *UPLOAD_QUEUE.lock().unwrap_or_else(|p| p.into_inner()));
    if uploaded.is_empty() {
        return;
    }
    for (name, bytes) in uploaded {
        blobs.0.insert(name, bytes);
    }
    // Force a respawn so the new meshes appear. Re-queue the model for the
    // connection spawn path; the demo path respawns on its own empty-guard.
    for (entity, handle) in robots.iter() {
        pending.0 = Some(handle.0.clone());
        commands.entity(entity).despawn();
    }
}

/// Spawns robots from their model and keeps them posed via FK.
pub struct RobotScenePlugin;

impl Plugin for RobotScenePlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<JointPositions>();
        app.init_resource::<PendingRobot>();
        app.init_resource::<MeshBlobs>();
        app.add_systems(
            Update,
            (
                drain_uploaded_meshes,
                sync_robot_poses,
                frame_camera_on_new_robot,
            ),
        );
    }
}

/// Rotation taking ROS coordinates (Z-up, X-forward) into Bevy's Y-up frame.
pub fn ros_to_bevy_rotation() -> Quat {
    // REP-103 -> Bevy: X stays forward-ish, ROS Z (up) becomes Bevy Y.
    Quat::from_rotation_x(-FRAC_PI_2)
}

/// Convert a nalgebra isometry (robot frame) into a Bevy [`Transform`].
pub fn isometry_to_transform(iso: &k::nalgebra::Isometry3<f32>) -> Transform {
    let t = iso.translation;
    let q = iso.rotation;
    Transform {
        translation: Vec3::new(t.x, t.y, t.z),
        rotation: Quat::from_xyzw(q.i, q.j, q.k, q.w),
        scale: Vec3::ONE,
    }
}

/// Spawn a robot under a new root entity and return that entity.
///
/// Every URDF link becomes a child of the root with a [`LinkEntity`]
/// component; its visuals are spawned beneath it. Mesh URIs that fail to
/// resolve degrade to a small marker so the kinematic structure stays
/// visible (and are logged).
pub fn spawn_robot(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    model: Arc<RobotModel>,
    resolver: &MeshResolver,
) -> Entity {
    let root = commands
        .spawn((
            RobotHandle(model.clone()),
            Transform::from_rotation(ros_to_bevy_rotation()),
            Visibility::default(),
            Name::new(format!("robot:{}", model.name())),
        ))
        .id();

    let transforms = model.link_world_transforms();
    let default_material = materials.add(robot_material(Color::srgb(0.65, 0.67, 0.7)));

    // URDF allows top-level named materials referenced from visuals.
    let named_materials: HashMap<&str, &urdf_rs::Material> = model
        .urdf
        .materials
        .iter()
        .map(|m| (m.name.as_str(), m))
        .collect();

    for link in &model.urdf.links {
        let transform = transforms
            .get(&link.name)
            .map(isometry_to_transform)
            .unwrap_or_default();

        let link_entity = commands
            .spawn((
                LinkEntity {
                    name: link.name.clone(),
                },
                transform,
                Visibility::default(),
                Name::new(format!("link:{}", link.name)),
                ChildOf(root),
            ))
            .id();

        if link.visual.is_empty() {
            // Skeleton fallback: a small marker keeps structure visible.
            commands.spawn((
                Mesh3d(meshes.add(Sphere::new(0.012))),
                MeshMaterial3d(default_material.clone()),
                Transform::default(),
                Name::new(format!("marker:{}", link.name)),
                ChildOf(link_entity),
            ));
            continue;
        }

        for (i, visual) in link.visual.iter().enumerate() {
            let material = visual_material(visual, &named_materials, materials)
                .unwrap_or_else(|| default_material.clone());
            let Some((mesh, extra_rotation, scale)) =
                visual_mesh(&visual.geometry, resolver, &link.name)
            else {
                continue;
            };

            let origin = Transform {
                translation: Vec3::new(
                    visual.origin.xyz.0[0] as f32,
                    visual.origin.xyz.0[1] as f32,
                    visual.origin.xyz.0[2] as f32,
                ),
                rotation: Quat::from_euler(
                    EulerRot::ZYX,
                    visual.origin.rpy.0[2] as f32,
                    visual.origin.rpy.0[1] as f32,
                    visual.origin.rpy.0[0] as f32,
                ) * extra_rotation,
                scale,
            };

            commands.spawn((
                Mesh3d(meshes.add(mesh)),
                MeshMaterial3d(material),
                origin,
                Name::new(format!("visual:{}:{i}", link.name)),
                ChildOf(link_entity),
            ));
        }
    }

    root
}

/// Build the Bevy mesh for a URDF geometry, with the extra local rotation
/// and scale it needs (URDF cylinders point along Z, Bevy's along Y).
fn visual_mesh(
    geometry: &urdf_rs::Geometry,
    resolver: &MeshResolver,
    link_name: &str,
) -> Option<(Mesh, Quat, Vec3)> {
    match geometry {
        urdf_rs::Geometry::Box { size } => Some((
            Cuboid::new(size.0[0] as f32, size.0[1] as f32, size.0[2] as f32).into(),
            Quat::IDENTITY,
            Vec3::ONE,
        )),
        urdf_rs::Geometry::Cylinder { radius, length } => Some((
            Cylinder::new(*radius as f32, *length as f32).into(),
            Quat::from_rotation_x(FRAC_PI_2),
            Vec3::ONE,
        )),
        urdf_rs::Geometry::Capsule { radius, length } => Some((
            Capsule3d::new(*radius as f32, *length as f32).into(),
            Quat::from_rotation_x(FRAC_PI_2),
            Vec3::ONE,
        )),
        urdf_rs::Geometry::Sphere { radius } => Some((
            Sphere::new(*radius as f32).into(),
            Quat::IDENTITY,
            Vec3::ONE,
        )),
        urdf_rs::Geometry::Mesh { filename, scale } => {
            let scale = scale
                .map(|s| Vec3::new(s.0[0] as f32, s.0[1] as f32, s.0[2] as f32))
                .unwrap_or(Vec3::ONE);
            // One entry point for filesystem (native) and uploaded bytes (web).
            match resolver.load(filename) {
                Ok(loaded) => Some((loaded_to_bevy_mesh(loaded), Quat::IDENTITY, scale)),
                Err(err) => {
                    tracing::warn!(%err, uri = filename, link = link_name, "mesh unavailable");
                    Some((Sphere::new(0.012).into(), Quat::IDENTITY, Vec3::ONE))
                }
            }
        }
    }
}

/// Convert renderer-agnostic mesh data into a Bevy mesh, computing smooth
/// normals when the file carries none.
pub fn loaded_to_bevy_mesh(loaded: LoadedMesh) -> Mesh {
    let has_normals = loaded.normals.len() == loaded.vertices.len();
    let has_colors = loaded.colors.len() == loaded.vertices.len();
    let mut mesh = Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::default(),
    );
    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, loaded.vertices);
    mesh.insert_indices(Indices::U32(loaded.indices));
    if has_normals {
        mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, loaded.normals);
    } else {
        mesh.compute_smooth_normals();
    }
    if has_colors {
        mesh.insert_attribute(Mesh::ATTRIBUTE_COLOR, loaded.colors);
    }
    mesh
}

/// Standard surface settings for robot parts.
///
/// Robot visual meshes are frequently open shells (hollow tube ends, plain
/// covers); rendering them single-sided punches see-through holes into the
/// robot, so render both faces like RViz does.
fn robot_material(color: Color) -> StandardMaterial {
    StandardMaterial {
        base_color: color,
        perceptual_roughness: 0.6,
        cull_mode: None,
        double_sided: true,
        ..default()
    }
}

/// Material for a visual: inline color, named URDF material, or none.
fn visual_material(
    visual: &urdf_rs::Visual,
    named: &HashMap<&str, &urdf_rs::Material>,
    materials: &mut Assets<StandardMaterial>,
) -> Option<Handle<StandardMaterial>> {
    let material = visual.material.as_ref()?;
    let color = material
        .color
        .as_ref()
        .or_else(|| named.get(material.name.as_str())?.color.as_ref())?;
    let [r, g, b, a] = color.rgba.0.map(|c| c as f32);
    Some(materials.add(robot_material(Color::srgba(r, g, b, a))))
}

/// Re-pose robots whenever [`JointPositions`] changes (and once when a
/// robot spawns, so a pre-populated resource takes effect immediately).
fn sync_robot_poses(
    joint_positions: Res<JointPositions>,
    robots: Query<(Entity, &RobotHandle, &Children)>,
    new_robots: Query<(), Added<RobotHandle>>,
    mut links: Query<(&LinkEntity, &mut Transform)>,
) {
    if !joint_positions.is_changed() && new_robots.is_empty() {
        return;
    }
    for (_entity, robot, children) in robots.iter() {
        // Atomic set + FK: the model could be shared between robots.
        let transforms = robot.0.pose_transforms(&joint_positions.positions);
        for child in children.iter() {
            if let Ok((link, mut transform)) = links.get_mut(child)
                && let Some(iso) = transforms.get(&link.name)
            {
                *transform = isometry_to_transform(iso);
            }
        }
    }
}

/// Marker added to robot roots once a camera has been framed on them.
#[derive(Component)]
struct Framed;

/// Bounding sphere of a robot in Bevy world space.
///
/// Prefers the union of render AABBs of the robot's mesh entities (which
/// include geometry extents); falls back to link origins when rendering is
/// not running (e.g. in headless logic tests).
fn robot_bounds(
    robot: &RobotHandle,
    root_transform: &Transform,
    root: Entity,
    children: &Query<&Children>,
    aabbs: &Query<(&GlobalTransform, &bevy::camera::primitives::Aabb)>,
) -> Option<(Vec3, f32)> {
    let mut points: Vec<Vec3> = Vec::new();
    for entity in children.iter_descendants(root) {
        if let Ok((global, aabb)) = aabbs.get(entity) {
            let center = Vec3::from(aabb.center);
            let half = Vec3::from(aabb.half_extents);
            for corner in [
                center + half,
                center - half,
                center + Vec3::new(half.x, half.y, -half.z),
                center + Vec3::new(half.x, -half.y, half.z),
                center + Vec3::new(-half.x, half.y, half.z),
                center + Vec3::new(half.x, -half.y, -half.z),
                center + Vec3::new(-half.x, half.y, -half.z),
                center + Vec3::new(-half.x, -half.y, half.z),
            ] {
                points.push(global.transform_point(corner));
            }
        }
    }
    if points.is_empty() {
        // Logic-only fallback: link origins.
        points = robot
            .0
            .link_world_transforms()
            .values()
            .map(|iso| {
                root_transform.transform_point(Vec3::new(
                    iso.translation.x,
                    iso.translation.y,
                    iso.translation.z,
                ))
            })
            .collect();
    }
    if points.is_empty() {
        return None;
    }
    let center = points.iter().sum::<Vec3>() / points.len() as f32;
    let radius = points
        .iter()
        .map(|p| p.distance(center))
        .fold(0.05_f32, f32::max);
    Some((center, radius))
}

/// Position [`AutoFrameCamera`] cameras to frame robots when they appear.
///
/// Waits a frame or two until render AABBs and global transforms are
/// propagated, then frames once per robot.
#[allow(clippy::type_complexity)]
fn frame_camera_on_new_robot(
    mut commands: Commands,
    mut waited: Local<u32>,
    robots: Query<(Entity, &RobotHandle, &Transform), Without<Framed>>,
    children: Query<&Children>,
    aabbs: Query<(&GlobalTransform, &bevy::camera::primitives::Aabb)>,
    mut cameras: Query<
        (&mut Transform, Option<&mut crate::camera::OrbitController>),
        (With<AutoFrameCamera>, Without<RobotHandle>),
    >,
) {
    let Some((entity, robot, root_transform)) = robots.iter().next() else {
        return;
    };
    // Give the renderer a few frames to compute mesh AABBs before falling
    // back to link origins (which underestimate geometry extents).
    let has_aabbs = children
        .iter_descendants(entity)
        .any(|e| aabbs.get(e).is_ok());
    if !has_aabbs && *waited < 3 {
        *waited += 1;
        return;
    }
    let Some((center, radius)) = robot_bounds(robot, root_transform, entity, &children, &aabbs)
    else {
        return;
    };
    // Pull back far enough for a ~50deg vertical FOV with some margin.
    let distance = (radius * 2.4).max(0.3);
    let eye = center + Vec3::new(distance * 0.75, distance * 0.5, distance * 0.75);
    for (mut transform, orbit) in cameras.iter_mut() {
        *transform = Transform::from_translation(eye).looking_at(center, Vec3::Y);
        // Hand the framed pose to the orbit controls so they take over from
        // here (no-op when the camera has none, e.g. headless/snapshot).
        if let Some(mut orbit) = orbit {
            orbit.aim_from_eye(center, eye);
        }
    }
    commands.entity(entity).insert(Framed);
}

/// Spawn the standard viewing rig: an auto-framing camera plus
/// [`spawn_lights`], shared by the app and examples.
pub fn spawn_viewing_rig(commands: &mut Commands) {
    let _camera = commands
        .spawn((
            Camera3d::default(),
            AutoFrameCamera,
            crate::camera::OrbitController::default(),
            Transform::from_xyz(2.0, 1.5, 2.0).looking_at(Vec3::ZERO, Vec3::Y),
        ))
        .id();
    // MSAA is unreliable on many Android GPUs (matches Bevy's mobile example).
    #[cfg(target_os = "android")]
    commands.entity(_camera).insert(Msaa::Off);
    spawn_lights(commands);
}

/// Spawn the standard lighting used across viewer, snapshots and tests:
/// one directional key light plus a strong ambient fill — deterministic,
/// no shadows.
///
/// Exactly one directional light is used on purpose: Bevy's WebGL2 backend
/// supports only a single directional light and silently drops extras (with
/// a warning), which left the web demo lit from one harsh side. A bright
/// ambient term fills the shadowed side instead, so native and web match and
/// matte parts (e.g. skeleton markers) read clearly.
pub fn spawn_lights(commands: &mut Commands) {
    commands.spawn((
        DirectionalLight {
            illuminance: 9_000.0,
            shadows_enabled: false,
            ..default()
        },
        Transform::from_xyz(4.0, 8.0, 4.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
    commands.spawn(AmbientLight {
        color: Color::WHITE,
        brightness: 1_400.0,
        affects_lightmapped_meshes: true,
    });
}

#[cfg(test)]
mod tests {
    use super::*;

    const TWO_LINK: &str = include_str!("../test-data/urdf/two_link_planar.urdf");
    const NAO: &str = include_str!("../assets/nao_robot.urdf");

    fn spawn_in_app(urdf: &str) -> (App, Entity) {
        let mut app = App::new();
        app.add_plugins(MinimalPlugins);
        app.add_plugins(AssetPlugin::default());
        app.init_asset::<Mesh>();
        app.init_asset::<StandardMaterial>();
        app.add_plugins(RobotScenePlugin);

        let model = Arc::new(RobotModel::from_urdf_str(urdf).expect("parses"));
        let root = {
            let world = app.world_mut();
            let mut state = bevy::ecs::system::SystemState::<(
                Commands,
                ResMut<Assets<Mesh>>,
                ResMut<Assets<StandardMaterial>>,
            )>::new(world);
            let (mut commands, mut meshes, mut materials) = state.get_mut(world);
            let root = spawn_robot(
                &mut commands,
                &mut meshes,
                &mut materials,
                model,
                &MeshResolver::default(),
            );
            state.apply(world);
            root
        };
        app.update();
        (app, root)
    }

    fn link_transforms(app: &mut App) -> HashMap<String, Transform> {
        let world = app.world_mut();
        let mut query = world.query::<(&LinkEntity, &Transform)>();
        query
            .iter(world)
            .map(|(l, t)| (l.name.clone(), *t))
            .collect()
    }

    #[test]
    fn spawns_every_nao_link() {
        let (mut app, _root) = spawn_in_app(NAO);
        let links = link_transforms(&mut app);
        let model = RobotModel::from_urdf_str(NAO).expect("parses");
        assert_eq!(
            links.len(),
            model.urdf.links.len(),
            "all links must spawn (the old renderer dropped subtrees)"
        );
    }

    #[test]
    fn joint_positions_repose_links() {
        let (mut app, _root) = spawn_in_app(TWO_LINK);
        let before = link_transforms(&mut app);

        let model = RobotModel::from_urdf_str(TWO_LINK).expect("parses");
        let joint = model.joint_names()[0].clone();
        app.world_mut()
            .resource_mut::<JointPositions>()
            .positions
            .insert(joint, std::f64::consts::FRAC_PI_2);
        app.update();

        let after = link_transforms(&mut app);
        let moved = before.iter().any(|(name, t)| {
            let t2 = &after[name];
            t.translation.distance(t2.translation) > 1e-4
                || t.rotation.angle_between(t2.rotation) > 1e-4
        });
        assert!(moved, "changing JointPositions must move link entities");
    }

    #[test]
    fn auto_frame_camera_points_at_robot() {
        let (mut app, _root) = {
            let mut app = App::new();
            app.add_plugins(MinimalPlugins);
            app.add_plugins(AssetPlugin::default());
            app.init_asset::<Mesh>();
            app.init_asset::<StandardMaterial>();
            app.add_plugins(RobotScenePlugin);
            app.world_mut()
                .spawn((Camera3d::default(), AutoFrameCamera, Transform::default()));
            let model = Arc::new(RobotModel::from_urdf_str(NAO).expect("parses"));
            let world = app.world_mut();
            let mut state = bevy::ecs::system::SystemState::<(
                Commands,
                ResMut<Assets<Mesh>>,
                ResMut<Assets<StandardMaterial>>,
            )>::new(world);
            let (mut commands, mut meshes, mut materials) = state.get_mut(world);
            let root = spawn_robot(
                &mut commands,
                &mut meshes,
                &mut materials,
                model,
                &MeshResolver::default(),
            );
            state.apply(world);
            // Several updates: framing waits a few frames for render AABBs
            // before falling back to link origins (none here, no renderer).
            for _ in 0..5 {
                app.update();
            }
            (app, root)
        };
        let world = app.world_mut();
        let mut query = world.query_filtered::<&Transform, With<AutoFrameCamera>>();
        let camera = query.iter(world).next().expect("camera exists");
        assert!(
            camera.translation.length() > 0.1,
            "camera should have moved to frame the robot"
        );
    }
}
