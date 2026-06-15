//! Self-contained demo: an embedded NAO waving hello, no transport at all.
//!
//! [`DemoPlugin`] spawns the bundled NAO description and drives
//! [`JointPositions`] from a [`JointScript`] every frame. It works in any
//! app that has [`RobotScenePlugin`](crate::scene::RobotScenePlugin):
//! the native viewer (`--demo`), headless snapshots, and the web build —
//! which is exactly why it knows nothing about ROS.

use std::sync::{Arc, Mutex};

use bevy::prelude::*;

use crate::robot::RobotModel;
use crate::robot::mesh::MeshResolver;
use crate::scene::{JointPositions, MeshBlobs, RobotHandle, spawn_robot};

/// The NAO H25 description bundled with the crate (geometry referenced via
/// `package://nao_meshes` resolves only if that package is provided; links
/// fall back to skeleton markers otherwise).
pub const NAO_URDF: &str = include_str!("../assets/nao_robot.urdf");

/// An optional URDF for the demo to show instead of the bundled NAO, set by
/// the page (`crate::web::set_demo_robot`) so the browser can switch robots.
/// A plain static for the same reason as the upload queue: the wasm runtime
/// has no `App` handle. Empty in native tests, so the default stays NAO.
static DEMO_URDF: Mutex<Option<String>> = Mutex::new(None);

/// Choose the robot the demo displays (its URDF). Takes effect on the next
/// spawn; reload the page (or clear the robot) to switch.
pub fn set_demo_urdf(urdf_xml: String) {
    *DEMO_URDF.lock().unwrap_or_else(|p| p.into_inner()) = Some(urdf_xml);
}

/// Joint positions as a function of elapsed time (seconds).
pub type JointScript = Box<dyn FnMut(f64) -> Vec<(String, f64)> + Send>;

/// Ready-made joint scripts for demos and tests.
pub mod scripts {
    use super::JointScript;

    /// Hold the given positions forever.
    pub fn static_pose(pose: Vec<(String, f64)>) -> JointScript {
        Box::new(move |_t| pose.clone())
    }

    /// Sweep every joint with a sine wave at slightly different frequencies.
    pub fn sine_sweep(joints: Vec<String>, amplitude: f64) -> JointScript {
        Box::new(move |t| {
            joints
                .iter()
                .enumerate()
                .map(|(i, name)| {
                    let freq = 0.5 + i as f64 * 0.3;
                    (name.clone(), amplitude * (t * freq).sin())
                })
                .collect()
        })
    }

    /// A NAO waving hello with its right arm.
    ///
    /// Joint names follow the NAO H25 description (see
    /// <http://doc.aldebaran.com/2-8/family/nao_technical/joints_naov6.html>).
    pub fn nao_wave() -> JointScript {
        Box::new(|t| {
            let swing = (t * 4.0).sin();
            vec![
                ("RShoulderPitch".into(), -1.2),
                ("RShoulderRoll".into(), -0.25),
                ("RElbowYaw".into(), 1.2),
                ("RElbowRoll".into(), 0.9 + 0.35 * swing),
                ("RWristYaw".into(), 0.0),
                ("HeadYaw".into(), -0.25 + 0.1 * swing),
            ]
        })
    }
}

/// The script currently animating [`JointPositions`].
///
/// Wrapped in a [`Mutex`] because scripts are `FnMut` and Bevy resources
/// must be `Sync`.
#[derive(Resource)]
pub struct DemoScript(pub Mutex<JointScript>);

/// Spawns the embedded NAO and animates it with [`scripts::nao_wave`]
/// (or whatever [`DemoScript`] is inserted before this plugin).
pub struct DemoPlugin;

impl Plugin for DemoPlugin {
    fn build(&self, app: &mut App) {
        if !app.world().contains_resource::<DemoScript>() {
            app.insert_resource(DemoScript(Mutex::new(scripts::nao_wave())));
        }
        // On Update (not just Startup) and guarded on "no robot present", so
        // it respawns after a mesh upload despawns the robot (see
        // `drain_uploaded_meshes`).
        app.add_systems(Update, (spawn_demo_robot, animate_demo));
    }
}

fn spawn_demo_robot(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    blobs: Res<MeshBlobs>,
    existing: Query<(), With<RobotHandle>>,
) {
    if !existing.is_empty() {
        return;
    }
    // The page-selected robot, or the bundled NAO.
    let custom = DEMO_URDF.lock().unwrap_or_else(|p| p.into_inner()).clone();
    let urdf = custom.as_deref().unwrap_or(NAO_URDF);
    let Ok(model) = RobotModel::from_urdf_str(urdf).inspect_err(|e| {
        tracing::error!("demo URDF failed to parse: {e}");
    }) else {
        return;
    };

    // Fit the animation to the robot: NAO waves; any other robot gets a
    // gentle sweep of its own joints so it looks alive without naming them.
    let script = if custom.is_some() {
        scripts::sine_sweep(model.joint_names(), 0.5)
    } else {
        scripts::nao_wave()
    };
    commands.insert_resource(DemoScript(Mutex::new(script)));

    spawn_robot(
        &mut commands,
        &mut meshes,
        &mut materials,
        Arc::new(model),
        &blobs.apply(MeshResolver::default()),
    );
}

fn animate_demo(
    time: Res<Time>,
    script: Res<DemoScript>,
    mut joint_positions: ResMut<JointPositions>,
) {
    let mut script = script.0.lock().unwrap_or_else(|p| p.into_inner());
    for (name, position) in script(time.elapsed_secs_f64()) {
        joint_positions.positions.insert(name, position);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bundled_nao_parses() {
        let model = RobotModel::from_urdf_str(NAO_URDF).expect("parses");
        assert_eq!(model.urdf.links.len(), 83);
        // The wave script only names real NAO joints.
        let joints = model.joint_names();
        let mut wave = scripts::nao_wave();
        for (name, _) in wave(0.0) {
            assert!(joints.contains(&name), "unknown joint in wave: {name}");
        }
    }

    #[test]
    fn scripts_produce_expected_joints() {
        let mut wave = scripts::nao_wave();
        assert!(wave(0.0).iter().any(|(n, _)| n == "RShoulderPitch"));

        let mut sweep = scripts::sine_sweep(vec!["a".into(), "b".into()], 1.0);
        assert_eq!(sweep(0.0).len(), 2);

        let mut fixed = scripts::static_pose(vec![("j".into(), 0.5)]);
        assert_eq!(fixed(123.0), vec![("j".to_string(), 0.5)]);
    }

    #[test]
    fn respawn_reloads_robot_with_uploaded_blob() {
        // The behaviour an upload triggers: a blob is recorded and the robot
        // despawned; the spawn system rebuilds it with the blob applied. We
        // drive that state directly rather than through the process-global
        // upload queue, which would race other parallel tests (the queue is
        // for the single-app wasm runtime; here it stays empty, so the drain
        // system is an inert no-op).
        use crate::scene::MeshBlobs;

        let mut app = App::new();
        app.add_plugins(MinimalPlugins);
        app.add_plugins(AssetPlugin::default());
        app.init_asset::<Mesh>();
        app.init_asset::<StandardMaterial>();
        app.add_plugins(crate::scene::RobotScenePlugin);
        app.add_plugins(DemoPlugin);

        let count = |app: &mut App| {
            app.world_mut()
                .query_filtered::<(), With<RobotHandle>>()
                .iter(app.world())
                .count()
        };
        app.update();
        assert_eq!(count(&mut app), 1, "demo robot spawns");

        // Record an uploaded mesh and despawn the robot, as the drain does.
        app.world_mut()
            .resource_mut::<MeshBlobs>()
            .0
            .insert("Head.dae".into(), b"solid s\nendsolid s\n".to_vec());
        let robots: Vec<Entity> = app
            .world_mut()
            .query_filtered::<Entity, With<RobotHandle>>()
            .iter(app.world())
            .collect();
        for entity in robots {
            app.world_mut().entity_mut(entity).despawn();
        }

        app.update();
        assert_eq!(count(&mut app), 1, "robot respawns with the blob present");
        assert!(
            app.world()
                .resource::<MeshBlobs>()
                .0
                .contains_key("Head.dae"),
            "uploaded mesh stays recorded across respawn"
        );
    }

    #[test]
    fn demo_app_animates_joint_positions() {
        let mut app = App::new();
        app.add_plugins(MinimalPlugins);
        app.add_plugins(AssetPlugin::default());
        app.init_asset::<Mesh>();
        app.init_asset::<StandardMaterial>();
        app.add_plugins(crate::scene::RobotScenePlugin);
        app.add_plugins(DemoPlugin);

        app.update();
        // Advance time a few frames so the script runs with t > 0.
        for _ in 0..3 {
            std::thread::sleep(std::time::Duration::from_millis(5));
            app.update();
        }
        let positions = &app.world().resource::<JointPositions>().positions;
        assert!(
            positions.contains_key("RElbowRoll"),
            "wave script should be driving joints, got: {positions:?}"
        );
    }
}
