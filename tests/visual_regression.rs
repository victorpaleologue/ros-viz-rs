//! Headless visual regression tests.
//!
//! Each test renders a URDF fixture entirely offscreen (no window, real GPU
//! via [`ros_viz_rs::snapshot`]), then checks the pixels two ways:
//!
//! 1. **Structurally** with [`ros_viz_rs::vision`]: a robot silhouette is
//!    visible, roughly centered and of sensible size — failures stay
//!    diagnosable without squinting at diffs.
//! 2. **Against a reference image** with an RMSE budget. Regenerate
//!    references after an intended rendering change with
//!    `ROS_VIZ_BLESS=1 cargo test --test visual_regression`.
//!
//! Artifacts (actual + diff PNGs) land in `.test_outputs/` on failure.

use std::path::Path;
use std::sync::{Arc, Mutex, OnceLock};

use bevy::ecs::system::SystemState;
use bevy::prelude::*;
use image::RgbaImage;

use ros_viz_rs::robot::RobotModel;
use ros_viz_rs::robot::mesh::MeshResolver;
use ros_viz_rs::scene::{
    AutoFrameCamera, JointPositions, RobotScenePlugin, spawn_lights, spawn_robot,
};
use ros_viz_rs::snapshot::{SnapshotCamera, SnapshotPlugin, capture};
use ros_viz_rs::vision;

const WIDTH: u32 = 800;
const HEIGHT: u32 = 600;
/// Tolerates anti-aliasing and driver variations, not structural changes.
const MAX_RMSE: f64 = 0.02;

/// One GPU app at a time per process keeps captures deterministic.
fn gpu_lock() -> std::sync::MutexGuard<'static, ()> {
    static LOCK: OnceLock<Mutex<()>> = OnceLock::new();
    match LOCK.get_or_init(|| Mutex::new(())).lock() {
        Ok(guard) => guard,
        Err(poisoned) => poisoned.into_inner(),
    }
}

/// Render a URDF fixture offscreen with optional joint positions.
fn render_urdf(urdf_path: &str, joints: &[(&str, f64)]) -> RgbaImage {
    let model = Arc::new(RobotModel::from_urdf_file(urdf_path).expect("URDF parses"));
    let resolver = MeshResolver::for_urdf_file(urdf_path);

    let mut positions = JointPositions::default();
    for (name, value) in joints {
        positions.positions.insert(name.to_string(), *value);
    }

    let mut app = App::new();
    app.add_plugins(SnapshotPlugin {
        width: WIDTH,
        height: HEIGHT,
    });
    app.add_plugins(RobotScenePlugin);
    app.insert_resource(ClearColor(Color::srgb(0.13, 0.14, 0.17)));
    app.insert_resource(positions);

    // Make the snapshot camera auto-frame the robot once it has bounds.
    app.add_systems(
        Update,
        |mut commands: Commands,
         cameras: Query<Entity, (With<SnapshotCamera>, Without<AutoFrameCamera>)>| {
            for entity in cameras.iter() {
                commands.entity(entity).insert(AutoFrameCamera);
            }
        },
    );

    type SpawnParams<'w, 's> = (
        Commands<'w, 's>,
        ResMut<'w, Assets<Mesh>>,
        ResMut<'w, Assets<StandardMaterial>>,
    );
    let world = app.world_mut();
    let mut state: SystemState<SpawnParams> = SystemState::new(world);
    {
        let (mut commands, mut meshes, mut materials) = state.get_mut(world);
        spawn_lights(&mut commands);
        spawn_robot(&mut commands, &mut meshes, &mut materials, model, &resolver);
    }
    state.apply(world);

    capture(&mut app, 12).expect("offscreen capture succeeds")
}

/// Structural sanity: something robot-shaped is visible and framed.
fn assert_robot_visible(img: &RgbaImage, name: &str) {
    // The corner pixel is reliable background (robots are auto-framed
    // around the center).
    let background = *img.get_pixel(0, 0);
    let s = vision::silhouette(img, background, 12);
    assert!(
        s.coverage > 0.002,
        "{name}: robot covers only {:.4}% of the frame",
        s.coverage * 100.0
    );
    assert!(
        s.coverage < 0.75,
        "{name}: 'robot' covers {:.1}% — camera inside geometry?",
        s.coverage * 100.0
    );
    let (x_min, y_min, x_max, y_max) = s.bbox.expect("bbox exists when coverage > 0");
    let (cx, cy) = (
        (x_min + x_max) as f64 / 2.0 / WIDTH as f64,
        (y_min + y_max) as f64 / 2.0 / HEIGHT as f64,
    );
    assert!(
        (0.2..=0.8).contains(&cx) && (0.2..=0.8).contains(&cy),
        "{name}: silhouette center ({cx:.2}, {cy:.2}) is off-frame"
    );
}

fn check_against_reference(img: &RgbaImage, name: &str) {
    let reference = Path::new("test-data/reference").join(format!("{name}.png"));
    let artifacts = Path::new(".test_outputs");
    vision::assert_matches_reference(img, &reference, MAX_RMSE, artifacts)
        .unwrap_or_else(|e| panic!("{name}: {e}"));
}

fn visual_test(urdf: &str, name: &str, joints: &[(&str, f64)]) {
    let _gpu = gpu_lock();
    let img = render_urdf(urdf, joints);
    assert_robot_visible(&img, name);
    check_against_reference(&img, name);
}

#[test]
fn simple_arm_renders() {
    visual_test("test-data/urdf/simple_arm.urdf", "simple_arm", &[]);
}

#[test]
fn two_link_planar_renders() {
    visual_test(
        "test-data/urdf/two_link_planar.urdf",
        "two_link_planar",
        &[],
    );
}

#[test]
fn two_link_planar_bends() {
    visual_test(
        "test-data/urdf/two_link_planar.urdf",
        "two_link_planar_bent",
        &[("joint2", std::f64::consts::FRAC_PI_2)],
    );
}

#[test]
fn triple_pendulum_renders() {
    visual_test(
        "test-data/urdf/triple_pendulum.urdf",
        "triple_pendulum",
        &[],
    );
}

#[test]
fn box_bot_renders() {
    visual_test("test-data/urdf/box_bot.urdf", "box_bot", &[]);
}

#[test]
fn nao_skeleton_renders() {
    // Without the nao_meshes package on disk the NAO renders as fallback
    // markers — still a strong structural check on FK and spawning.
    visual_test("assets/nao_robot.urdf", "nao_robot", &[]);
}

#[test]
fn nao_posed_differs_from_rest() {
    let _gpu = gpu_lock();
    let rest = render_urdf("assets/nao_robot.urdf", &[]);
    let posed = render_urdf(
        "assets/nao_robot.urdf",
        &[("LShoulderPitch", -1.5), ("HeadYaw", 0.8)],
    );
    let rmse = vision::rmse(&rest, &posed).expect("same dimensions");
    assert!(
        rmse > 0.004,
        "posing joints changed almost nothing (rmse {rmse:.5}) — FK broken?"
    );
}

#[test]
fn demo_mode_renders_and_waves() {
    let _gpu = gpu_lock();
    let mut app = App::new();
    app.add_plugins(SnapshotPlugin {
        width: WIDTH,
        height: HEIGHT,
    });
    app.add_plugins(RobotScenePlugin);
    app.add_plugins(ros_viz_rs::demo::DemoPlugin);
    app.insert_resource(ClearColor(Color::srgb(0.13, 0.14, 0.17)));
    app.add_systems(
        Update,
        |mut commands: Commands,
         cameras: Query<Entity, (With<SnapshotCamera>, Without<AutoFrameCamera>)>| {
            for entity in cameras.iter() {
                commands.entity(entity).insert(AutoFrameCamera);
            }
        },
    );
    let lights = |mut commands: Commands| spawn_lights(&mut commands);
    app.add_systems(Startup, lights);

    let early = capture(&mut app, 12).expect("first capture");
    assert_robot_visible(&early, "demo_nao");
    // Half a second later the wave script is in a different phase.
    std::thread::sleep(std::time::Duration::from_millis(500));
    let later = capture(&mut app, 2).expect("second capture");
    let rmse = vision::rmse(&early, &later).expect("same dimensions");
    assert!(
        rmse > 0.001,
        "the demo NAO should be waving (rmse {rmse:.5} between frames)"
    );
}

#[test]
fn rendering_is_deterministic() {
    let _gpu = gpu_lock();
    let a = render_urdf("test-data/urdf/simple_arm.urdf", &[]);
    let b = render_urdf("test-data/urdf/simple_arm.urdf", &[]);
    let rmse = vision::rmse(&a, &b).expect("same dimensions");
    assert!(
        rmse < 0.001,
        "two identical renders differ (rmse {rmse:.5}) — nondeterminism"
    );
}
