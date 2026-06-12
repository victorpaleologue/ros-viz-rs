//! End-to-end test: a DDS-emulated robot is rendered by the full app.
//!
//! [`ros_viz_rs::emulator::Emulator`] publishes `/robot_description`
//! (latched) and `/joint_states` on a random DDS domain; the real
//! application ([`ros_viz_rs::app::run`] in `--snapshot-to` mode) joins the
//! domain, receives the robot, renders it offscreen on the GPU and writes a
//! PNG. The pixels are then checked structurally — the whole pipeline
//! (transport → URDF → FK → scene → render) in one assertion.

use std::collections::BTreeMap;
use std::sync::{Mutex, OnceLock};

use ros_viz_rs::emulator::{Emulator, EmulatorConfig, scripts};
use ros_viz_rs::options::Options;
use ros_viz_rs::vision;

const URDF: &str = include_str!("../test-data/urdf/two_link_planar.urdf");

/// One GPU app at a time per process.
fn gpu_lock() -> std::sync::MutexGuard<'static, ()> {
    static LOCK: OnceLock<Mutex<()>> = OnceLock::new();
    match LOCK.get_or_init(|| Mutex::new(())).lock() {
        Ok(guard) => guard,
        Err(poisoned) => poisoned.into_inner(),
    }
}

fn random_domain_id() -> u16 {
    use std::collections::hash_map::RandomState;
    use std::hash::{BuildHasher, Hasher};
    let hash = RandomState::new().build_hasher().finish();
    (hash % 232 + 1) as u16
}

fn snapshot_app(domain: u16, path: &std::path::Path) {
    let options = Options {
        domain,
        snapshot_to: Some(path.to_path_buf()),
        package: vec![],
        width: 640,
        height: 480,
    };
    ros_viz_rs::app::run(options).expect("app renders the emulated robot");
}

#[test]
fn emulated_robot_renders_through_full_app() {
    let _gpu = gpu_lock();
    let domain = random_domain_id();

    let _emulator = Emulator::spawn(
        EmulatorConfig::new(domain, URDF)
            .with_initial_joints(BTreeMap::from([
                ("joint1".to_string(), 0.0),
                ("joint2".to_string(), 0.0),
            ]))
            .with_script(scripts::static_pose(vec![
                ("joint1".into(), 0.3),
                ("joint2".into(), 1.2),
            ])),
    )
    .expect("emulator starts");

    let dir = tempfile::tempdir().expect("tempdir");
    let png = dir.path().join("e2e.png");
    snapshot_app(domain, &png);

    let img = image::open(&png).expect("snapshot readable").to_rgba8();
    let background = *img.get_pixel(0, 0);
    let silhouette = vision::silhouette(&img, background, 12);
    assert!(
        silhouette.coverage > 0.002,
        "robot should be visible in the end-to-end render (coverage {:.4}%)",
        silhouette.coverage * 100.0
    );
    let bbox = silhouette
        .bbox
        .expect("robot silhouette has a bounding box");
    let (x_min, y_min, x_max, y_max) = bbox;
    let (cx, cy) = (
        (x_min + x_max) as f64 / 2.0 / 640.0,
        (y_min + y_max) as f64 / 2.0 / 480.0,
    );
    assert!(
        (0.15..=0.85).contains(&cx) && (0.15..=0.85).contains(&cy),
        "robot should be roughly framed (center {cx:.2},{cy:.2})"
    );
}
