//! Docker-backed end-to-end test for the **Zenoh** transport.
//!
//! Builds `docker/zenoh` (ROS 2 Jazzy + rmw_zenoh: a Zenoh router plus
//! robot_state_publisher and joint_state_publisher), starts it with the
//! router port published, then drives the real app — [`ros_viz_rs::app::run`]
//! with `--zenoh` in snapshot mode — and asserts the robot is visible in the
//! PNG. Proves the modularity claim: a brand-new transport on the
//! [`topics`](ros_viz_rs::topics) seam renders a robot with the rest of the
//! app unchanged.
//!
//! Unlike the DDS [`docker_integration`] tests, this connects to the router
//! over **TCP** (a published port), so it needs no `--network host` and runs
//! on macOS/Windows too — only Docker is required.
//!
//! Only compiled with `--features zenoh`, and `#[ignore]`d so plain
//! `cargo test` stays self-contained:
//!
//! ```bash
//! cargo test --features zenoh --test zenoh_integration -- --ignored
//! ```
#![cfg(feature = "zenoh")]

use std::path::PathBuf;
use std::process::Command;
use std::thread::sleep;
use std::time::Duration;

use ros_viz_rs::options::Options;
use ros_viz_rs::vision;

const ROUTER_PORT: &str = "7447";

fn docker_available() -> bool {
    Command::new("docker")
        .arg("version")
        .output()
        .map(|o| o.status.success())
        .unwrap_or(false)
}

/// A container force-removed on drop so a failure never leaks it.
struct Container {
    name: String,
}

impl Drop for Container {
    fn drop(&mut self) {
        let _ = Command::new("docker")
            .args(["rm", "-f", &self.name])
            .output();
    }
}

#[test]
#[ignore = "requires Docker (builds a ROS 2 + rmw_zenoh image)"]
fn zenoh_robot_renders_through_full_app() {
    if !docker_available() {
        eprintln!("SKIPPED: docker is not available on this host.");
        return;
    }

    let manifest = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let context = manifest.join("docker/zenoh");
    let build = Command::new("docker")
        .args(["build", "-t", "ros-viz-zenoh"])
        .arg(&context)
        .status()
        .expect("docker build");
    assert!(build.success(), "docker build failed");

    let name = format!("ros-viz-zenoh-{}", std::process::id());
    let container = Container { name: name.clone() };
    let run = Command::new("docker")
        .args([
            "run",
            "-d",
            "--name",
            &name,
            "-p",
            &format!("{ROUTER_PORT}:{ROUTER_PORT}"),
            "ros-viz-zenoh",
        ])
        .status()
        .expect("docker run");
    assert!(run.success(), "docker run failed");

    // Give the router + publishers a moment to come up; the app then waits
    // up to its own internal timeout for the latched description.
    sleep(Duration::from_secs(8));

    let out_dir = manifest.join(".test_outputs/zenoh_integration");
    std::fs::create_dir_all(&out_dir).expect("create out dir");
    let png = out_dir.join("zenoh_robot.png");

    let options = Options {
        zenoh: Some(format!("tcp/localhost:{ROUTER_PORT}")),
        snapshot_to: Some(png.clone()),
        width: 640,
        height: 480,
        ..Options::default()
    };
    // Snapshot mode blocks until a robot is received (or its timeout) and
    // writes the PNG.
    ros_viz_rs::app::run(options).expect("app renders the robot over Zenoh");
    drop(container);

    let img = image::open(&png).expect("snapshot readable").to_rgba8();
    let background = *img.get_pixel(0, 0);
    let silhouette = vision::silhouette(&img, background, 12);
    assert!(
        silhouette.coverage > 0.002,
        "robot should be visible in the Zenoh render (coverage {:.4}%)",
        silhouette.coverage * 100.0
    );
    let (x_min, y_min, x_max, y_max) = silhouette
        .bbox
        .expect("robot silhouette has a bounding box");
    let (cx, cy) = (
        (x_min + x_max) as f64 / 2.0 / 640.0,
        (y_min + y_max) as f64 / 2.0 / 480.0,
    );
    assert!(
        (0.15..=0.85).contains(&cx) && (0.15..=0.85).contains(&cy),
        "robot should be roughly framed (center {cx:.2},{cy:.2})"
    );
}
