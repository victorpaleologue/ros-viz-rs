//! Docker-backed end-to-end tests: real ROS 2 robot stacks in containers,
//! rendered by the real visualizer binary on the host.
//!
//! Each test builds an image from `docker/<robot>/`, starts it with
//! `--network host` on a random DDS domain, waits for its Docker healthcheck
//! (topics discoverable), then drives the actual CLI —
//! `cargo run -- --snapshot-to <png> --domain <id>` — and asserts a robot is
//! visible in the PNG with [`ros_viz_rs::vision::silhouette`] (same pattern
//! as `tests/ros_e2e.rs`).
//!
//! # Running
//!
//! The tests are `#[ignore]`d so that plain `cargo test` stays
//! self-contained (no Docker, no network):
//!
//! ```bash
//! cargo test --test docker_integration -- --ignored
//! ```
//!
//! First run builds the images (the fake NAO one colcon-builds
//! `naoqi_driver` from source: ~10–30 min depending on the machine); later
//! runs reuse the Docker build cache.
//!
//! # Linux only
//!
//! The containers must share the host's network for DDS multicast discovery
//! to reach the visualizer, which only `--network host` on a Linux host
//! provides. On macOS/Windows, Docker runs containers in a VM and "host"
//! networking attaches to the VM's stack, not the real host
//! (<https://docs.docker.com/engine/network/drivers/host/>), so these tests
//! skip there. They also skip when `docker` is unavailable or when the host
//! cannot do DDS multicast at all.

use std::path::{Path, PathBuf};
use std::process::Command;
use std::sync::{Mutex, OnceLock};
use std::time::{Duration, Instant};

use ros_viz_rs::vision;

/// How long a container may take to become healthy after `docker run`.
///
/// The images are already built at this point; this only covers node
/// startup plus the healthcheck's `ros2 topic list` discovery round-trips.
const HEALTHY_TIMEOUT: Duration = Duration::from_secs(120);

/// One GPU app (and one `cargo run` target-dir build) at a time per process.
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

fn manifest_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
}

/// Output directory for snapshots, uploaded as a CI artifact on failure.
fn output_dir() -> PathBuf {
    let dir = manifest_dir().join(".test_outputs/docker_integration");
    std::fs::create_dir_all(&dir).expect("create output dir");
    dir
}

fn docker_available() -> bool {
    Command::new("docker")
        .arg("version")
        .output()
        .map(|out| out.status.success())
        .unwrap_or(false)
}

/// Skip (early-return) when this host cannot run the Docker DDS round-trip.
macro_rules! require_docker_dds_host {
    () => {
        if cfg!(target_os = "macos") || cfg!(target_os = "windows") {
            eprintln!(
                "SKIPPED: on macOS/Windows, Docker containers run in a VM and \
                 --network host cannot reach this host's DDS participants; \
                 this end-to-end test is Linux-only."
            );
            return;
        }
        if !docker_available() {
            eprintln!("SKIPPED: `docker` is not available on this host.");
            return;
        }
        ros_viz_rs::require_dds_multicast!();
    };
}

/// A running container, force-removed on drop so failures never leak it.
struct Container {
    name: String,
}

impl Container {
    /// `docker build` the context, then `docker run` it detached with host
    /// networking on the given DDS domain.
    fn start(image_tag: &str, context: &str, name_prefix: &str, domain: u16) -> Self {
        let context_dir = manifest_dir().join(context);
        run_logged(
            Command::new("docker")
                .args(["build", "-t", image_tag])
                .arg(&context_dir),
            "docker build",
        );

        let name = format!("{name_prefix}-{}-{domain}", std::process::id());
        let container = Self { name: name.clone() };
        run_logged(
            Command::new("docker").args([
                "run",
                "-d",
                "--name",
                &name,
                "--network",
                "host",
                "-e",
                &format!("ROS_DOMAIN_ID={domain}"),
                image_tag,
            ]),
            "docker run",
        );
        container
    }

    /// Wait for the image's HEALTHCHECK to report `healthy`.
    fn wait_healthy(&self, timeout: Duration) {
        let deadline = Instant::now() + timeout;
        loop {
            let output = Command::new("docker")
                .args([
                    "inspect",
                    "--format",
                    "{{.State.Health.Status}}",
                    &self.name,
                ])
                .output()
                .expect("docker inspect runs");
            let status = String::from_utf8_lossy(&output.stdout).trim().to_string();
            match status.as_str() {
                "healthy" => return,
                "unhealthy" => self.dump_logs_and_panic("container reported unhealthy"),
                _ => {}
            }
            if Instant::now() > deadline {
                self.dump_logs_and_panic(&format!("container not healthy within {timeout:?}"));
            }
            std::thread::sleep(Duration::from_secs(2));
        }
    }

    /// Copy a path out of the container (e.g. an installed mesh package).
    fn copy_out(&self, container_path: &str, host_dir: &Path) {
        run_logged(
            Command::new("docker")
                .arg("cp")
                .arg(format!("{}:{container_path}", self.name))
                .arg(host_dir),
            "docker cp",
        );
    }

    fn dump_logs_and_panic(&self, reason: &str) -> ! {
        let logs = Command::new("docker")
            .args(["logs", &self.name])
            .output()
            .expect("docker logs runs");
        panic!(
            "{reason} ({})\n--- container stdout ---\n{}\n--- container stderr ---\n{}",
            self.name,
            String::from_utf8_lossy(&logs.stdout),
            String::from_utf8_lossy(&logs.stderr),
        );
    }
}

impl Drop for Container {
    fn drop(&mut self) {
        let _ = Command::new("docker")
            .args(["rm", "-f", &self.name])
            .output();
    }
}

/// Run a command inheriting stdio (so CI logs show build progress) and
/// panic with context if it fails.
fn run_logged(command: &mut Command, what: &str) {
    let status = command.status().expect("command spawns");
    assert!(status.success(), "{what} failed: {command:?}");
}

/// Drive the real CLI end to end: `cargo run -- --snapshot-to ... --domain ...`.
fn snapshot_via_cli(domain: u16, png: &Path, extra_args: &[String]) {
    let cargo = env!("CARGO");
    let mut command = Command::new(cargo);
    command
        .current_dir(manifest_dir())
        .args(["run", "--quiet", "--"])
        .arg("--snapshot-to")
        .arg(png)
        .args(["--domain", &domain.to_string()])
        .args(["--width", "640", "--height", "480"])
        .args(extra_args);
    run_logged(&mut command, "cargo run --snapshot-to");
}

/// Same structural assertions as `tests/ros_e2e.rs`: something robot-shaped
/// is visible and roughly framed.
fn assert_robot_visible(png: &Path, what: &str) {
    let img = image::open(png).expect("snapshot readable").to_rgba8();
    let background = *img.get_pixel(0, 0);
    let silhouette = vision::silhouette(&img, background, 12);
    assert!(
        silhouette.coverage > 0.002,
        "{what} should be visible in the render (coverage {:.4}%)",
        silhouette.coverage * 100.0
    );
    let (x_min, y_min, x_max, y_max) = silhouette.bbox.expect("silhouette has a bounding box");
    let (cx, cy) = (
        (x_min + x_max) as f64 / 2.0 / 640.0,
        (y_min + y_max) as f64 / 2.0 / 480.0,
    );
    assert!(
        (0.15..=0.85).contains(&cx) && (0.15..=0.85).contains(&cy),
        "{what} should be roughly framed (center {cx:.2},{cy:.2})"
    );
}

#[test]
#[ignore = "needs Docker + Linux host networking; run with --ignored"]
fn ur5e_container_renders_through_full_app() {
    require_docker_dds_host!();
    let domain = random_domain_id();

    let container = Container::start("ros-viz-test-ur5e", "docker/ur5e", "ros-viz-ur5e", domain);
    container.wait_healthy(HEALTHY_TIMEOUT);

    // The UR5e URDF references its visual meshes as
    // `package://ur_description/...`; resolve them from the container's own
    // installed copy so the host renders the real robot geometry.
    let meshes = tempfile::tempdir().expect("tempdir");
    container.copy_out("/opt/ros/jazzy/share/ur_description", meshes.path());

    let png = output_dir().join("ur5e.png");
    {
        let _gpu = gpu_lock();
        snapshot_via_cli(
            domain,
            &png,
            &[format!(
                "--package=ur_description={}",
                meshes.path().join("ur_description").display()
            )],
        );
    }
    assert_robot_visible(&png, "the UR5e arm");
}

#[test]
#[ignore = "needs Docker + Linux host networking; run with --ignored"]
fn fake_nao_container_renders_through_full_app() {
    require_docker_dds_host!();
    let domain = random_domain_id();

    let container = Container::start(
        "ros-viz-test-fake-nao",
        "docker/fake_nao",
        "ros-viz-fake-nao",
        domain,
    );
    container.wait_healthy(HEALTHY_TIMEOUT);

    // No mesh package on purpose: the NAO description references
    // `package://nao_meshes/...`, which is not installed; links render as
    // skeleton markers, which the existing `nao_skeleton_renders` visual
    // test shows is enough silhouette to assert on.
    let png = output_dir().join("fake_nao.png");
    {
        let _gpu = gpu_lock();
        snapshot_via_cli(domain, &png, &[]);
    }
    assert_robot_visible(&png, "the fake NAO");
}
