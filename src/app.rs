//! The ros-viz-rs application: connect to ROS 2, receive the robot
//! description and joint states, and render the robot live.
//!
//! Two modes share the same systems:
//!
//! - **Windowed** (default): a Bevy window with the 3D view and an egui
//!   topics panel.
//! - **Snapshot** (`--snapshot-to <PATH>`): completely windowless; waits for
//!   `/robot_description`, renders one frame offscreen with a real GPU and
//!   writes it to disk — handy for headless checks of a live system.

#[cfg(feature = "ros2")]
use std::sync::Arc;
use std::time::Duration;
#[cfg(not(target_arch = "wasm32"))]
use std::time::Instant;

use bevy::prelude::*;
use bevy_egui::EguiPlugin;
#[cfg(feature = "ros2")]
use ros2_client::ros2::{
    Duration as RosDuration, QosPolicyBuilder,
    policy::{Durability, History, Reliability},
};
#[cfg(not(target_arch = "wasm32"))]
use tracing_subscriber::EnvFilter;

use crate::options::Options;
#[cfg(feature = "ros2")]
use crate::robot::RobotModel;
#[cfg(any(feature = "ros2", feature = "rosbridge"))]
use crate::robot::mesh::MeshResolver;
#[cfg(feature = "ros2")]
use crate::ros_msgs;
#[cfg(feature = "ros2")]
use crate::ros_plugin::{RosPlugin, RosSession};
#[cfg(feature = "ros2")]
use crate::scene::JointPositions;
#[cfg(any(feature = "ros2", feature = "rosbridge"))]
use crate::scene::PendingRobot;
#[cfg(any(feature = "ros2", feature = "rosbridge"))]
use crate::scene::spawn_robot;
use crate::scene::{RobotHandle, RobotScenePlugin, spawn_viewing_rig};
#[cfg(not(target_arch = "wasm32"))]
use crate::snapshot::{self, SnapshotPlugin};
#[cfg(feature = "ros2")]
use crate::topics_io::{TopicIOPlugin, setup_typed_subscription};
use crate::topics_view::{TopicsPanelMode, TopicsTreePlugin};

/// Typed subscriptions for `/robot_description` and `/joint_states`.
///
/// Created lazily once [`RosSession`] is available. Receivers are wrapped in
/// [`std::sync::Mutex`] because subscriptions are not `Sync`, which Bevy
/// resources require.
#[cfg(feature = "ros2")]
#[derive(Resource)]
struct AppSubscriptions {
    robot_description: std::sync::Mutex<ros2_client::Subscription<ros_msgs::String>>,
    joint_states: std::sync::Mutex<ros2_client::Subscription<ros_msgs::JointState>>,
}

/// Warns once when no description shows up within a grace period.
#[cfg_attr(not(feature = "ros2"), allow(dead_code))]
#[derive(Resource)]
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

/// Initialize tracing and run the app in the mode selected by `options`.
pub fn run(options: Options) -> anyhow::Result<()> {
    init_tracing();
    tracing::info!("Starting ros-viz-rs with {options:?}");

    #[cfg(not(target_arch = "wasm32"))]
    if let Some(path) = options.snapshot_to.clone() {
        let mut app = build_app(&options);
        return run_headless_snapshot(&mut app, &path, Duration::from_secs(30));
    }
    build_app(&options).run();
    Ok(())
}

/// Build the Bevy app for the selected mode (windowed unless
/// `options.snapshot_to` is set).
pub fn build_app(options: &Options) -> App {
    let mut app = App::new();
    app.insert_resource(options.clone());
    app.init_resource::<UrdfWaitTimer>();

    #[cfg(target_arch = "wasm32")]
    let windowed = true;
    #[cfg(not(target_arch = "wasm32"))]
    let windowed = options.snapshot_to.is_none();

    #[cfg(not(target_arch = "wasm32"))]
    if !windowed {
        app.add_plugins(SnapshotPlugin {
            width: options.width,
            height: options.height,
        });
        // Auto-frame the offscreen camera on the robot when it spawns.
        app.add_systems(
            Update,
            |mut commands: Commands,
             cameras: Query<
                Entity,
                (
                    With<crate::snapshot::SnapshotCamera>,
                    Without<crate::scene::AutoFrameCamera>,
                ),
            >| {
                for entity in cameras.iter() {
                    commands
                        .entity(entity)
                        .insert(crate::scene::AutoFrameCamera);
                }
            },
        );
        app.add_systems(Startup, |mut commands: Commands| {
            crate::scene::spawn_lights(&mut commands);
        });
    }
    if windowed {
        #[allow(unused_mut)]
        let mut window = bevy::window::Window {
            title: "ros-viz-rs".into(),
            resolution: bevy::window::WindowResolution::new(options.width, options.height),
            ..Default::default()
        };
        // On the web, render into the page's canvas and track its size.
        #[cfg(target_arch = "wasm32")]
        {
            window.canvas = Some("#ros-viz-canvas".into());
            window.fit_canvas_to_parent = true;
            window.prevent_default_event_handling = false;
        }
        app.add_plugins(DefaultPlugins.set(bevy::window::WindowPlugin {
            primary_window: Some(window),
            ..Default::default()
        }));
        app.add_plugins(EguiPlugin::default());
        app.add_plugins(TopicsTreePlugin {
            panel_mode: TopicsPanelMode::Side,
        });
        app.add_systems(Startup, |mut commands: Commands| {
            spawn_viewing_rig(&mut commands);
        });
    }

    app.insert_resource(ClearColor(Color::srgb(0.13, 0.14, 0.17)));
    app.add_plugins(RobotScenePlugin);

    if options.demo {
        app.add_plugins(crate::demo::DemoPlugin);
        return app;
    }

    #[cfg(feature = "rosbridge")]
    if let Some(url) = options.rosbridge.clone() {
        app.add_plugins(crate::rosbridge::RosbridgePlugin { url });
        app.add_systems(Update, spawn_pending_robot);
        return app;
    }

    #[cfg(feature = "ros2")]
    match RosPlugin::new(options.domain, "ros_viz_rs") {
        Ok(ros) => {
            app.add_plugins((ros, TopicIOPlugin));
            app.add_systems(
                Update,
                (
                    setup_app_subscriptions,
                    receive_robot_description,
                    receive_joint_states,
                    spawn_pending_robot,
                )
                    .chain(),
            );
        }
        Err(e) => {
            tracing::error!("ROS connection unavailable: {e}");
        }
    }
    #[cfg(not(feature = "ros2"))]
    tracing::warn!("built without the ros2 feature: no DDS connection");

    app
}

/// Drive a snapshot-mode app until a robot is rendered, then write the PNG.
#[cfg(not(target_arch = "wasm32"))]
fn run_headless_snapshot(
    app: &mut App,
    path: &std::path::Path,
    timeout: Duration,
) -> anyhow::Result<()> {
    snapshot::ensure_ready(app);
    let deadline = Instant::now() + timeout;
    loop {
        app.update();
        let robot_spawned = app
            .world_mut()
            .query::<&RobotHandle>()
            .iter(app.world())
            .next()
            .is_some();
        if robot_spawned {
            break;
        }
        anyhow::ensure!(
            Instant::now() < deadline,
            "no robot received from ROS within {timeout:?}; \
             is something publishing /robot_description on this domain?"
        );
        std::thread::sleep(Duration::from_millis(15));
    }

    let image = snapshot::capture(app, 12)?;
    snapshot::save_png(&image, path)?;
    tracing::info!(?path, "snapshot written");
    Ok(())
}

/// Lazily create the typed subscriptions once [`RosSession`] exists.
#[cfg(feature = "ros2")]
fn setup_app_subscriptions(world: &mut World) {
    if world.get_resource::<AppSubscriptions>().is_some()
        || world.get_resource::<RosSession>().is_none()
    {
        return;
    }
    let session = world.resource::<RosSession>().clone();

    // robot_description is latched: publishers keep the last message for
    // late joiners, which requires TransientLocal durability on our side.
    let latched = QosPolicyBuilder::new()
        .durability(Durability::TransientLocal)
        .history(History::KeepLast { depth: 1 })
        .reliability(Reliability::Reliable {
            max_blocking_time: RosDuration::from_secs(1),
        })
        .build();

    let robot_description = match setup_typed_subscription::<ros_msgs::String>(
        &session.node,
        "/robot_description",
        Some(&latched),
    ) {
        Ok(sub) => sub,
        Err(e) => {
            tracing::warn!("Failed to subscribe to /robot_description: {e}");
            return;
        }
    };
    let joint_states = match setup_typed_subscription::<ros_msgs::JointState>(
        &session.node,
        "/joint_states",
        None,
    ) {
        Ok(sub) => sub,
        Err(e) => {
            tracing::warn!("Failed to subscribe to /joint_states: {e}");
            return;
        }
    };

    world.insert_resource(AppSubscriptions {
        robot_description: std::sync::Mutex::new(robot_description),
        joint_states: std::sync::Mutex::new(joint_states),
    });
    tracing::info!("Subscribed to /robot_description and /joint_states");
}

/// Receive the URDF, parse it into a [`RobotModel`].
#[cfg(feature = "ros2")]
fn receive_robot_description(
    mut pending: ResMut<PendingRobot>,
    robots: Query<&RobotHandle>,
    subs: Option<Res<AppSubscriptions>>,
    options: Res<Options>,
    mut wait: ResMut<UrdfWaitTimer>,
    time: Res<Time>,
) {
    if pending.0.is_some() || !robots.is_empty() {
        return;
    }
    wait.timer.tick(time.delta());

    let Some(subs) = subs.as_ref() else {
        return;
    };
    let sub = subs
        .robot_description
        .lock()
        .unwrap_or_else(|p| p.into_inner());
    let mut latest = None;
    while let Ok(Some((msg, _))) = sub.take() {
        latest = Some(msg.data);
    }
    if let Some(xml) = latest {
        match RobotModel::from_urdf_str(&xml) {
            Ok(model) => {
                tracing::info!(
                    "Received robot '{}': {} links, {} joints",
                    model.name(),
                    model.urdf.links.len(),
                    model.urdf.joints.len()
                );
                pending.0 = Some(Arc::new(model));
            }
            Err(err) => tracing::error!(?err, "Failed to parse /robot_description"),
        }
    } else if wait.timer.is_finished() && !wait.warned {
        wait.warned = true;
        tracing::warn!(
            "No /robot_description after {}s on domain {}. Common publishers: \
             robot_state_publisher (latched topic).",
            wait.timer.duration().as_secs(),
            options.domain,
        );
    }
}

/// Feed `/joint_states` into the scene's [`JointPositions`].
#[cfg(feature = "ros2")]
fn receive_joint_states(
    mut joint_positions: ResMut<JointPositions>,
    subs: Option<Res<AppSubscriptions>>,
) {
    let Some(subs) = subs.as_ref() else { return };
    let sub = subs.joint_states.lock().unwrap_or_else(|p| p.into_inner());
    let mut latest = None;
    while let Ok(Some((msg, _))) = sub.take() {
        latest = Some(msg);
    }
    if let Some(msg) = latest {
        for (name, position) in msg.name.iter().zip(msg.position.iter()) {
            joint_positions.positions.insert(name.clone(), *position);
        }
    }
}

/// Spawn the robot scene once a model has been received.
#[cfg(any(feature = "ros2", feature = "rosbridge"))]
fn spawn_pending_robot(
    mut commands: Commands,
    mut pending: ResMut<PendingRobot>,
    robots: Query<&RobotHandle>,
    options: Res<Options>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let Some(model) = pending.0.take() else {
        return;
    };
    if !robots.is_empty() {
        return;
    }
    let mut resolver = MeshResolver::default();
    resolver.fallback_dirs.extend(std::env::current_dir().ok());
    for spec in &options.package {
        if let Some((name, path)) = spec.split_once('=') {
            resolver = resolver.with_package(name, path);
        } else {
            tracing::warn!("--package expects NAME=PATH, got '{spec}'");
        }
    }
    tracing::info!("Spawning robot '{}'", model.name());
    spawn_robot(&mut commands, &mut meshes, &mut materials, model, &resolver);
}

#[cfg(not(target_arch = "wasm32"))]
fn init_tracing() {
    let env_filter = EnvFilter::try_from_default_env().unwrap_or_else(|_| EnvFilter::new("info"));
    let _ = tracing_subscriber::fmt()
        .with_env_filter(env_filter)
        .try_init();
}

#[cfg(target_arch = "wasm32")]
fn init_tracing() {
    // Bevy's LogPlugin routes tracing to the browser console on wasm.
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Snapshot-mode app builds headless with all resources in place.
    #[test]
    fn builds_headless_app() {
        let options = Options {
            snapshot_to: Some("unused.png".into()),
            ..Options::default()
        };
        let app = build_app(&options);
        assert_eq!(
            app.world().get_resource::<Options>().cloned(),
            Some(options)
        );
        assert!(app.world().contains_resource::<JointPositions>());
        assert!(
            app.world()
                .get_resource::<PendingRobot>()
                .is_some_and(|p| p.0.is_none())
        );
    }

    /// Headless snapshot errors out when no robot arrives in time.
    #[test]
    fn headless_snapshot_times_out_without_robot() {
        let options = Options {
            snapshot_to: Some("unused.png".into()),
            // Random-ish high domain to avoid receiving a real robot.
            domain: 199,
            ..Options::default()
        };
        let mut app = build_app(&options);
        let err = run_headless_snapshot(
            &mut app,
            std::path::Path::new("unused.png"),
            Duration::from_millis(300),
        )
        .expect_err("must time out");
        assert!(err.to_string().contains("no robot received"));
    }
}
