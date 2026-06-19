//! The ros-viz-rs application: connect to a data source, receive the robot
//! description and joint states, and render the robot live.
//!
//! Two modes share the same systems:
//!
//! - **Windowed** (default): a Bevy window with the 3D view, a connection
//!   panel and an egui topics panel.
//! - **Snapshot** (`--snapshot-to <PATH>`): completely windowless; waits for
//!   `/robot_description`, renders one frame offscreen with a real GPU and
//!   writes it to disk — handy for headless checks of a live system.
//!
//! The data source (demo / rosbridge / DDS) is chosen at startup from
//! [`Options`] and can be switched at runtime — see [`crate::connection`].

use std::time::Duration;
#[cfg(not(target_arch = "wasm32"))]
use std::time::Instant;

use bevy::prelude::*;
use bevy_egui::{EguiGlobalSettings, EguiPlugin, PrimaryEguiContext};
#[cfg(all(not(target_arch = "wasm32"), not(target_os = "android")))]
use tracing_subscriber::EnvFilter;

use crate::connection::{ConnectionPlugin, initial_mode};
use crate::options::Options;
use crate::robot::mesh::MeshResolver;
#[cfg(test)]
use crate::scene::JointPositions;
use crate::scene::{
    MeshBlobs, PendingRobot, RobotHandle, RobotScenePlugin, spawn_robot, spawn_viewing_rig,
};
#[cfg(not(target_arch = "wasm32"))]
use crate::snapshot::{self, SnapshotPlugin};
use crate::topics_view::{TopicsPanelMode, TopicsTreePlugin};

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
        // On Android, stay a normal windowed activity: the system status and
        // navigation bars are kept, and the surface is laid out within their
        // insets, so the egui top bar sits below the status bar instead of
        // behind it. (Borderless fullscreen pushed the surface edge-to-edge
        // under the bars, hiding the connection bar.)
        app.add_plugins(DefaultPlugins.set(bevy::window::WindowPlugin {
            primary_window: Some(window),
            ..Default::default()
        }));
        // Only redraw on input/events instead of continuously, to spare the
        // battery on mobile.
        #[cfg(target_os = "android")]
        app.insert_resource(bevy::winit::WinitSettings::mobile());
        app.add_plugins(EguiPlugin::default());
        // bevy_egui renders the UI through a camera carrying its primary
        // context. Its auto-creation attaches that context to a context entity
        // without a render graph, so egui drew nothing (no panels on any
        // platform). Disable the auto-creation and instead tag our own
        // `Camera3d` (which has a render graph) with `PrimaryEguiContext`.
        app.world_mut()
            .resource_mut::<EguiGlobalSettings>()
            .auto_create_primary_context = false;
        app.add_plugins(crate::camera::OrbitCameraPlugin);
        app.add_plugins(TopicsTreePlugin {
            panel_mode: TopicsPanelMode::Side,
        });
        // The runtime connection panel (switch demo / rosbridge / DDS).
        crate::connection_ui::register(&mut app, options);
        // On Android, send the app to the background on Back instead of
        // exiting — see `android_back_backgrounds` for why exiting crashes.
        #[cfg(target_os = "android")]
        app.add_systems(Update, android_back_backgrounds);
        // Hold a Wi-Fi multicast lock so native DDS discovery can receive peers
        // on the LAN (Android otherwise drops multicast). Best-effort.
        #[cfg(all(target_os = "android", feature = "ros2"))]
        app.add_systems(Startup, acquire_wifi_multicast_lock);
        // Reserve the system-bar / cutout areas before the other egui panels
        // claim space, so nothing renders behind the status / nav bars.
        #[cfg(target_os = "android")]
        app.add_systems(
            bevy_egui::EguiPrimaryContextPass,
            crate::android::apply_safe_area_insets
                .before(crate::connection_ui::ConnectionPanelSet)
                .before(crate::topics_view::topics_tree_ui_system),
        );
        app.add_systems(Startup, |mut commands: Commands| {
            let camera = spawn_viewing_rig(&mut commands);
            commands.entity(camera).insert(PrimaryEguiContext);
        });
    }

    app.insert_resource(ClearColor(Color::srgb(0.13, 0.14, 0.17)));
    app.add_plugins(RobotScenePlugin);
    // A spinning indicator while we wait for a robot; despawns on arrival.
    app.add_plugins(crate::loading::LoadingIndicatorPlugin);

    // Zenoh is a build-time-only transport (chosen via --zenoh); it predates
    // the runtime connection switcher and stays a startup path for now.
    #[cfg(feature = "zenoh")]
    if let Some(endpoint) = options.zenoh.clone() {
        app.add_plugins(crate::zenoh::ZenohPlugin {
            endpoints: vec![endpoint],
        });
        app.add_systems(Update, spawn_pending_robot);
        return app;
    }

    // Everything else (demo / rosbridge / DDS) is managed at runtime.
    app.add_plugins(ConnectionPlugin {
        initial: initial_mode(options),
    });

    app
}

/// Send the app to the background when the Android Back button is pressed,
/// instead of exiting.
///
/// Pressing Back used to write [`AppExit`], which tears down winit's event
/// loop. But Android keeps the process alive, and Bevy/winit can't restart a
/// torn-down event loop — so relaunching from the launcher or recents crashed
/// (#36). Backgrounding the task (the same thing the Home button does) routes
/// through the normal `Suspended`/`Resumed` path winit already handles, so
/// reopening resumes the same instance cleanly.
///
/// winit 0.30 delivers Android's `BACK` keycode only as the logical
/// [`Key::BrowserBack`] — it has no physical [`KeyCode`] — so we match on the
/// logical key rather than `ButtonInput<KeyCode>`, which would never fire.
#[cfg(target_os = "android")]
fn android_back_backgrounds(mut keys: MessageReader<bevy::input::keyboard::KeyboardInput>) {
    use bevy::input::ButtonState;
    use bevy::input::keyboard::Key;
    let backed = keys
        .read()
        .any(|k| k.state == ButtonState::Pressed && k.logical_key == Key::BrowserBack);
    if backed {
        move_task_to_back();
    }
}

/// Call `Activity.moveTaskToBack(true)` over JNI, backgrounding the app like
/// the Home button rather than finishing the activity.
///
/// The `JavaVM` and the `Activity` come from [`ndk_context`], populated by the
/// android-activity glue; `jni` makes the one virtual call. Failures are logged
/// and swallowed — at worst Back does nothing, which still beats crashing.
#[cfg(target_os = "android")]
fn move_task_to_back() {
    use jni::objects::{JObject, JValue};

    let ctx = ndk_context::android_context();
    // SAFETY: ndk_context exposes the process-wide JavaVM and Activity set up
    // by android-activity/winit; both are valid for the life of the process.
    let vm = match unsafe { jni::JavaVM::from_raw(ctx.vm().cast()) } {
        Ok(vm) => vm,
        Err(err) => {
            tracing::warn!("Back: JavaVM::from_raw failed: {err}");
            return;
        }
    };
    let mut env = match vm.attach_current_thread() {
        Ok(env) => env,
        Err(err) => {
            tracing::warn!("Back: attach_current_thread failed: {err}");
            return;
        }
    };
    // SAFETY: `context()` is the Activity jobject for the current process.
    let activity = unsafe { JObject::from_raw(ctx.context().cast()) };
    if let Err(err) = env.call_method(&activity, "moveTaskToBack", "(Z)Z", &[JValue::Bool(1)]) {
        tracing::warn!("Back: moveTaskToBack failed: {err}");
    }
}

/// Acquire and hold a Wi-Fi [`MulticastLock`] so native DDS discovery works.
///
/// DDS finds peers via SPDP over UDP multicast, but Android's Wi-Fi driver
/// drops multicast/broadcast packets unless an app holds a
/// `WifiManager.MulticastLock`. Without it, discovery never receives other
/// participants and DDS appears dead even on a good LAN. We acquire one at
/// startup and leak a global ref so it's held for the whole process.
///
/// This only helps on a real LAN — mobile data and many locked-down Wi-Fi
/// networks still block multicast, so DDS on a phone stays unreliable (the
/// connection bar warns about this and points at rosbridge).
#[cfg(all(target_os = "android", feature = "ros2"))]
fn acquire_wifi_multicast_lock() {
    if let Err(err) = try_acquire_wifi_multicast_lock() {
        tracing::warn!(
            "DDS: could not acquire Wi-Fi multicast lock (discovery may not work): {err}"
        );
    }
}

#[cfg(all(target_os = "android", feature = "ros2"))]
fn try_acquire_wifi_multicast_lock() -> jni::errors::Result<()> {
    use jni::objects::{JObject, JValue};

    let ctx = ndk_context::android_context();
    // SAFETY: process-wide JavaVM + Activity from the android-activity glue.
    let vm = unsafe { jni::JavaVM::from_raw(ctx.vm().cast()) }?;
    let mut env = vm.attach_current_thread()?;
    let activity = unsafe { JObject::from_raw(ctx.context().cast()) };

    // WifiManager wifi = (WifiManager) ctx.getSystemService("wifi");
    let service = env.new_string("wifi")?;
    let wifi = env
        .call_method(
            &activity,
            "getSystemService",
            "(Ljava/lang/String;)Ljava/lang/Object;",
            &[JValue::Object(&service)],
        )?
        .l()?;
    if wifi.is_null() {
        tracing::warn!("DDS: WIFI_SERVICE unavailable; skipping multicast lock");
        return Ok(());
    }

    // MulticastLock lock = wifi.createMulticastLock("ros-viz-dds");
    let tag = env.new_string("ros-viz-dds")?;
    let lock = env
        .call_method(
            &wifi,
            "createMulticastLock",
            "(Ljava/lang/String;)Landroid/net/wifi/WifiManager$MulticastLock;",
            &[JValue::Object(&tag)],
        )?
        .l()?;
    env.call_method(&lock, "setReferenceCounted", "(Z)V", &[JValue::Bool(0)])?;
    env.call_method(&lock, "acquire", "()V", &[])?;

    // Keep the MulticastLock object alive for the life of the process so it is
    // never GC'd (which would release the lock): leak a global reference.
    std::mem::forget(env.new_global_ref(&lock)?);
    tracing::info!("DDS: Wi-Fi multicast lock acquired");
    Ok(())
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

/// Spawn the robot scene once a model has been received into [`PendingRobot`].
///
/// Shared by every streaming transport (rosbridge, DDS, zenoh) and the
/// connection manager; a no-op when nothing is pending or a robot is already
/// present.
pub fn spawn_pending_robot(
    mut commands: Commands,
    mut pending: ResMut<PendingRobot>,
    robots: Query<&RobotHandle>,
    options: Res<Options>,
    blobs: Res<MeshBlobs>,
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
    #[cfg(not(target_arch = "wasm32"))]
    resolver.fallback_dirs.extend(std::env::current_dir().ok());
    for spec in &options.package {
        if let Some((name, path)) = spec.split_once('=') {
            resolver = resolver.with_package(name, path);
        } else {
            tracing::warn!("--package expects NAME=PATH, got '{spec}'");
        }
    }
    // Merge any uploaded meshes (web) on top.
    let resolver = blobs.apply(resolver);
    tracing::info!("Spawning robot '{}'", model.name());
    spawn_robot(&mut commands, &mut meshes, &mut materials, model, &resolver);
}

#[cfg(all(not(target_arch = "wasm32"), not(target_os = "android")))]
fn init_tracing() {
    let env_filter = EnvFilter::try_from_default_env().unwrap_or_else(|_| EnvFilter::new("info"));
    let _ = tracing_subscriber::fmt()
        .with_env_filter(env_filter)
        .try_init();
}

#[cfg(any(target_arch = "wasm32", target_os = "android"))]
fn init_tracing() {
    // Bevy's LogPlugin routes tracing to the browser console (wasm) and to
    // logcat (Android); a second subscriber here would just claim the global
    // slot and send logs nowhere visible.
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
