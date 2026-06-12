//! True headless offscreen rendering: render a scene to a texture without
//! opening any window, then read the pixels back to the CPU as PNG-able bytes.
//!
//! Works from regular `#[test]` functions (off the main thread) because the
//! winit event loop is never started: `WinitPlugin` is disabled and the app is
//! driven by [`ScheduleRunnerPlugin`] / manual [`App::update`] calls.
//!
//! References:
//! - <https://github.com/bevyengine/bevy/blob/v0.18.0/examples/app/headless_renderer.rs>
//! - <https://docs.rs/bevy/0.18.0/bevy/render/gpu_readback/index.html>
//! - <https://github.com/bevyengine/bevy/blob/v0.18.0/examples/shader/gpu_readback.rs>

use std::path::Path;
use std::time::Duration;

use anyhow::{Context, bail, ensure};
use bevy::app::{PluginsState, ScheduleRunnerPlugin};
use bevy::camera::RenderTarget;
use bevy::core_pipeline::tonemapping::{DebandDither, Tonemapping};
use bevy::image::TextureFormatPixelInfo;
use bevy::prelude::*;
use bevy::render::RenderPlugin;
use bevy::render::gpu_readback::{Readback, ReadbackComplete};
use bevy::render::render_resource::{TextureFormat, TextureUsages};
use bevy::render::renderer::RenderDevice;
use bevy::window::ExitCondition;
use bevy::winit::WinitPlugin;
use image::RgbaImage;

/// Texture format of the snapshot render target.
///
/// `Rgba8UnormSrgb` matches `TextureFormat::bevy_default()` on desktop and
/// keeps the readback buffer in straightforward RGBA byte order.
const SNAPSHOT_FORMAT: TextureFormat = TextureFormat::Rgba8UnormSrgb;

/// Maximum number of extra frames [`capture`] is willing to run while waiting
/// for the asynchronous GPU readback to deliver a frame.
const MAX_READBACK_FRAMES: u32 = 1000;

/// Marker component on the camera spawned by [`spawn_snapshot_camera`], so
/// callers can reposition it (e.g. `Query<&mut Transform, With<SnapshotCamera>>`).
#[derive(Component)]
pub struct SnapshotCamera;

/// Handle and dimensions of the offscreen render target currently captured.
///
/// Inserted by [`spawn_snapshot_camera`]; read by [`capture`].
#[derive(Resource, Clone)]
pub struct SnapshotTarget {
    /// The image asset the snapshot camera renders into.
    pub image: Handle<Image>,
    /// Width of the render target in pixels.
    pub width: u32,
    /// Height of the render target in pixels.
    pub height: u32,
}

/// Latest raw (row-padded) frame delivered by the GPU readback.
#[derive(Resource, Default)]
struct SnapshotFrame(Option<Vec<u8>>);

/// Render target size requested through [`SnapshotPlugin`].
#[derive(Resource, Clone, Copy)]
struct SnapshotSize {
    width: u32,
    height: u32,
}

/// Configures a completely windowless Bevy app that renders to an offscreen
/// texture and reads it back to the CPU.
///
/// Adds `DefaultPlugins` with `primary_window: None`, the `WinitPlugin`
/// disabled (so no event loop, no Dock icon, and it runs off the main thread)
/// and a [`ScheduleRunnerPlugin`] instead. At `Startup` it spawns a snapshot
/// camera via [`spawn_snapshot_camera`]; add your own entities in `Startup`
/// systems, reposition the camera through the [`SnapshotCamera`] marker, then
/// call [`capture`].
pub struct SnapshotPlugin {
    /// Width of the offscreen render target in pixels.
    pub width: u32,
    /// Height of the offscreen render target in pixels.
    pub height: u32,
}

impl Default for SnapshotPlugin {
    fn default() -> Self {
        Self {
            width: 512,
            height: 512,
        }
    }
}

impl Plugin for SnapshotPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(
            DefaultPlugins
                .set(WindowPlugin {
                    // No window is ever created.
                    primary_window: None,
                    // Don't exit just because there is no window.
                    exit_condition: ExitCondition::DontExit,
                    ..default()
                })
                .set(RenderPlugin {
                    // Makes the first rendered frames complete on platforms
                    // with async pipeline compilation (e.g. Linux/Vulkan).
                    synchronous_pipeline_compilation: true,
                    ..default()
                })
                // The winit event loop demands the main thread and a display
                // server; without it, windowless apps run from any thread.
                .disable::<WinitPlugin>(),
        )
        .add_plugins(ScheduleRunnerPlugin::run_loop(Duration::from_secs_f64(
            1.0 / 60.0,
        )))
        .init_resource::<SnapshotFrame>()
        .insert_resource(SnapshotSize {
            width: self.width,
            height: self.height,
        })
        .add_systems(Startup, setup_snapshot_camera);
    }
}

/// Spawns the default snapshot camera at `Startup` using the size configured
/// in [`SnapshotPlugin`].
fn setup_snapshot_camera(
    mut commands: Commands,
    mut images: ResMut<Assets<Image>>,
    size: Res<SnapshotSize>,
) {
    spawn_snapshot_camera(&mut commands, &mut images, size.width, size.height);
}

/// Creates a `width` x `height` render-target image, spawns a [`Camera3d`]
/// rendering into it, and registers a GPU [`Readback`] that copies every
/// rendered frame back to the CPU for [`capture`].
///
/// Returns the handle of the target image. The camera carries the
/// [`SnapshotCamera`] marker and spawns at the default transform (origin,
/// looking towards -Z); reposition it as needed.
///
/// [`SnapshotPlugin`] already calls this once at `Startup`; only call it
/// yourself for custom setups, and note that [`capture`] returns frames from
/// whichever readback completed last.
pub fn spawn_snapshot_camera(
    commands: &mut Commands,
    images: &mut Assets<Image>,
    width: u32,
    height: u32,
) -> Handle<Image> {
    // TEXTURE_BINDING | COPY_DST | RENDER_ATTACHMENT are set by
    // `new_target_texture`; COPY_SRC is required for the GPU -> CPU readback.
    let mut target = Image::new_target_texture(width, height, SNAPSHOT_FORMAT, None);
    target.texture_descriptor.usage |= TextureUsages::COPY_SRC;
    let handle = images.add(target);

    // Reads the texture back every frame and stores the latest copy.
    commands.spawn(Readback::texture(handle.clone())).observe(
        |event: On<ReadbackComplete>, mut frame: ResMut<SnapshotFrame>| {
            frame.0 = Some(event.data.clone());
        },
    );

    commands.spawn((
        Camera3d::default(),
        // Since Bevy 0.17 the render target is its own component, no longer a
        // field of `Camera`.
        RenderTarget::Image(handle.clone().into()),
        // Keep pixel values predictable for assertions on raw colors.
        Tonemapping::None,
        DebandDither::Disabled,
        Msaa::Off,
        SnapshotCamera,
    ));

    commands.insert_resource(SnapshotTarget {
        image: handle.clone(),
        width,
        height,
    });

    handle
}

/// Runs the app for `settle_frames` updates, then keeps updating until the
/// GPU readback delivers a frame rendered after the settling period, and
/// converts it to an [`RgbaImage`].
///
/// Finishes deferred plugin setup (the async wgpu adapter request) on first
/// use, so it can be called right after building the [`App`].
pub fn capture(app: &mut App, settle_frames: u32) -> anyhow::Result<RgbaImage> {
    ensure_ready(app);

    for _ in 0..settle_frames {
        app.update();
    }

    // Discard any frame captured while settling, then wait for a fresh one.
    app.world_mut().resource_mut::<SnapshotFrame>().0 = None;
    for _ in 0..MAX_READBACK_FRAMES {
        app.update();
        if let Some(data) = app.world_mut().resource_mut::<SnapshotFrame>().0.take() {
            let target = app
                .world()
                .get_resource::<SnapshotTarget>()
                .context("no SnapshotTarget resource: was a snapshot camera spawned?")?
                .clone();
            return frame_to_image(&data, target.width, target.height);
        }
    }
    bail!("GPU readback did not deliver a frame within {MAX_READBACK_FRAMES} updates");
}

/// Finish deferred plugin setup (the async wgpu adapter request) so the app
/// can be driven by manual [`App::update`] calls.
///
/// Equivalent of the `ScheduleRunnerPlugin` runner preamble; required before
/// updating a windowless app outside `App::run` (e.g. waiting for ROS data
/// before [`capture`]). Idempotent.
pub fn ensure_ready(app: &mut App) {
    if app.plugins_state() != PluginsState::Cleaned {
        while app.plugins_state() == PluginsState::Adding {
            bevy::tasks::tick_global_task_pools_on_main_thread();
        }
        app.finish();
        app.cleanup();
    }
}

/// Saves a captured image as a PNG file.
pub fn save_png(img: &RgbaImage, path: &Path) -> anyhow::Result<()> {
    img.save_with_format(path, image::ImageFormat::Png)
        .with_context(|| format!("failed to save PNG to {}", path.display()))
}

/// Converts a raw readback buffer into an [`RgbaImage`], dropping the per-row
/// padding wgpu requires for texture-to-buffer copies (rows are aligned to
/// `wgpu::COPY_BYTES_PER_ROW_ALIGNMENT`, i.e. 256 bytes).
fn frame_to_image(data: &[u8], width: u32, height: u32) -> anyhow::Result<RgbaImage> {
    let pixel_size = SNAPSHOT_FORMAT
        .pixel_size()
        .map_err(|e| anyhow::anyhow!("cannot get pixel size of {SNAPSHOT_FORMAT:?}: {e:?}"))?;
    let unpadded_row = width as usize * pixel_size;
    let padded_row = RenderDevice::align_copy_bytes_per_row(unpadded_row);
    ensure!(
        data.len() == padded_row * height as usize,
        "unexpected readback size: got {} bytes, expected {} ({}x{}, {} bytes per padded row)",
        data.len(),
        padded_row * height as usize,
        width,
        height,
        padded_row,
    );

    let mut pixels = Vec::with_capacity(unpadded_row * height as usize);
    for row in data.chunks_exact(padded_row) {
        pixels.extend_from_slice(&row[..unpadded_row]);
    }
    RgbaImage::from_raw(width, height, pixels)
        .context("readback buffer does not match image dimensions")
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Mutex;

    /// Serializes GPU-backed tests: each one creates its own wgpu device, and
    /// running them concurrently in one process is needlessly heavy.
    static GPU_TEST_LOCK: Mutex<()> = Mutex::new(());

    /// 321 px wide so a row is 1284 bytes, forcing 256-byte row padding
    /// (1536 bytes) and exercising the stride handling in `frame_to_image`.
    const WIDTH: u32 = 321;
    const HEIGHT: u32 = 200;

    fn headless_app() -> App {
        let mut app = App::new();
        app.add_plugins(SnapshotPlugin {
            width: WIDTH,
            height: HEIGHT,
        });
        app
    }

    fn spawn_red_cuboid(
        mut commands: Commands,
        mut meshes: ResMut<Assets<Mesh>>,
        mut materials: ResMut<Assets<StandardMaterial>>,
    ) {
        commands.spawn((
            Mesh3d(meshes.add(Cuboid::new(1.0, 1.0, 1.0))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgb(1.0, 0.0, 0.0),
                unlit: true,
                ..default()
            })),
            // In front of the camera, which looks towards -Z by default.
            Transform::from_xyz(0.0, 0.0, -3.0),
        ));
    }

    #[test]
    fn snapshot_clear_color_fills_frame() {
        let _lock = GPU_TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());

        let mut app = headless_app();
        app.insert_resource(ClearColor(Color::srgb(0.0, 0.0, 1.0)));

        let img = capture(&mut app, 5).expect("headless capture failed");
        assert_eq!(img.dimensions(), (WIDTH, HEIGHT));
        for (x, y, pixel) in img.enumerate_pixels() {
            let [r, g, b, a] = pixel.0;
            assert!(
                r < 60 && g < 60 && b > 200 && a == 255,
                "pixel ({x}, {y}) is not the blue clear color: {:?}",
                pixel.0,
            );
        }
    }

    #[test]
    fn snapshot_red_cuboid_in_front_of_camera() {
        let _lock = GPU_TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());

        let mut app = headless_app();
        app.insert_resource(ClearColor(Color::srgb(0.0, 0.0, 1.0)));
        app.add_systems(Startup, spawn_red_cuboid);

        let img = capture(&mut app, 10).expect("headless capture failed");

        let [r, g, b, _] = img.get_pixel(WIDTH / 2, HEIGHT / 2).0;
        assert!(
            r > 200 && g < 60 && b < 60,
            "center pixel is not red: ({r}, {g}, {b})",
        );

        let [r, g, b, _] = img.get_pixel(5, 5).0;
        assert!(
            b > 200 && r < 60 && g < 60,
            "corner pixel is not the blue background: ({r}, {g}, {b})",
        );

        // Round-trip through `save_png` to cover the PNG path as well.
        let dir = tempfile::tempdir().expect("failed to create temp dir");
        let path = dir.path().join("snapshot.png");
        save_png(&img, &path).expect("failed to save PNG");
        let bytes = std::fs::metadata(&path).expect("PNG file missing").len();
        assert!(bytes > 0, "PNG file is empty");
    }
}
