use crate::config::{AppConfig, RenderConfig};
use crate::emulator::{Emulator, EmulatorConfig};
#[cfg(feature = "ros")]
use crate::ros::{self, JointStateMsg, RosConfig};
use crate::urdf::{parse_urdf, UrdfScene};
use bevy::app::App;
use bevy::prelude::*;
use bevy::transform::TransformPlugin;
use bevy::MinimalPlugins;
use image::{Rgba, RgbaImage};
use std::collections::{BTreeMap, HashMap};
use tracing_subscriber::EnvFilter;

// Temporary built-in URDF to keep the pipeline exercised without external assets.
const DEFAULT_URDF_XML: &str = include_str!("../../assets/tests/urdf/box_bot.urdf");

#[cfg(feature = "ros")]
#[derive(Debug, Resource)]
struct RosState {
    handle: ros::RosHandle,
}

#[derive(Debug, Clone, Resource)]
pub struct RobotAssets {
    pub urdf_xml: String,
    pub scene: UrdfScene,
}

#[derive(Debug, Component)]
struct LinkNode;

#[derive(Debug, Component)]
struct JointNode {
    name: String,
    position: f64,
}

/// Initialize tracing and launch the (future) Bevy-based app.
/// Tracing setup mirrors https://docs.rs/tracing-subscriber/latest/tracing_subscriber/fmt/index.html
pub fn run(config: AppConfig) -> anyhow::Result<()> {
    init_tracing();
    tracing::info!(
        domain_id = config.domain_id,
        headless = config.headless,
        output_image = ?config.output_image,
        "Starting ros-viz-rs application"
    );

    // TODO: Replace stub with Bevy app runner when rendering is added.
    let _app = build_app(&config);
    Ok(())
}

/// Build a Bevy `App` configured for headless or windowed rendering.
/// Rendering plugins are intentionally minimal for now to keep CI stable; full stacks will be added later.
pub fn build_app(config: &AppConfig) -> App {
    let mut app = App::new();
    app.insert_resource(config.clone());

    let (urdf_xml, scene) = prepare_urdf_assets();
    let initial_joints = zeroed_joints(&scene);
    let emulator = Emulator::start(
        EmulatorConfig::from_app(config, "ros_viz_robot", urdf_xml.clone()),
        initial_joints,
    );

    app.insert_resource(RobotAssets { urdf_xml, scene });
    app.insert_resource(emulator);

    #[cfg(feature = "ros")]
    {
        if let Some(ros) = maybe_init_ros(config) {
            app.insert_resource(RosState { handle: ros });
        }
    }

    if config.headless {
        app.add_plugins(MinimalPlugins);
    } else {
        // Placeholder: use MinimalPlugins until windowed rendering is wired; DefaultPlugins will come later.
        app.add_plugins(MinimalPlugins);
    }

    app.add_plugins(TransformPlugin);
    app.add_systems(Startup, populate_urdf_scene);
    app.add_systems(Update, sync_joint_transforms);
    app.add_systems(Startup, capture_output_image);

    #[cfg(feature = "ros")]
    {
        app.add_systems(Startup, publish_robot_description);
        app.add_systems(Update, (publish_joint_states, apply_joint_commands));
    }

    app
}

#[cfg(feature = "ros")]
fn maybe_init_ros(config: &AppConfig) -> Option<ros::RosHandle> {
    let cfg = RosConfig::from_app(config);
    match ros::connect(&cfg) {
        Ok(handle) => Some(handle),
        Err(err) => {
            tracing::warn!(?err, "ROS connect failed; continuing without ROS");
            None
        }
    }
}

fn prepare_urdf_assets() -> (String, UrdfScene) {
    let xml = DEFAULT_URDF_XML.trim().to_string();
    let scene = match parse_urdf(&xml) {
        Ok(scene) => scene,
        Err(err) => {
            tracing::warn!(?err, "URDF parsing disabled or failed; using empty scene");
            UrdfScene::empty()
        }
    };
    (xml, scene)
}

fn zeroed_joints(scene: &UrdfScene) -> BTreeMap<String, f64> {
    scene
        .joints
        .iter()
        .map(|name| (name.clone(), 0.0))
        .collect()
}

fn populate_urdf_scene(mut commands: Commands, assets: Res<RobotAssets>) {
    for link in &assets.scene.links {
        commands.spawn((LinkNode, Name::new(format!("link:{link}"))));
    }

    for joint in &assets.scene.joints {
        commands.spawn((
            JointNode {
                name: joint.clone(),
                position: 0.0,
            },
            Transform::default(),
            GlobalTransform::default(),
            Name::new(format!("joint:{joint}")),
        ));
    }
}

fn sync_joint_transforms(
    emulator: Res<Emulator>,
    mut query: Query<(&mut JointNode, &mut Transform)>,
) {
    let snapshot = emulator.joint_state_snapshot();
    let values: HashMap<&str, f64> = snapshot
        .names
        .iter()
        .zip(snapshot.positions.iter())
        .map(|(name, pos)| (name.as_str(), *pos))
        .collect();

    for (mut node, mut transform) in query.iter_mut() {
        if let Some(value) = values.get(node.name.as_str()) {
            node.position = *value;
            transform.translation.x = *value as f32;
        }
    }
}

#[cfg(feature = "ros")]
fn publish_robot_description(assets: Res<RobotAssets>, ros: Option<Res<RosState>>) {
    if let Some(ros) = ros.as_ref() {
        if let Err(err) = ros.handle.publish_robot_description(&assets.urdf_xml) {
            tracing::warn!(?err, "Failed to publish robot_description");
        }
    }
}

fn capture_output_image(
    config: Res<AppConfig>,
    assets: Res<RobotAssets>,
    emulator: Res<Emulator>,
) {
    if let Some(path) = &config.output_image {
        let img = render_stub_image(&assets, &emulator, &config.render);
        if let Err(err) = img.save(path) {
            tracing::warn!(?err, "Failed to save output image");
        } else {
            tracing::info!(?path, "Saved output image");
        }
    }
}

fn render_stub_image(
    assets: &RobotAssets,
    emulator: &Emulator,
    render: &RenderConfig,
) -> RgbaImage {
    let width = render.width.max(1);
    let height = render.height.max(1);
    let bg = Rgba([20, 20, 28, 255]);
    let mut img = RgbaImage::from_pixel(width, height, bg);

    let snapshot = emulator.joint_state_snapshot();
    let joint_count = snapshot.names.len().max(1);
    let bar_width = (width / joint_count as u32).max(1);

    for (idx, value) in snapshot.positions.iter().enumerate() {
        let x_start = (idx as u32).saturating_mul(bar_width).min(width - 1);
        let intensity = ((*value * 50.0).abs() as u8).saturating_add(40);
        let color = Rgba([intensity, 180, 220, 255]);
        for x in x_start..width.min(x_start + bar_width) {
            for y in 0..height {
                img.put_pixel(x, y, color);
            }
        }
    }

    // Stamp link count on the first pixel to keep deterministic variation with scene changes.
    let link_marker = assets.scene.link_count().min(255) as u8;
    img.put_pixel(0, 0, Rgba([bg[0], link_marker, bg[2], 255]));
    img
}

#[cfg(feature = "ros")]
fn publish_joint_states(emulator: Res<Emulator>, ros: Option<Res<RosState>>) {
    if let Some(ros) = ros.as_ref() {
        let snapshot = emulator.joint_state_snapshot();
        let msg = JointStateMsg {
            names: snapshot.names,
            positions: snapshot.positions,
        };
        if let Err(err) = ros.handle.publish_joint_states(msg) {
            tracing::warn!(?err, "Failed to publish joint_states");
        }
    }
}

#[cfg(feature = "ros")]
fn apply_joint_commands(emulator: Res<Emulator>, ros: Option<Res<RosState>>) {
    if let Some(ros) = ros.as_ref() {
        match ros.handle.try_take_joint_commands() {
            Ok(Some(cmds)) => {
                let updates = cmds
                    .names
                    .into_iter()
                    .zip(cmds.positions.into_iter())
                    .collect::<Vec<_>>();
                emulator.apply_joint_commands(updates);
            }
            Ok(None) => {}
            Err(err) => tracing::warn!(?err, "Failed to read joint_commands"),
        }
    }
}

fn init_tracing() {
    // Allow overrides with RUST_LOG; default to info for early scaffolding.
    let env_filter = EnvFilter::try_from_default_env().unwrap_or_else(|_| EnvFilter::new("info"));
    let _ = tracing_subscriber::fmt()
        .with_env_filter(env_filter)
        .try_init();
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn builds_headless_app() {
        let mut cfg = AppConfig::new(0);
        cfg.headless = true;
        let app = build_app(&cfg);
        let stored = app.world().get_resource::<AppConfig>().cloned();
        assert_eq!(stored, Some(cfg));
        assert!(app.world().contains_resource::<RobotAssets>());
        assert!(app.world().contains_resource::<Emulator>());
    }

    #[test]
    fn builds_windowed_app_placeholder() {
        let cfg = AppConfig::new(0);
        let app = build_app(&cfg);
        let stored = app.world().get_resource::<AppConfig>().cloned();
        assert_eq!(stored, Some(cfg));
        assert!(app.world().contains_resource::<RobotAssets>());
        assert!(app.world().contains_resource::<Emulator>());
    }

    #[test]
    fn startup_spawns_scene_entities() {
        let cfg = AppConfig::new(0);
        let mut app = build_app(&cfg);
        app.update();

        let assets = app
            .world()
            .get_resource::<RobotAssets>()
            .cloned()
            .expect("assets present");

        let world = app.world_mut();
        let link_count = world.query::<&LinkNode>().iter(&*world).count();
        let joint_count = world
            .query::<(&JointNode, &Transform)>()
            .iter(&*world)
            .count();

        assert_eq!(link_count, assets.scene.link_count());
        assert_eq!(joint_count, assets.scene.joint_count());
    }

    #[test]
    fn stub_image_matches_resolution_and_marks_joint() {
        let scene = UrdfScene {
            joints: vec!["j1".to_string()],
            links: vec!["l1".to_string()],
        };
        let assets = RobotAssets {
            urdf_xml: "<robot></robot>".to_string(),
            scene,
        };
        let emulator = Emulator::start(
            EmulatorConfig::from_app(&AppConfig::new(0), "dummy", "<robot></robot>"),
            BTreeMap::new(),
        );

        emulator.apply_joint_commands(vec![("j1".to_string(), 1.0)]);
        let img = render_stub_image(&assets, &emulator, &RenderConfig::new(64, 48));
        assert_eq!(img.width(), 64);
        assert_eq!(img.height(), 48);
        let top_left = img.get_pixel(0, 0);
        let sample_bar = img.get_pixel(1, 10);
        assert_ne!(top_left, sample_bar, "joint bar should differ from marker");
    }

    #[cfg(feature = "urdf")]
    #[test]
    fn joint_commands_update_transforms() {
        let cfg = AppConfig::new(0);
        let mut app = build_app(&cfg);
        app.update(); // run startup systems

        {
            let emulator = app
                .world()
                .get_resource::<Emulator>()
                .cloned()
                .expect("emulator present");
            emulator.apply_joint_commands(vec![("shoulder".to_string(), 1.25)]);
        }

        app.update(); // run sync system

        let mut world = app.world_mut();
        let mut query = world.query::<(&JointNode, &Transform)>();
        let mut found = false;
        for (node, transform) in query.iter(&world) {
            if node.name == "shoulder" {
                found = true;
                assert!((transform.translation.x - 1.25).abs() < 1e-6);
            }
        }
        assert!(found);
    }
}
