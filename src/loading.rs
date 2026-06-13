//! A loading/connection indicator shown until a robot appears.
//!
//! Rendered with Bevy meshes (not egui), so it works identically on native
//! and on the web — where egui cannot render on WebGL2 (see issue #26). A
//! ring of glowing spheres spins at the origin while the app waits for
//! `/robot_description`; it despawns the moment a [`RobotHandle`] is spawned.
//!
//! The web page shows its own HTML spinner during the earlier
//! wasm-download/compile phase (see `web/index.html`); this indicator takes
//! over once the Bevy app is running but no robot has arrived yet.

use std::f32::consts::TAU;

use bevy::prelude::*;

use crate::scene::RobotHandle;

/// Marks the loading-indicator hierarchy so it can be despawned wholesale.
#[derive(Component)]
struct LoadingIndicator;

/// Number of spheres in the spinning ring.
const SPHERE_COUNT: usize = 8;
/// Radius of the ring, in metres.
const RING_RADIUS: f32 = 0.35;
/// Revolutions per second.
const SPIN_HZ: f32 = 0.5;

/// Spawns a spinning loading indicator and removes it once a robot spawns.
pub struct LoadingIndicatorPlugin;

impl Plugin for LoadingIndicatorPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, spawn_indicator);
        app.add_systems(Update, (spin_indicator, despawn_when_robot_ready));
    }
}

fn spawn_indicator(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Emissive so the ring glows regardless of scene lighting.
    let material = materials.add(StandardMaterial {
        base_color: Color::srgb(0.1, 0.4, 0.8),
        emissive: LinearRgba::rgb(0.15, 0.55, 1.2),
        ..default()
    });
    let sphere = meshes.add(Sphere::new(0.05));

    let parent = commands
        .spawn((
            LoadingIndicator,
            Transform::default(),
            Visibility::default(),
            Name::new("loading-indicator"),
        ))
        .id();

    for i in 0..SPHERE_COUNT {
        let angle = i as f32 / SPHERE_COUNT as f32 * TAU;
        // Scale spheres down around the ring so the leading edge reads as a
        // comet head — a clear sense of motion once it spins.
        let scale = 0.4 + 0.6 * (i as f32 / SPHERE_COUNT as f32);
        commands.spawn((
            Mesh3d(sphere.clone()),
            MeshMaterial3d(material.clone()),
            Transform::from_xyz(angle.cos() * RING_RADIUS, 0.0, angle.sin() * RING_RADIUS)
                .with_scale(Vec3::splat(scale)),
            ChildOf(parent),
        ));
    }
}

fn spin_indicator(time: Res<Time>, mut query: Query<&mut Transform, With<LoadingIndicator>>) {
    for mut transform in query.iter_mut() {
        transform.rotate_y(SPIN_HZ * TAU * time.delta_secs());
    }
}

fn despawn_when_robot_ready(
    mut commands: Commands,
    new_robots: Query<(), Added<RobotHandle>>,
    indicators: Query<Entity, With<LoadingIndicator>>,
) {
    if new_robots.is_empty() {
        return;
    }
    for entity in indicators.iter() {
        commands.entity(entity).despawn();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::robot::RobotModel;
    use std::sync::Arc;

    fn test_app() -> App {
        let mut app = App::new();
        app.add_plugins(MinimalPlugins);
        app.add_plugins(AssetPlugin::default());
        app.init_asset::<Mesh>();
        app.init_asset::<StandardMaterial>();
        app.add_plugins(LoadingIndicatorPlugin);
        app
    }

    #[test]
    fn indicator_spawns_then_despawns_when_robot_ready() {
        let mut app = test_app();
        app.update();
        let count = app
            .world_mut()
            .query_filtered::<(), With<LoadingIndicator>>()
            .iter(app.world())
            .count();
        assert_eq!(count, 1, "one loading-indicator root should spawn");

        // A robot appears.
        let model = Arc::new(
            RobotModel::from_urdf_str("<robot name='t'><link name='base'/></robot>")
                .expect("parses"),
        );
        app.world_mut().spawn((
            RobotHandle(model),
            Transform::default(),
            Visibility::default(),
        ));
        app.update();

        let count = app
            .world_mut()
            .query_filtered::<(), With<LoadingIndicator>>()
            .iter(app.world())
            .count();
        assert_eq!(count, 0, "indicator must despawn once a robot is present");
    }
}
