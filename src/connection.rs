//! Runtime connection management.
//!
//! Lets the viewer switch its data source live — the embedded [`demo`], a
//! [`rosbridge`] WebSocket, or native [`DDS`] — without restarting. Each
//! transport registers its systems once (gated on a session resource, so they
//! only run while connected) and exposes `connect` / `disconnect`; this module
//! reconciles a requested [`ConnectionMode`] by tearing down the current
//! source and starting the chosen one.
//!
//! [`demo`]: crate::demo
//! [`rosbridge`]: crate::rosbridge
//! [`DDS`]: crate::ros_plugin

use bevy::prelude::*;

use crate::options::Options;
use crate::scene::{JointPositions, PendingRobot, RobotHandle};
use crate::topics::TopicInfo;

/// A data source the viewer can show.
///
/// All variants exist on every build; which ones are actually offered (and
/// can be connected) depends on the compiled-in transport features — see
/// [`ConnectionPlugin`] and the connection UI.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ConnectionMode {
    /// Embedded NAO demo, no transport.
    Demo,
    /// rosbridge WebSocket at this URL (e.g. `ws://host:9090`).
    Rosbridge(String),
    /// Native DDS on this ROS domain id.
    Dds(u16),
}

impl ConnectionMode {
    /// Short human label for status text.
    pub fn label(&self) -> &'static str {
        match self {
            ConnectionMode::Demo => "demo",
            ConnectionMode::Rosbridge(_) => "rosbridge",
            ConnectionMode::Dds(_) => "DDS",
        }
    }
}

/// The data source currently live (`None` until the first connection applies).
#[derive(Resource, Default)]
pub struct ActiveConnection(pub Option<ConnectionMode>);

/// A requested data source, consumed by the reconciler. The UI (or startup)
/// sets this; it is applied on the next frame.
#[derive(Resource, Default)]
pub struct PendingConnection(pub Option<ConnectionMode>);

impl PendingConnection {
    /// Request a switch to `mode` on the next frame.
    pub fn request(&mut self, mode: ConnectionMode) {
        self.0 = Some(mode);
    }
}

/// Wires up runtime connection switching: registers every compiled-in
/// transport's systems (gated on their session resource), the shared
/// robot-spawn system, and the reconciler that applies [`PendingConnection`].
pub struct ConnectionPlugin {
    /// The data source to connect on startup (from CLI options / platform).
    pub initial: ConnectionMode,
}

impl Plugin for ConnectionPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<ActiveConnection>();
        app.insert_resource(PendingConnection(Some(self.initial.clone())));

        crate::demo::register_systems(app);
        #[cfg(feature = "rosbridge")]
        crate::rosbridge::register_systems(app);
        #[cfg(feature = "ros2")]
        {
            crate::ros_plugin::register_systems(app);
            crate::ros_plugin::register_robot_feed(app);
            crate::topics_io::register_systems(app);
        }

        // Shared across transports: spawn a received robot. Harmless when
        // nothing is pending.
        app.add_systems(Update, crate::app::spawn_pending_robot);

        // Reconcile requested -> active. Exclusive because it inserts/removes
        // (non-send) session resources and despawns entities.
        app.add_systems(Update, apply_pending_connection);
    }
}

/// Pick the startup [`ConnectionMode`] from CLI options, honoring the
/// transports compiled into this build.
pub fn initial_mode(options: &Options) -> ConnectionMode {
    #[cfg(feature = "rosbridge")]
    if let Some(url) = &options.rosbridge {
        return ConnectionMode::Rosbridge(url.clone());
    }
    if options.demo {
        return ConnectionMode::Demo;
    }
    #[cfg(feature = "ros2")]
    {
        ConnectionMode::Dds(options.domain)
    }
    // Without native DDS and no rosbridge URL, the demo is the only source.
    #[cfg(not(feature = "ros2"))]
    ConnectionMode::Demo
}

/// Apply a pending connection request: tear down the current source and start
/// the requested one.
fn apply_pending_connection(world: &mut World) {
    let Some(target) = world.resource_mut::<PendingConnection>().0.take() else {
        return;
    };
    // Already on the requested source: nothing to do (e.g. a redundant click).
    if world.resource::<ActiveConnection>().0.as_ref() == Some(&target) {
        return;
    }

    teardown(world);

    match &target {
        ConnectionMode::Demo => crate::demo::connect(world),
        ConnectionMode::Rosbridge(url) => {
            #[cfg(feature = "rosbridge")]
            if let Err(e) = crate::rosbridge::connect(world, url) {
                tracing::error!("{e}");
            }
            #[cfg(not(feature = "rosbridge"))]
            {
                let _ = url;
                tracing::error!("rosbridge support is not built into this binary");
            }
        }
        ConnectionMode::Dds(domain) => {
            #[cfg(feature = "ros2")]
            if let Err(e) = crate::ros_plugin::connect(world, *domain, "ros_viz_rs") {
                tracing::error!("{e}");
            }
            #[cfg(not(feature = "ros2"))]
            {
                let _ = domain;
                tracing::error!("native DDS support is not built into this binary");
            }
        }
    }

    tracing::info!("connection: switched to {}", target.label());
    world.resource_mut::<ActiveConnection>().0 = Some(target);
}

/// Stop every transport and clear the current robot and topics, returning the
/// world to a clean slate before the next connection starts.
fn teardown(world: &mut World) {
    // Each transport's disconnect is a no-op when it isn't the active one.
    crate::demo::disconnect(world);
    #[cfg(feature = "rosbridge")]
    crate::rosbridge::disconnect(world);
    #[cfg(feature = "ros2")]
    crate::ros_plugin::disconnect(world);

    // Despawn the current robot (and its link/visual children) and clear its
    // joint state so a stale pose can't bleed into the next source.
    let robots: Vec<Entity> = world
        .query_filtered::<Entity, With<RobotHandle>>()
        .iter(world)
        .collect();
    for entity in robots {
        world.entity_mut(entity).despawn();
    }
    world.resource_mut::<PendingRobot>().0 = None;
    world.resource_mut::<JointPositions>().positions.clear();

    // Drop discovered topics so the panel reflects the new source only.
    let topics: Vec<Entity> = world
        .query_filtered::<Entity, With<TopicInfo>>()
        .iter(world)
        .collect();
    for entity in topics {
        world.entity_mut(entity).despawn();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn initial_mode_prefers_demo_flag() {
        let options = Options {
            demo: true,
            ..Options::default()
        };
        assert_eq!(initial_mode(&options), ConnectionMode::Demo);
    }

    #[cfg(feature = "rosbridge")]
    #[test]
    fn initial_mode_uses_rosbridge_url() {
        let options = Options {
            rosbridge: Some("ws://host:9090".into()),
            ..Options::default()
        };
        assert_eq!(
            initial_mode(&options),
            ConnectionMode::Rosbridge("ws://host:9090".into())
        );
    }

    #[cfg(feature = "ros2")]
    #[test]
    fn initial_mode_defaults_to_dds_domain() {
        let options = Options {
            domain: 7,
            ..Options::default()
        };
        assert_eq!(initial_mode(&options), ConnectionMode::Dds(7));
    }
}
