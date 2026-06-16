//! A generic ROS 2 robot visualizer built on [Bevy](https://bevy.org) and
//! [egui](https://github.com/emilk/egui).
//!
//! ros-viz-rs subscribes to `/robot_description`, renders the URDF with its
//! real geometry (primitives and STL/OBJ/COLLADA meshes), animates joints
//! from `/joint_states` through proper forward kinematics, and shows every
//! topic on the graph in an editable inspector. It speaks DDS directly (no
//! rclcpp, no ROS installation), so it runs anywhere Rust runs — macOS,
//! Windows, Linux — and compiles to WebAssembly, where it connects through
//! [rosbridge](https://github.com/RobotWebTools/rosbridge_suite).
//!
//! # Architecture
//!
//! The crate is a library; the `ros-viz-rs` binary, the examples and the
//! browser build are thin shells over it.
//!
//! - [`robot`] — the model: full-fidelity URDF ([`urdf_rs::Robot`]) plus a
//!   [`k`] kinematic chain. Joint positions in, per-link world transforms
//!   out. No Bevy types; usable headless.
//! - [`scene`] — Bevy integration: spawns one entity per link with its
//!   visuals, keeps transforms in sync with [`scene::JointPositions`] via
//!   FK, auto-frames cameras. ROS is Z-up, Bevy Y-up; the robot root
//!   entity carries the conversion.
//! - [`topics`] — the transport seam: every connection backend discovers
//!   topics into [`topics::TopicInfo`] entities and attaches type-erased
//!   subscription/publisher handles plus reflected [`ron::Value`] trees
//!   (lossless: distinct integer widths, non-finite floats). Consumers
//!   (the UI, your code) never see transport types.
//! - [`messages`] + [`ros_msgs`] — 50 standard ROS message types as plain
//!   serde structs and a registry mapping type names to reflection (and,
//!   with the `ros2` feature, DDS I/O factories). Adding a message type is
//!   one struct + one registration line.
//! - Connection backends: `ros_plugin`/`topics_io` (DDS via
//!   [ros2-client](https://crates.io/crates/ros2-client), feature `ros2`,
//!   default) and `rosbridge` (JSON over WebSocket via ewebsock, feature
//!   `rosbridge`, works on wasm).
//! - [`snapshot`] + [`vision`] — headless verification: real GPU rendering
//!   to PNG without any window, and pure-Rust image comparison (RMSE,
//!   silhouette analysis, reference blessing). This is how `cargo test`
//!   proves a robot actually renders.
//! - [`demo`] — the embedded NAO waving hello with zero transport, used by
//!   `--demo`, tests and the web demo; [`emulator`] — a fake robot
//!   publishing over real DDS for end-to-end tests.
//!
//! # Quick taste
//!
//! Render a URDF to pixels without ROS, a window, or a GPU you can see:
//!
//! ```no_run
//! use std::sync::Arc;
//! use bevy::prelude::*;
//! use ros_viz_rs::robot::{RobotModel, mesh::MeshResolver};
//! use ros_viz_rs::scene::{RobotScenePlugin, spawn_lights, spawn_robot};
//! use ros_viz_rs::snapshot::{SnapshotPlugin, capture, save_png};
//!
//! let model = Arc::new(RobotModel::from_urdf_file("robot.urdf")?);
//! let mut app = App::new();
//! app.add_plugins(SnapshotPlugin::default());
//! app.add_plugins(RobotScenePlugin);
//!
//! let world = app.world_mut();
//! let mut state: bevy::ecs::system::SystemState<(
//!     Commands,
//!     ResMut<Assets<Mesh>>,
//!     ResMut<Assets<StandardMaterial>>,
//! )> = bevy::ecs::system::SystemState::new(world);
//! let (mut commands, mut meshes, mut materials) = state.get_mut(world);
//! spawn_lights(&mut commands);
//! spawn_robot(&mut commands, &mut meshes, &mut materials, model,
//!             &MeshResolver::default());
//! state.apply(world);
//!
//! let image = capture(&mut app, 12)?;
//! save_png(&image, std::path::Path::new("robot.png"))?;
//! # Ok::<(), anyhow::Error>(())
//! ```
//!
//! # Feature flags
//!
//! | feature | default | effect |
//! |---|---|---|
//! | `ros2` | yes | DDS transport (ros2-client/rustdds); native only |
//! | `rosbridge` | yes | JSON/WebSocket transport; native and wasm |
//!
//! The browser build uses `--no-default-features --features rosbridge`.

#[cfg(target_os = "android")]
pub mod android;
pub mod app;
pub mod camera;
pub mod demo;
pub mod diagnostics;
#[cfg(feature = "ros2")]
pub mod emulator;
pub mod loading;
pub mod messages;
pub mod options;
pub mod robot;
pub mod ros_msgs;
#[cfg(feature = "ros2")]
pub mod ros_plugin;
#[cfg(feature = "rosbridge")]
pub mod rosbridge;
pub mod scene;
#[cfg(not(target_arch = "wasm32"))]
pub mod snapshot;
pub mod topics;
#[cfg(feature = "ros2")]
pub mod topics_io;
pub mod topics_view;
pub mod vision;
#[cfg(target_arch = "wasm32")]
pub mod web;
#[cfg(feature = "zenoh")]
pub mod zenoh;
