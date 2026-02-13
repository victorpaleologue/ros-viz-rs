//! Display a tree of ROS 2 topics discovered via DDS.
//!
//! # Usage
//!
//! ```bash
//! cargo run --example view_topics --features ros,ui
//! ```

use bevy::prelude::*;
use bevy_egui::EguiPlugin;
use ros_viz_rs::ros_systems::{RosContext, RosDiscoveryPlugin};
use ros_viz_rs::topics_view::{TopicsPanelMode, TopicsTreePlugin};

fn main() {
    let ctx = ros2_client::Context::new().expect("failed to create ROS 2 context");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "Topics Tree".to_string(),
                resolution: (400.0, 600.0).into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(EguiPlugin)
        .add_plugins(TopicsTreePlugin)
        .add_plugins(RosDiscoveryPlugin)
        .insert_resource(RosContext(ctx))
        .insert_resource(TopicsPanelMode::Central)
        .add_systems(Startup, setup)
        .run();
}

fn setup(mut commands: Commands) {
    commands.spawn(Camera2d);
}
