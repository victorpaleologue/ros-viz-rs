//! Display a tree of ROS 2 topics discovered via DDS.
//!
//! # Usage
//!
//! ```bash
//! cargo run --example view_topics --features ros,ui
//! ```

use bevy::prelude::*;
use bevy_egui::EguiPlugin;
use ros_viz_rs::ros_plugin::RosPlugin;
use ros_viz_rs::topics_io::TopicIOPlugin;
use ros_viz_rs::topics_view::{TopicsPanelMode, TopicsTreePlugin};

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "Topics Tree".to_string(),
                resolution: (400, 600).into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(EguiPlugin { ..default() })
        .add_plugins(TopicsTreePlugin {
            panel_mode: TopicsPanelMode::Central,
        })
        .add_plugins(RosPlugin::new(0u16, "topics_viewer").unwrap())
        .add_plugins(TopicIOPlugin)
        .run();
}
