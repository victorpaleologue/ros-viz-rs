//! Display a tree of ROS 2 topics discovered via DDS.
//!
//! # Usage
//!
//! ```bash
//! cargo run --example view_topics --features ros,ui
//! ```

use bevy::prelude::*;
use bevy_egui::EguiPlugin;
use ros_viz_rs::topics_view::{TopicsPanelMode, TopicsTreePlugin};

fn main() {
    let mut app = App::new();
    app.add_plugins(DefaultPlugins.set(WindowPlugin {
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
    });

    // Register the DDS discovery + I/O systems, then open a session on
    // domain 0. The systems are gated on the session, so order is fine.
    ros_viz_rs::ros_plugin::register_systems(&mut app);
    ros_viz_rs::topics_io::register_systems(&mut app);
    ros_viz_rs::ros_plugin::connect(app.world_mut(), 0u16, "topics_viewer")
        .expect("failed to open DDS session");

    app.run();
}
