//! A tree widget for displaying ROS-style topics in a collapsible hierarchy.
//!
//! Topics are slash-separated paths (e.g. `/robot/joint_states`) that form
//! a natural tree structure. This module provides:
//!
//! - [`Named`] – a trait exposing a `name()` accessor.
//! - [`SlashPath`] – a trait that decomposes slash-separated names into parts.
//! - [`TopicInfo`] – a Bevy [`Component`] carrying a topic name.
//! - [`TopicsTreePlugin`] – Bevy + egui plugin for the collapsible tree panel.

use std::collections::BTreeMap;

use bevy::prelude::*;
use bevy_egui::{EguiContexts, EguiPrimaryContextPass, egui};

use crate::ros_plugin::TopicInfo;
use crate::topics_io::{
    PublishRequest, Publisher, Subscription, TopicEditBuffer, TopicLatestValue,
};

// ---------------------------------------------------------------------------
// Traits
// ---------------------------------------------------------------------------

/// Anything that has a human-readable name.
pub trait Named {
    fn name(&self) -> &str;
}

/// Decompose a slash-separated name into its constituent path segments.
///
/// A leading slash produces a leading empty segment, so `/a/b/c` yields
/// `["", "a", "b", "c"]`.  Names without a leading slash are split normally:
/// `x/y` yields `["x", "y"]`.
pub trait SlashPath: Named {
    /// Return the individual segments of the slash-separated path.
    fn path_parts(&self) -> Vec<&str> {
        let name = self.name();
        if let Some(stripped) = name.strip_prefix('/') {
            // Leading empty segment represents the root "/"
            std::iter::once("")
                .chain(stripped.split('/').filter(|s| !s.is_empty()))
                .collect()
        } else {
            name.split('/').filter(|s| !s.is_empty()).collect()
        }
    }
}

impl Named for TopicInfo {
    fn name(&self) -> &str {
        &self.topic_name
    }
}

impl SlashPath for TopicInfo {}

// ---------------------------------------------------------------------------
// Internal tree data-structure (used to build the UI)
// ---------------------------------------------------------------------------

/// Intermediate tree built from a flat list of topic paths.
#[derive(Debug, Default)]
struct TopicTreeNode {
    /// Child nodes keyed by segment name.
    children: BTreeMap<String, TopicTreeNode>,
    /// If this node corresponds to an actual topic, its info and entity.
    leaf: Option<TopicLeaf>,
}

/// Data associated with a leaf node in the topic tree.
#[derive(Debug, Clone)]
struct TopicLeaf {
    info: TopicInfo,
    entity: Entity,
}

impl TopicTreeNode {
    /// Insert a topic into the tree at the position given by `parts`.
    fn insert(&mut self, parts: &[&str], info: TopicInfo, entity: Entity) {
        if parts.is_empty() {
            self.leaf = Some(TopicLeaf { info, entity });
            return;
        }
        let child = self
            .children
            .entry(parts[0].to_owned())
            .or_default();
        child.insert(&parts[1..], info, entity);
    }

    /// Build a tree from an iterator of `(Entity, &TopicInfo)` pairs.
    fn from_topics<'a>(topics: impl IntoIterator<Item = (Entity, &'a TopicInfo)>) -> Self {
        let mut root = Self::default();
        for (entity, topic) in topics {
            let parts = topic.path_parts();
            if parts.is_empty() {
                panic!("Unexpected empty path for topic '{}'", topic.name());
            }
            root.insert(&parts, topic.clone(), entity);
        }
        root
    }
}

// ---------------------------------------------------------------------------
// Marker components
// ---------------------------------------------------------------------------

/// Marker for topic entities that represent the authoritative data source
/// (as opposed to UI copies attached to tree leaf rows).
#[derive(Component, Debug)]
pub struct TopicDataSource;

// ---------------------------------------------------------------------------
// Panel mode
// ---------------------------------------------------------------------------

/// Controls how the topics tree is rendered.
#[derive(Resource, Debug, Clone, Copy, PartialEq, Eq)]
pub enum TopicsPanelMode {
    /// Render as a left side-panel (fixed width, leaves room for other content).
    Side,
    /// Render as the central panel (fills the remaining window area).
    Central,
}

// ---------------------------------------------------------------------------
// Plugin
// ---------------------------------------------------------------------------

/// Bevy plugin that registers the topics-tree UI systems.
///
/// Requires [`bevy_egui::EguiPlugin`] to be added first.
pub struct TopicsTreePlugin {
    pub panel_mode: TopicsPanelMode,
}

impl Plugin for TopicsTreePlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(self.panel_mode);
        app.add_systems(EguiPrimaryContextPass, topics_tree_ui_system);
    }
}

// ---------------------------------------------------------------------------
// egui rendering
// ---------------------------------------------------------------------------

/// Bevy system that renders the topics tree as an egui side-panel.
///
/// It queries all [`TopicInfo`] entities tagged with [`TopicDataSource`],
/// builds a [`TopicTreeNode`] hierarchy, and renders it using
/// [`egui::SidePanel`] with collapsible headers.
///
/// For topics with an active subscription ([`HasSubscription`]) the latest
/// received value is shown as a disabled text field.  For topics with an
/// active publisher ([`HasPublisher`]) an editable text field and a
/// *Submit* button are shown; pressing Enter also triggers a publish.
#[allow(clippy::too_many_arguments)]
pub fn topics_tree_ui_system(
    mut commands: Commands,
    mut contexts: EguiContexts,
    panel_mode: Res<TopicsPanelMode>,
    topics: Query<(Entity, &TopicInfo)>,
    mut values: Query<&mut TopicLatestValue>,
    mut buffers: Query<&mut TopicEditBuffer>,
    subscribable: Query<(), With<Subscription>>,
    publishable: Query<(), With<Publisher>>,
) {
    let topic_list: Vec<(Entity, &TopicInfo)> = topics.iter().collect();
    let has_topics = !topic_list.is_empty();
    let tree = TopicTreeNode::from_topics(topic_list);

    let Ok(ctx) = contexts.ctx_mut() else {
        return;
    };

    let render_body = |ui: &mut egui::Ui| {
        ui.heading("Topics");
        ui.separator();
        if !has_topics {
            render_empty_state(ui);
        } else {
        egui::ScrollArea::vertical().show(ui, |ui| {
            render_tree_children(
                ui,
                &tree,
                &mut commands,
                &mut values,
                &mut buffers,
                &subscribable,
                &publishable,
            );
        });
        }
    };

    match *panel_mode {
        TopicsPanelMode::Central => {
            egui::CentralPanel::default().show(ctx, render_body);
        }
        TopicsPanelMode::Side => {
            egui::SidePanel::left("topics_panel")
                .default_width(300.0)
                .show(ctx, render_body);
        }
    }
}

/// Render the empty-state message shown when no topics have been discovered.
fn render_empty_state(ui: &mut egui::Ui) {
    let seconds = ui.input(|i| i.time);
    let phase = (seconds * 2.0) as usize % 4;
    let dots = &["", ".", "..", "..."][phase];
    ui.add_space(8.0);
    ui.label(
        egui::RichText::new(format!("Waiting for topics on DDS domain{dots}"))
            .color(egui::Color32::from_rgb(160, 160, 160))
            .italics(),
    );
    ui.label(
        egui::RichText::new("No ROS 2 nodes discovered yet.")
            .color(egui::Color32::from_rgb(120, 120, 120))
            .small(),
    );
    ui.ctx().request_repaint();
}

/// Recursively render the children of a [`TopicTreeNode`] using egui.
fn render_tree_children(
    ui: &mut egui::Ui,
    node: &TopicTreeNode,
    commands: &mut Commands,
    values: &mut Query<&mut TopicLatestValue>,
    buffers: &mut Query<&mut TopicEditBuffer>,
    subscribable: &Query<(), With<Subscription>>,
    publishable: &Query<(), With<Publisher>>,
) {
    for (segment, child) in &node.children {
        let display = if segment.is_empty() {
            "/"
        } else {
            segment.as_str()
        };
        let is_leaf = child.children.is_empty();

        if is_leaf {
            // Leaf: show the segment name (green) and type name (grey),
            // followed by value / editing widgets when applicable.
            ui.horizontal(|ui| {
                ui.label(
                    egui::RichText::new(display).color(egui::Color32::from_rgb(140, 200, 140)),
                );
                if let Some(leaf) = &child.leaf {
                    ui.label(
                        egui::RichText::new(format!("({})", leaf.info.type_name))
                            .color(egui::Color32::GRAY)
                            .small(),
                    );
                }
            });

            // Inline value widgets (indented under the leaf label).
            if let Some(leaf) = &child.leaf {
                render_leaf_io(
                    ui,
                    leaf,
                    commands,
                    values,
                    buffers,
                    subscribable,
                    publishable,
                );
            }
        } else {
            // Branch: collapsible header with a trailing "/".
            let header_text = if segment.is_empty() {
                "/".to_string()
            } else {
                format!("{display}/")
            };
            egui::CollapsingHeader::new(
                egui::RichText::new(&header_text).color(egui::Color32::from_rgb(190, 210, 230)),
            )
            .default_open(false)
            .show(ui, |ui| {
                render_tree_children(
                    ui,
                    child,
                    commands,
                    values,
                    buffers,
                    subscribable,
                    publishable,
                );
            });
        }
    }
}

/// Render the subscription value / publisher editing widgets for a single leaf.
fn render_leaf_io(
    ui: &mut egui::Ui,
    leaf: &TopicLeaf,
    commands: &mut Commands,
    values: &mut Query<&mut TopicLatestValue>,
    buffers: &mut Query<&mut TopicEditBuffer>,
    subscribable: &Query<(), With<Subscription>>,
    publishable: &Query<(), With<Publisher>>,
) {
    let entity = leaf.entity;
    let is_subscribable = subscribable.get(entity).is_ok();
    let is_publishable = publishable.get(entity).is_ok();
    let has_value = values.get(entity).is_ok();
    let has_buffer = buffers.get(entity).is_ok();

    // If we don't even know whether the topic is subscribable/publishable yet,
    // show a loading indicator (transitional state).
    if !is_subscribable && !is_publishable {
        ui.indent(format!("io_{}", leaf.info.topic_name), |ui| {
            let seconds = ui.input(|i| i.time);
            let phase = (seconds * 2.0) as usize % 4;
            let dots = &["", ".", "..", "..."][phase];
            ui.label(
                egui::RichText::new(format!("loading{dots}"))
                    .color(egui::Color32::from_rgb(160, 160, 160))
                    .italics(),
            );
            ui.ctx().request_repaint();
        });
        return;
    }

    // If the type is not supported, show appropriate message.
    if (is_subscribable && !has_value) || (is_publishable && !has_buffer) {
        let msg = match (is_subscribable && !has_value, is_publishable && !has_buffer) {
            (true, true) => "View or edition of this type is not supported",
            (true, false) => "View of this type is not supported",
            (false, true) => "Edition of this type is not supported",
            _ => unreachable!(),
        };
        ui.indent(format!("io_{}", leaf.info.topic_name), |ui| {
            ui.label(
                egui::RichText::new(msg)
                    .color(egui::Color32::from_rgb(200, 160, 80))
                    .italics(),
            );
        });
        // If one side IS supported, fall through below to render it.
        if !has_value && !has_buffer {
            return;
        }
    }

    ui.indent(format!("io_{}", leaf.info.topic_name), |ui| {
        // --- Subscription: show latest value in a disabled text field ---
        if let Ok(latest) = values.get(entity) {
            match latest {
                TopicLatestValue::None => {
                    ui.label(
                        egui::RichText::new("no value received yet")
                            .color(egui::Color32::from_rgb(160, 160, 160))
                            .italics(),
                    );
                }
                TopicLatestValue::String(s) => {
                    let mut display = s.clone();
                    ui.add_enabled(
                        false,
                        egui::TextEdit::singleline(&mut display).desired_width(200.0),
                    );
                }
            }
        }

        // --- Publisher: editable text field + submit button ---
        if let Ok(mut buf) = buffers.get_mut(entity) {
            let mut submit = false;
            ui.horizontal(|ui| {
                match &mut *buf {
                    TopicEditBuffer::None => {}
                    TopicEditBuffer::String(s) => {
                        // s is the editable buffer for the text field.
                        // When the user presses Enter or clicks Submit, we set submit=true to trigger a publish.
                        let response = ui.add(
                            egui::TextEdit::singleline(s)
                                .desired_width(160.0)
                                .hint_text("publish…"),
                        );
                        if response.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter)) {
                            submit = true;
                        }
                        if ui.button("Submit").clicked() {
                            submit = true;
                        }
                    }
                }
            });
            if submit {
                commands.entity(entity).insert(PublishRequest);
            }
        }
    });
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use bevy_egui::EguiFullOutput;
    use crate::ros_plugin::TopicKind;

    // -----------------------------------------------------------------------
    // Unit tests for traits
    // -----------------------------------------------------------------------

    #[test]
    fn named_returns_topic_name() {
        let topic = TopicInfo::new("/sensor/imu", "sensor_msgs/Imu", TopicKind::Unknown);
        assert_eq!(topic.name(), "/sensor/imu");
    }

    #[test]
    fn slash_path_decomposes_leading_slash() {
        let topic = TopicInfo::new("/a/b/c", "some_msgs/Type", TopicKind::Unknown);
        assert_eq!(topic.path_parts(), vec!["", "a", "b", "c"]);
    }

    #[test]
    fn slash_path_no_leading_slash() {
        let topic = TopicInfo::new("x/y", "some_msgs/Type", TopicKind::Unknown);
        assert_eq!(topic.path_parts(), vec!["x", "y"]);
    }

    #[test]
    fn slash_path_single_segment() {
        let topic = TopicInfo::new("/rosout", "some_msgs/Type", TopicKind::Unknown);
        assert_eq!(topic.path_parts(), vec!["", "rosout"]);
    }

    #[test]
    fn slash_path_empty_string() {
        let topic = TopicInfo::new("", "some_msgs/Type", TopicKind::Unknown);
        let parts = topic.path_parts();
        assert!(parts.is_empty());
    }

    #[test]
    fn slash_path_root_only() {
        let topic = TopicInfo::new("/", "some_msgs/Type", TopicKind::Unknown);
        let parts = topic.path_parts();
        assert_eq!(parts, vec![""]);
    }

    // -----------------------------------------------------------------------
    // Unit tests for the internal tree builder
    // -----------------------------------------------------------------------

    /// Dummy entity for tests (Bevy entities from raw bits).
    fn dummy_entity(index: u32) -> Entity {
        Entity::from_raw_u32(index).expect("non-zero entity index")
    }

    #[test]
    fn tree_single_topic() {
        let topics = [TopicInfo::new(
            "/robot/joint_states",
            "sensor_msgs/JointState",
            TopicKind::Unknown,
        )];
        let tree = TopicTreeNode::from_topics(
            topics
                .iter()
                .enumerate()
                .map(|(i, t)| (dummy_entity(i as u32), t)),
        );
        // Leading-slash topics go under the empty-string key (displayed as "/")
        assert!(tree.children.contains_key(""), "missing '' root");
        let slash = &tree.children[""];
        assert!(slash.children.contains_key("robot"));
        let robot = &slash.children["robot"];
        assert!(robot.children.contains_key("joint_states"));
        assert_eq!(
            robot.children["joint_states"]
                .leaf
                .as_ref()
                .map(|l| l.info.topic_name.as_str()),
            Some("/robot/joint_states")
        );
    }

    #[test]
    fn tree_multiple_topics_shared_prefix() {
        let topics = vec![
            TopicInfo::new(
                "/robot/joint_states",
                "sensor_msgs/JointState",
                TopicKind::Unknown,
            ),
            TopicInfo::new("/robot/description", "std_msgs/String", TopicKind::Unknown),
            TopicInfo::new("/sensor/imu", "sensor_msgs/Imu", TopicKind::Unknown),
        ];
        let tree = TopicTreeNode::from_topics(
            topics
                .iter()
                .enumerate()
                .map(|(i, t)| (dummy_entity(i as u32), t)),
        );

        // All leading-slash topics sit under the empty-string key
        assert_eq!(tree.children.len(), 1);
        let slash = &tree.children[""];
        assert_eq!(slash.children.len(), 2);
        assert!(slash.children.contains_key("robot"));
        assert!(slash.children.contains_key("sensor"));

        // robot branch has two leaves
        let robot = &slash.children["robot"];
        assert_eq!(robot.children.len(), 2);
        assert!(robot.children.contains_key("joint_states"));
        assert!(robot.children.contains_key("description"));

        // sensor branch has one leaf
        let sensor = &slash.children["sensor"];
        assert_eq!(sensor.children.len(), 1);
        assert!(sensor.children.contains_key("imu"));
    }

    #[test]
    fn tree_deeply_nested() {
        let topics = [TopicInfo::new(
            "/a/b/c/d",
            "some_msgs/Type",
            TopicKind::Unknown,
        )];
        let tree = TopicTreeNode::from_topics(
            topics
                .iter()
                .enumerate()
                .map(|(i, t)| (dummy_entity(i as u32), t)),
        );
        let slash = &tree.children[""];
        let a = &slash.children["a"];
        let b = &a.children["b"];
        let c = &b.children["c"];
        let d = &c.children["d"];
        assert_eq!(
            d.leaf.as_ref().map(|l| l.info.topic_name.as_str()),
            Some("/a/b/c/d")
        );
        assert!(d.children.is_empty());
    }

    #[test]
    fn tree_preserves_sorted_order() {
        let topics = vec![
            TopicInfo::new("/z", "some_msgs/Type", TopicKind::Unknown),
            TopicInfo::new("/a", "some_msgs/Type", TopicKind::Unknown),
            TopicInfo::new("/m", "some_msgs/Type", TopicKind::Unknown),
        ];
        let tree = TopicTreeNode::from_topics(
            topics
                .iter()
                .enumerate()
                .map(|(i, t)| (dummy_entity(i as u32), t)),
        );
        let slash = &tree.children[""];
        let keys: Vec<&String> = slash.children.keys().collect();
        assert_eq!(keys, vec!["a", "m", "z"]);
    }

    /// Recursively collect all rendered text from egui shapes.
    fn collect_shape_texts(shapes: &[egui::epaint::ClippedShape]) -> Vec<String> {
        let mut texts = Vec::new();
        for clipped in shapes {
            collect_texts_from_shape(&clipped.shape, &mut texts);
        }
        texts
    }

    fn collect_texts_from_shape(shape: &egui::Shape, texts: &mut Vec<String>) {
        match shape {
            egui::Shape::Text(text_shape) => {
                texts.push(text_shape.galley.text().to_owned());
            }
            egui::Shape::Vec(shapes) => {
                for s in shapes {
                    collect_texts_from_shape(s, texts);
                }
            }
            _ => {}
        }
    }

    /// Resource used by the test capture system to store rendered text
    /// before `process_output_system` consumes `EguiFullOutput`.
    #[derive(Resource, Default)]
    struct CapturedEguiTexts(Vec<String>);

    /// Capture system that reads egui shapes and extracts text strings.
    /// Runs between `EndPass` (which populates `EguiFullOutput`) and
    /// `ProcessOutput` (which consumes it via `.take()`).
    fn capture_egui_texts(
        contexts: Query<&EguiFullOutput>,
        mut captured: ResMut<CapturedEguiTexts>,
    ) {
        for full_output in contexts.iter() {
            if let Some(output) = &full_output.0 {
                captured.0 = collect_shape_texts(&output.shapes);
            }
        }
    }

    #[test]
    fn empty_state_shows_waiting_label() {
        use bevy::asset::AssetPlugin;
        use bevy::image::ImagePlugin;
        use bevy::input::InputPlugin;
        use bevy::window::{Window, WindowPlugin};
        use bevy_egui::{EguiPlugin, EguiPostUpdateSet};

        let mut app = App::new();
        app.add_plugins(MinimalPlugins);
        app.add_plugins(AssetPlugin::default());
        app.init_asset::<bevy::prelude::Shader>();
        app.add_plugins(ImagePlugin::default());
        app.add_plugins(WindowPlugin {
            primary_window: Some(Window::default()),
            ..default()
        });
        app.add_plugins(InputPlugin);
        app.add_plugins(EguiPlugin { ..default() });
        app.add_plugins(TopicsTreePlugin {
            panel_mode: TopicsPanelMode::Central,
        });

        // Capture egui shape texts between EndPass and ProcessOutput,
        // because ProcessOutput consumes the output via .take().
        app.insert_resource(CapturedEguiTexts::default());
        app.add_systems(
            PostUpdate,
            capture_egui_texts
                .after(EguiPostUpdateSet::EndPass)
                .before(EguiPostUpdateSet::ProcessOutput),
        );

        // bevy_egui auto-creates the primary egui context on the first camera
        // entity it detects, so we need a camera for the context to exist.
        app.world_mut().spawn(Camera2d);

        // No TopicInfo entities -- the system should render the empty state.
        // First update initialises the egui context; second runs the UI system.
        app.update();
        app.update();

        let texts = &app.world().resource::<CapturedEguiTexts>().0;
        assert!(
            texts
                .iter()
                .any(|t| t.contains("Waiting for topics on DDS domain")),
            "Expected waiting message, found: {texts:?}"
        );
        assert!(
            texts
                .iter()
                .any(|t| t.contains("No ROS 2 nodes discovered yet.")),
            "Expected secondary message, found: {texts:?}"
        );
    }
}
