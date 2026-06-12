//! A tree widget for displaying ROS-style topics in a collapsible hierarchy.
//!
//! Topics are slash-separated paths (e.g. `/robot/joint_states`) that form
//! a natural tree structure. This module provides:
//!
//! - [`Named`] – a trait exposing a `name()` accessor.
//! - [`SlashPath`] – a trait that decomposes slash-separated names into parts.
//! - [`TopicInfo`] – a Bevy [`Component`] carrying a topic name.
//! - [`TopicsTreePlugin`] – Bevy + egui plugin for the collapsible tree panel.
//!
//! Topic values are reflected [`serde_json::Value`] trees (see
//! [`crate::messages`]); rendering is fully generic: any message type present
//! in the [`MessageRegistry`] gets a recursive read-only view for
//! subscriptions and recursive editable widgets for publishers.

use std::collections::BTreeMap;

use bevy::prelude::*;
use bevy_egui::{EguiContexts, EguiPrimaryContextPass, egui};
use serde_json::Value;

use crate::messages::MessageRegistry;
use crate::ros_plugin::TopicInfo;
use crate::topics_io::{PublishRequest, Publisher, Subscription, TopicEdit, TopicValue};

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
        let child = self.children.entry(parts[0].to_owned()).or_default();
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
        app.init_resource::<MessageRegistry>();
        app.add_systems(EguiPrimaryContextPass, topics_tree_ui_system);
    }
}

// ---------------------------------------------------------------------------
// Colors
// ---------------------------------------------------------------------------

/// Color used for message field names.
const FIELD_COLOR: egui::Color32 = egui::Color32::from_rgb(150, 170, 200);
/// Color used for scalar values in the read-only view.
const VALUE_COLOR: egui::Color32 = egui::Color32::from_rgb(220, 220, 220);
/// Color used for transitional / informational text.
const INFO_COLOR: egui::Color32 = egui::Color32::from_rgb(160, 160, 160);
/// Color used for warnings (e.g. unsupported types).
const WARN_COLOR: egui::Color32 = egui::Color32::from_rgb(200, 160, 80);

// ---------------------------------------------------------------------------
// egui rendering
// ---------------------------------------------------------------------------

/// Bevy system that renders the topics tree as an egui panel.
///
/// It queries all [`TopicInfo`] entities, builds a [`TopicTreeNode`]
/// hierarchy, and renders it using collapsible headers.
///
/// For topics with an active [`Subscription`] the latest reflected value
/// ([`TopicValue`]) is rendered as a read-only tree.  For topics with an
/// active [`Publisher`] the [`TopicEdit`] buffer is rendered as recursive
/// editable widgets with a *Publish* button; pressing Enter in a text field
/// also publishes.  Types absent from the [`MessageRegistry`] are flagged as
/// unsupported.
#[allow(clippy::too_many_arguments)]
pub fn topics_tree_ui_system(
    mut commands: Commands,
    mut contexts: EguiContexts,
    panel_mode: Res<TopicsPanelMode>,
    registry: Res<MessageRegistry>,
    topics: Query<(Entity, &TopicInfo)>,
    values: Query<&TopicValue>,
    mut edits: Query<&mut TopicEdit>,
    subscribed: Query<(), With<Subscription>>,
    published: Query<(), With<Publisher>>,
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
                    &registry,
                    &values,
                    &mut edits,
                    &subscribed,
                    &published,
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
            .color(INFO_COLOR)
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
#[allow(clippy::too_many_arguments)]
fn render_tree_children(
    ui: &mut egui::Ui,
    node: &TopicTreeNode,
    commands: &mut Commands,
    registry: &MessageRegistry,
    values: &Query<&TopicValue>,
    edits: &mut Query<&mut TopicEdit>,
    subscribed: &Query<(), With<Subscription>>,
    published: &Query<(), With<Publisher>>,
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
                    ui, leaf, commands, registry, values, edits, subscribed, published,
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
                    ui, child, commands, registry, values, edits, subscribed, published,
                );
            });
        }
    }
}

/// Render the subscription value / publisher editing widgets for a single leaf.
#[allow(clippy::too_many_arguments)]
fn render_leaf_io(
    ui: &mut egui::Ui,
    leaf: &TopicLeaf,
    commands: &mut Commands,
    registry: &MessageRegistry,
    values: &Query<&TopicValue>,
    edits: &mut Query<&mut TopicEdit>,
    subscribed: &Query<(), With<Subscription>>,
    published: &Query<(), With<Publisher>>,
) {
    let entity = leaf.entity;
    let indent_id = ui.id().with("io").with(&leaf.info.topic_name);

    // Types absent from the registry can neither be viewed nor edited.
    if !registry.contains(&leaf.info.type_name) {
        ui.indent(indent_id, |ui| {
            ui.label(
                egui::RichText::new("Message type not supported")
                    .color(WARN_COLOR)
                    .italics(),
            );
        });
        return;
    }

    let has_subscription = subscribed.get(entity).is_ok();
    let has_publisher = published.get(entity).is_ok();

    // Supported type but no I/O set up yet: transitional state.
    if !has_subscription && !has_publisher {
        ui.indent(indent_id, |ui| {
            let seconds = ui.input(|i| i.time);
            let phase = (seconds * 2.0) as usize % 4;
            let dots = &["", ".", "..", "..."][phase];
            ui.label(
                egui::RichText::new(format!("loading{dots}"))
                    .color(INFO_COLOR)
                    .italics(),
            );
            ui.ctx().request_repaint();
        });
        return;
    }

    ui.indent(indent_id, |ui| {
        // --- Subscription: latest reflected value, read-only ---
        if let Ok(latest) = values.get(entity) {
            match &latest.0 {
                None => {
                    ui.label(
                        egui::RichText::new("no value received yet")
                            .color(INFO_COLOR)
                            .italics(),
                    );
                }
                Some(value) => {
                    render_value_read(ui, indent_id.with("read"), value);
                }
            }
        }

        // --- Publisher: recursive editable widgets + publish button ---
        if let Ok(mut edit) = edits.get_mut(entity) {
            let mut submit = render_value_edit(ui, indent_id.with("edit"), &mut edit.0);
            if ui.button("Publish").clicked() {
                submit = true;
            }
            if submit {
                commands.entity(entity).insert(PublishRequest);
            }
        }
    });
}

// ---------------------------------------------------------------------------
// Generic reflected-value rendering (read-only)
// ---------------------------------------------------------------------------

/// Render a reflected message value as a compact read-only tree.
///
/// Top-level objects render their fields directly (no wrapping header).
fn render_value_read(ui: &mut egui::Ui, id: egui::Id, value: &Value) {
    match value {
        Value::Object(map) => {
            for (key, child) in map {
                render_field_read(ui, id.with(key.as_str()), key, child);
            }
        }
        other => {
            ui.label(egui::RichText::new(scalar_text(other)).color(VALUE_COLOR));
        }
    }
}

/// Render one named field of a reflected value (read-only).
fn render_field_read(ui: &mut egui::Ui, id: egui::Id, label: &str, value: &Value) {
    match value {
        Value::Object(map) => {
            ui.label(egui::RichText::new(format!("{label}:")).color(FIELD_COLOR));
            ui.indent(id, |ui| {
                for (key, child) in map {
                    render_field_read(ui, id.with(key.as_str()), key, child);
                }
            });
        }
        Value::Array(items) => {
            egui::CollapsingHeader::new(
                egui::RichText::new(format!("{label} [{}]", items.len())).color(FIELD_COLOR),
            )
            .id_salt(id)
            .default_open(false)
            .show(ui, |ui| {
                for (index, item) in items.iter().enumerate() {
                    render_field_read(ui, id.with(index), &index.to_string(), item);
                }
            });
        }
        scalar => {
            ui.horizontal(|ui| {
                ui.label(egui::RichText::new(format!("{label}:")).color(FIELD_COLOR));
                ui.label(egui::RichText::new(scalar_text(scalar)).color(VALUE_COLOR));
            });
        }
    }
}

/// Compact display text for a scalar [`Value`].
fn scalar_text(value: &Value) -> String {
    match value {
        Value::Null => "null".to_owned(),
        Value::Bool(b) => b.to_string(),
        Value::Number(n) => n.to_string(),
        Value::String(s) => s.clone(),
        // Objects/arrays are handled by the callers.
        other => other.to_string(),
    }
}

// ---------------------------------------------------------------------------
// Generic reflected-value rendering (editable)
// ---------------------------------------------------------------------------

/// Render a reflected message value as recursive editable widgets.
///
/// Returns `true` when the user pressed Enter in a text field, signalling a
/// publish request.
fn render_value_edit(ui: &mut egui::Ui, id: egui::Id, value: &mut Value) -> bool {
    match value {
        Value::Object(map) => {
            let mut submit = false;
            for (key, child) in map.iter_mut() {
                let key_id = id.with(key.as_str());
                submit |= render_field_edit(ui, key_id, key, child);
            }
            submit
        }
        other => render_scalar_edit(ui, id, other),
    }
}

/// Render one named field of a reflected value (editable).
fn render_field_edit(ui: &mut egui::Ui, id: egui::Id, label: &str, value: &mut Value) -> bool {
    match value {
        Value::Object(map) => {
            ui.label(egui::RichText::new(format!("{label}:")).color(FIELD_COLOR));
            let mut submit = false;
            ui.indent(id, |ui| {
                for (key, child) in map.iter_mut() {
                    let key_id = id.with(key.as_str());
                    submit |= render_field_edit(ui, key_id, key, child);
                }
            });
            submit
        }
        Value::Array(items) => {
            let mut submit = false;
            egui::CollapsingHeader::new(
                egui::RichText::new(format!("{label} [{}]", items.len())).color(FIELD_COLOR),
            )
            .id_salt(id)
            .default_open(true)
            .show(ui, |ui| {
                for (index, item) in items.iter_mut().enumerate() {
                    submit |= render_field_edit(ui, id.with(index), &index.to_string(), item);
                }
                ui.horizontal(|ui| {
                    if ui
                        .button("+")
                        .on_hover_text("Append an element (clones the last one)")
                        .clicked()
                    {
                        // Clone the last element when possible so the new
                        // entry matches the element type; Null otherwise.
                        let new_item = items.last().cloned().unwrap_or(Value::Null);
                        items.push(new_item);
                    }
                    if ui
                        .button("-")
                        .on_hover_text("Remove the last element")
                        .clicked()
                    {
                        items.pop();
                    }
                });
            });
            submit
        }
        scalar => {
            let mut submit = false;
            ui.horizontal(|ui| {
                ui.label(egui::RichText::new(format!("{label}:")).color(FIELD_COLOR));
                submit = render_scalar_edit(ui, id, scalar);
            });
            submit
        }
    }
}

/// Render an editable widget for a scalar [`Value`].
///
/// Returns `true` when the user pressed Enter in a text field.
fn render_scalar_edit(ui: &mut egui::Ui, id: egui::Id, value: &mut Value) -> bool {
    match value {
        Value::Bool(b) => {
            ui.checkbox(b, "");
            false
        }
        Value::Number(n) => {
            // Preserve the JSON number flavour: floats stay floats and
            // integers stay integers, so reflection back to the typed
            // message keeps working.
            if n.is_f64() {
                let mut x = n.as_f64().unwrap_or(0.0);
                if ui.add(egui::DragValue::new(&mut x).speed(0.05)).changed() {
                    *value = Value::from(x);
                }
            } else if let Some(i) = n.as_i64() {
                let mut x = i;
                if ui.add(egui::DragValue::new(&mut x)).changed() {
                    *value = Value::from(x);
                }
            } else if let Some(u) = n.as_u64() {
                let mut x = u;
                if ui.add(egui::DragValue::new(&mut x)).changed() {
                    *value = Value::from(x);
                }
            }
            false
        }
        Value::String(s) => {
            let response = ui.add(
                egui::TextEdit::singleline(s)
                    .id_salt(id)
                    .desired_width(160.0),
            );
            response.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter))
        }
        Value::Null => {
            ui.label(egui::RichText::new("null").color(INFO_COLOR).italics());
            false
        }
        // Objects/arrays are handled by the callers.
        _ => false,
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::messages::{DynPublisher, DynSubscription};
    use crate::ros_plugin::TopicKind;
    use bevy_egui::EguiFullOutput;
    use serde_json::json;

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
        let topics = [
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
        let topics = [
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

    // -----------------------------------------------------------------------
    // egui rendering tests
    // -----------------------------------------------------------------------

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

    /// Build a headless Bevy app with egui and the topics-tree plugin, plus
    /// the text-capture machinery used by the rendering tests.
    fn ui_test_app() -> App {
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
        app
    }

    /// Run enough updates for the UI to render and return the captured texts.
    fn rendered_texts(app: &mut App) -> Vec<String> {
        // First update initialises the egui context; second runs the UI system.
        app.update();
        app.update();
        app.world().resource::<CapturedEguiTexts>().0.clone()
    }

    /// Fake subscription used to mark an entity as subscribed in UI tests.
    struct NullSubscription;

    impl DynSubscription for NullSubscription {
        fn poll(&self) -> Option<Value> {
            None
        }
    }

    /// Fake publisher used to mark an entity as publishable in UI tests.
    struct NullPublisher;

    impl DynPublisher for NullPublisher {
        fn publish(&self, _value: &Value) -> Result<(), String> {
            Ok(())
        }
    }

    #[test]
    fn empty_state_shows_waiting_label() {
        let mut app = ui_test_app();

        // No TopicInfo entities -- the system should render the empty state.
        let texts = rendered_texts(&mut app);
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

    #[test]
    fn subscribed_twist_renders_fields() {
        let mut app = ui_test_app();
        let twist = json!({
            "linear": {"x": 1.5, "y": 0.0, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": -0.75},
        });
        app.world_mut().spawn((
            // No leading slash: the leaf renders at the root level instead of
            // under the collapsed "/" header (collapsing headers default closed).
            TopicInfo::new(
                "cmd_vel",
                "geometry_msgs/Twist",
                TopicKind::Normal("/cmd_vel".into()),
            ),
            Subscription(Box::new(NullSubscription)),
            TopicValue(Some(twist)),
        ));

        let texts = rendered_texts(&mut app);
        for expected in ["cmd_vel", "linear:", "angular:", "x:", "1.5", "-0.75"] {
            assert!(
                texts.iter().any(|t| t.contains(expected)),
                "Expected '{expected}' in rendered texts, found: {texts:?}"
            );
        }
    }

    #[test]
    fn publishable_twist_renders_editor_and_publish_button() {
        let mut app = ui_test_app();
        let registry = MessageRegistry::standard();
        let seed = registry.default_value("geometry_msgs/Twist").unwrap();
        app.world_mut().spawn((
            TopicInfo::new(
                "cmd_vel",
                "geometry_msgs/Twist",
                TopicKind::Normal("/cmd_vel".into()),
            ),
            Publisher(Box::new(NullPublisher)),
            TopicEdit(seed),
        ));

        let texts = rendered_texts(&mut app);
        for expected in ["linear:", "angular:", "Publish"] {
            assert!(
                texts.iter().any(|t| t.contains(expected)),
                "Expected '{expected}' in rendered texts, found: {texts:?}"
            );
        }
    }

    #[test]
    fn unregistered_type_shows_unsupported() {
        let mut app = ui_test_app();
        app.world_mut().spawn(TopicInfo::new(
            "proprietary",
            "custom_msgs/Proprietary",
            TopicKind::Normal("/proprietary".into()),
        ));

        let texts = rendered_texts(&mut app);
        assert!(
            texts
                .iter()
                .any(|t| t.contains("Message type not supported")),
            "Expected unsupported-type message, found: {texts:?}"
        );
    }

    #[test]
    fn registered_type_without_io_shows_loading() {
        let mut app = ui_test_app();
        app.world_mut().spawn(TopicInfo::new(
            "chatter",
            "std_msgs/String",
            TopicKind::Normal("/chatter".into()),
        ));

        let texts = rendered_texts(&mut app);
        assert!(
            texts.iter().any(|t| t.contains("loading")),
            "Expected loading message, found: {texts:?}"
        );
    }

    #[test]
    fn subscribed_without_value_shows_placeholder() {
        let mut app = ui_test_app();
        app.world_mut().spawn((
            TopicInfo::new(
                "chatter",
                "std_msgs/String",
                TopicKind::Normal("/chatter".into()),
            ),
            Subscription(Box::new(NullSubscription)),
            TopicValue::default(),
        ));

        let texts = rendered_texts(&mut app);
        assert!(
            texts.iter().any(|t| t.contains("no value received yet")),
            "Expected placeholder, found: {texts:?}"
        );
    }
}
