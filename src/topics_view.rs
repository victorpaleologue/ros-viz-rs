//! A tree widget for displaying ROS-style topics in a collapsible hierarchy.
//!
//! Topics are slash-separated paths (e.g. `/robot/joint_states`) that form
//! a natural tree structure. This module provides:
//!
//! - [`Named`] – a trait exposing a `name()` accessor.
//! - [`SlashPath`] – a trait that decomposes slash-separated names into parts.
//! - [`TopicInfo`] – a Bevy [`Component`] carrying a topic name.
//! - [`TopicsTreePlugin`] – Bevy + egui plugin for the collapsible tree panel.

use bevy::prelude::*;
use std::collections::BTreeMap;

use bevy_egui::{EguiContexts, egui};

use crate::topic_io::{
    HasPublisher, HasSubscription, PublishRequest, TopicEditBuffer, TopicLatestValue,
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
        if name.starts_with('/') {
            // Leading empty segment represents the root "/"
            std::iter::once("")
                .chain(name[1..].split('/').filter(|s| !s.is_empty()))
                .collect()
        } else {
            name.split('/').filter(|s| !s.is_empty()).collect()
        }
    }
}

// ---------------------------------------------------------------------------
// TopicInfo component
// ---------------------------------------------------------------------------

/// Classification of a DDS topic based on its name prefix/pattern.
///
/// ROS 2 maps its concepts onto DDS using naming conventions:
/// - `rt/…` — regular pub/sub topics
/// - `rq/…` — service request channels
/// - `rr/…` — service reply channels
/// - `rt/…/_action/…` — action-related topics
/// - anything else — unknown / internal
#[derive(Debug, Clone, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub enum TopicKind {
    /// A regular pub/sub topic, carrying the clean ROS topic name.
    Normal(String),
    /// A service request channel, carrying the service name.
    ServiceRequest(String),
    /// A service reply channel, carrying the service name.
    ServiceReply(String),
    /// An action-related topic, carrying the action name.
    Action(String),
    /// Anything that doesn't match a known pattern.
    Unknown,
}

/// A Bevy component that represents a single topic.
#[derive(Component, Debug, Clone, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub struct TopicInfo {
    /// The full topic name (raw DDS name), e.g. `rt/robot/joint_states`.
    pub topic_name: String,

    /// The data type name of the topic, e.g. `sensor_msgs/JointState`.
    pub type_name: String,

    /// The kind of topic (normal, service, action, …).
    pub kind: TopicKind,
}

impl TopicInfo {
    pub fn new(name: impl Into<String>, type_name: impl Into<String>, kind: TopicKind) -> Self {
        Self {
            topic_name: name.into(),
            type_name: type_name.into(),
            kind,
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
            .or_insert_with(TopicTreeNode::default);
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
///
/// Insert this as a Bevy resource before adding [`TopicsTreePlugin`].
/// Defaults to [`TopicsPanelMode::Side`].
#[derive(Resource, Debug, Clone, Copy, PartialEq, Eq)]
pub enum TopicsPanelMode {
    /// Render as a left side-panel (fixed width, leaves room for other content).
    Side,
    /// Render as the central panel (fills the remaining window area).
    Central,
}

impl Default for TopicsPanelMode {
    fn default() -> Self {
        Self::Side
    }
}

// ---------------------------------------------------------------------------
// Plugin
// ---------------------------------------------------------------------------

/// Bevy plugin that registers the topics-tree UI systems.
///
/// Requires [`bevy_egui::EguiPlugin`] to be added first.
///
/// Insert a [`TopicsPanelMode`] resource to choose between a side-panel
/// (default) and a central panel.
pub struct TopicsTreePlugin;

impl Plugin for TopicsTreePlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<TopicsPanelMode>();
        app.add_systems(Update, topics_tree_ui_system);
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
pub fn topics_tree_ui_system(
    mut commands: Commands,
    mut contexts: EguiContexts,
    panel_mode: Res<TopicsPanelMode>,
    topics: Query<(Entity, &TopicInfo), With<TopicDataSource>>,
    mut values: Query<&mut TopicLatestValue>,
    mut buffers: Query<&mut TopicEditBuffer>,
    subs: Query<(), With<HasSubscription>>,
    pubs: Query<(), With<HasPublisher>>,
) {
    let topic_list: Vec<(Entity, &TopicInfo)> = topics.iter().collect();
    let tree = TopicTreeNode::from_topics(topic_list);

    let ctx = contexts.ctx_mut();

    let render_body = |ui: &mut egui::Ui| {
        ui.heading("Topics");
        ui.separator();
        egui::ScrollArea::vertical().show(ui, |ui| {
            render_tree_children(
                ui,
                &tree,
                &mut commands,
                &mut values,
                &mut buffers,
                &subs,
                &pubs,
            );
        });
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

/// Recursively render the children of a [`TopicTreeNode`] using egui.
fn render_tree_children(
    ui: &mut egui::Ui,
    node: &TopicTreeNode,
    commands: &mut Commands,
    values: &mut Query<&mut TopicLatestValue>,
    buffers: &mut Query<&mut TopicEditBuffer>,
    subs: &Query<(), With<HasSubscription>>,
    pubs: &Query<(), With<HasPublisher>>,
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
                render_leaf_io(ui, leaf, commands, values, buffers, subs, pubs);
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
            .default_open(true)
            .show(ui, |ui| {
                render_tree_children(ui, child, commands, values, buffers, subs, pubs);
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
    subs: &Query<(), With<HasSubscription>>,
    pubs: &Query<(), With<HasPublisher>>,
) {
    let entity = leaf.entity;
    let has_sub = subs.get(entity).is_ok();
    let has_pub = pubs.get(entity).is_ok();

    if !has_sub && !has_pub {
        return;
    }

    ui.indent(format!("io_{}", leaf.info.topic_name), |ui| {
        // --- Subscription: show latest value in a disabled text field ---
        if has_sub {
            if let Ok(latest) = values.get(entity) {
                let mut display = latest.0.clone();
                ui.add_enabled(
                    false,
                    egui::TextEdit::singleline(&mut display).desired_width(200.0),
                );
            }
        }

        // --- Publisher: editable text field + submit button ---
        if has_pub {
            if let Ok(mut buf) = buffers.get_mut(entity) {
                let mut submit = false;
                ui.horizontal(|ui| {
                    let response = ui.add(
                        egui::TextEdit::singleline(&mut buf.0)
                            .desired_width(160.0)
                            .hint_text("publish…"),
                    );
                    if response.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter)) {
                        submit = true;
                    }
                    if ui.button("Submit").clicked() {
                        submit = true;
                    }
                });
                if submit {
                    commands.entity(entity).insert(PublishRequest);
                }
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
        Entity::from_raw(index)
    }

    #[test]
    fn tree_single_topic() {
        let topics = vec![TopicInfo::new(
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
        let topics = vec![TopicInfo::new(
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
}
