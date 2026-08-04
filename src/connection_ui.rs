//! egui panel for switching the data source at runtime.
//!
//! A thin top bar that shows the active connection and lets the user switch
//! between the demo, a rosbridge URL, and (native only) a DDS domain. It only
//! writes a [`PendingConnection`] request; [`crate::connection`] does the work.

use bevy::prelude::*;
use bevy_egui::{EguiContexts, EguiPrimaryContextPass, egui};

use crate::connection::{ActiveConnection, ConnectionMode, PendingConnection};
use crate::options::Options;

/// Amber used for the Android "DDS is unreliable on mobile" warning.
#[cfg(target_os = "android")]
const WARN_COLOR: egui::Color32 = egui::Color32::from_rgb(220, 170, 60);

/// Editable form state for the connection bar, kept across frames.
#[derive(Resource)]
struct ConnectionForm {
    rosbridge_url: String,
    domain: u16,
}

impl Default for ConnectionForm {
    fn default() -> Self {
        Self {
            rosbridge_url: "ws://localhost:9090".into(),
            domain: 0,
        }
    }
}

/// Register the connection panel, seeding the form from `options`.
pub fn register(app: &mut App, options: &Options) {
    let mut form = ConnectionForm::default();
    if let Some(url) = &options.rosbridge {
        form.rosbridge_url = url.clone();
    }
    form.domain = options.domain;
    app.insert_resource(form);
    app.add_systems(
        EguiPrimaryContextPass,
        connection_panel.in_set(ConnectionPanelSet),
    );
}

/// System set for [`connection_panel`], so other egui systems (e.g. the
/// Android safe-area insets) can order themselves around the connection bar
/// without referencing the panel's private parameter types.
#[derive(SystemSet, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub(crate) struct ConnectionPanelSet;

// `form` is only mutated by the rosbridge/DDS widgets; a build with neither
// transport feature shows just the Demo button and leaves it untouched.
#[cfg_attr(
    not(any(feature = "rosbridge", feature = "ros2")),
    allow(unused_variables, unused_mut)
)]
fn connection_panel(
    mut contexts: EguiContexts,
    mut form: ResMut<ConnectionForm>,
    active: Res<ActiveConnection>,
    mut pending: ResMut<PendingConnection>,
) {
    let Ok(ctx) = contexts.ctx_mut() else {
        return;
    };

    egui::TopBottomPanel::top("connection_bar").show(ctx, |ui| {
        ui.horizontal_wrapped(|ui| {
            ui.heading("Source");
            ui.separator();

            // Status: the live source, plus a hint while a switch is pending.
            match &active.0 {
                Some(mode) => ui.label(format!("● {}", mode.label())),
                None => ui.label("○ connecting…"),
            };
            if let Some(req) = &pending.0
                && active.0.as_ref() != Some(req)
            {
                ui.weak(format!("→ switching to {}…", req.label()));
            }
            ui.separator();

            let is_active = |m: &ConnectionMode| active.0.as_ref() == Some(m);

            if ui
                .selectable_label(is_active(&ConnectionMode::Demo), "Demo")
                .clicked()
            {
                pending.request(ConnectionMode::Demo);
            }

            #[cfg(feature = "rosbridge")]
            {
                ui.separator();
                ui.label("rosbridge:");
                ui.add(
                    egui::TextEdit::singleline(&mut form.rosbridge_url)
                        .desired_width(180.0)
                        .hint_text("ws://host:9090"),
                );
                if ui.button("Connect").clicked() {
                    pending.request(ConnectionMode::Rosbridge(form.rosbridge_url.clone()));
                }
            }

            // When native ROS 2 (DDS) isn't compiled in — the Android and web
            // builds are rosbridge-only by design (DDS multicast is unreliable
            // on mobile and unavailable in the browser) — say so. Otherwise the
            // absent DDS option reads as a bug rather than a deliberate choice.
            // (#37)
            #[cfg(all(feature = "rosbridge", not(feature = "ros2")))]
            {
                ui.separator();
                ui.label("ⓘ rosbridge-only").on_hover_text(
                    "This build connects over rosbridge only — native DDS isn't \
                     available here.\nTo view a ROS 2 (DDS) graph, run \
                     `rosbridge_server` on a machine on your network and point \
                     this field at its ws://<host>:9090 address.",
                );
            }

            #[cfg(feature = "ros2")]
            {
                ui.separator();
                ui.label("DDS domain:");
                ui.add(egui::DragValue::new(&mut form.domain).range(0..=232));
                if ui.button("Connect").clicked() {
                    pending.request(ConnectionMode::Dds(form.domain));
                }
                // On a phone, native DDS usually won't find the robot: discovery
                // is UDP multicast, which mobile data and many Wi-Fi networks
                // drop. We hold a multicast lock (best effort) but warn anyway
                // and point at rosbridge, which always works.
                #[cfg(target_os = "android")]
                ui.label(egui::RichText::new("⚠ unreliable on mobile").color(WARN_COLOR))
                    .on_hover_text(
                        "Native DDS finds the robot via UDP multicast. It can \
                         work when the phone is on the same Wi-Fi LAN as the \
                         robot, but mobile data and many Wi-Fi networks block \
                         multicast — if it doesn't connect, use rosbridge.",
                    );
            }
        });
    });
}
