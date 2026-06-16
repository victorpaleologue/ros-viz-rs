//! egui panel for switching the data source at runtime.
//!
//! A thin top bar that shows the active connection and lets the user switch
//! between the demo, a rosbridge URL, and (native only) a DDS domain. It only
//! writes a [`PendingConnection`] request; [`crate::connection`] does the work.

use bevy::prelude::*;
use bevy_egui::{EguiContexts, EguiPrimaryContextPass, egui};

use crate::connection::{ActiveConnection, ConnectionMode, PendingConnection};
use crate::options::Options;

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
    app.add_systems(EguiPrimaryContextPass, connection_panel);
}

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

            #[cfg(feature = "ros2")]
            {
                ui.separator();
                ui.label("DDS domain:");
                ui.add(egui::DragValue::new(&mut form.domain).range(0..=232));
                if ui.button("Connect").clicked() {
                    pending.request(ConnectionMode::Dds(form.domain));
                }
            }
        });
    });
}
