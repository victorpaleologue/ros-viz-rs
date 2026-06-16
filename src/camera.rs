//! Interactive orbit camera for the windowed viewer.
//!
//! The 3D view has no camera controls of its own — [`crate::scene`] frames the
//! robot once and the camera then sits still. This plugin makes that camera
//! interactive with the **mouse** (desktop, web) and **touch** (Android):
//!
//! - drag / one finger — orbit around the focus point,
//! - wheel / two-finger pinch — zoom,
//! - right- or middle-drag / two-finger drag — pan the focus.
//!
//! It is purely additive. [`crate::scene::frame_camera_on_new_robot`] still
//! sets the initial pose and *seeds* the [`OrbitController`] from it (via
//! [`OrbitController::aim_from_eye`]), so headless / snapshot builds — which
//! never add this plugin — behave exactly as before.

use bevy::input::mouse::{AccumulatedMouseMotion, AccumulatedMouseScroll};
use bevy::input::touch::Touches;
use bevy::prelude::*;
use bevy_egui::EguiContexts;

/// Smallest orbit radius, so zooming can't cross through the focus point.
const MIN_RADIUS: f32 = 0.05;
/// Keep the camera just shy of straight up/down to avoid gimbal flip.
const MAX_PITCH: f32 = std::f32::consts::FRAC_PI_2 - 0.05;
/// Orbit sensitivity, radians per pixel of drag.
const ORBIT_SPEED: f32 = 0.005;
/// Pan sensitivity, world units per pixel per unit of radius.
const PAN_SPEED: f32 = 0.0015;
/// Wheel zoom sensitivity, as a fraction of radius per scroll unit.
const ZOOM_SPEED: f32 = 0.1;
/// Pinch zoom sensitivity, as a fraction of radius per pixel of pinch.
const PINCH_ZOOM_SPEED: f32 = 0.005;

/// Spherical orbit state for a camera: the point it looks at ([`focus`]) and
/// the direction/distance it views from ([`yaw`]/[`pitch`]/[`radius`]).
///
/// [`focus`]: Self::focus
/// [`yaw`]: Self::yaw
/// [`pitch`]: Self::pitch
/// [`radius`]: Self::radius
#[derive(Component, Debug, Clone)]
pub struct OrbitController {
    /// World-space point the camera orbits and looks at.
    pub focus: Vec3,
    /// Rotation around the world Y (up) axis, radians.
    pub yaw: f32,
    /// Elevation above the focus, radians; clamped off the poles.
    pub pitch: f32,
    /// Distance from the focus to the camera.
    pub radius: f32,
    /// Seeded once from the auto-frame; until then input is ignored so we
    /// don't fight the framing.
    pub initialized: bool,
}

impl Default for OrbitController {
    fn default() -> Self {
        Self {
            focus: Vec3::ZERO,
            yaw: 0.0,
            pitch: 0.0,
            radius: 5.0,
            initialized: false,
        }
    }
}

impl OrbitController {
    /// Seed the orbit state from an `eye` position looking at `focus` — the
    /// inverse of [`eye`](Self::eye). Marks the controller initialized.
    pub fn aim_from_eye(&mut self, focus: Vec3, eye: Vec3) {
        let v = eye - focus;
        self.focus = focus;
        self.radius = v.length().max(MIN_RADIUS);
        let dir = v / self.radius;
        self.pitch = dir.y.clamp(-1.0, 1.0).asin().clamp(-MAX_PITCH, MAX_PITCH);
        self.yaw = dir.x.atan2(dir.z);
        self.initialized = true;
    }

    /// World-space eye position implied by the current orbit state.
    pub fn eye(&self) -> Vec3 {
        let (sp, cp) = self.pitch.sin_cos();
        let (sy, cy) = self.yaw.sin_cos();
        self.focus + Vec3::new(cp * sy, sp, cp * cy) * self.radius
    }

    /// Camera transform placing the eye at [`eye`](Self::eye), looking at the
    /// focus with world up.
    pub fn transform(&self) -> Transform {
        Transform::from_translation(self.eye()).looking_at(self.focus, Vec3::Y)
    }

    /// Orbit by pixel deltas (positive `dx` drags right, `dy` drags down).
    fn orbit(&mut self, dx: f32, dy: f32) {
        self.yaw -= dx * ORBIT_SPEED;
        self.pitch = (self.pitch + dy * ORBIT_SPEED).clamp(-MAX_PITCH, MAX_PITCH);
    }

    /// Multiply the radius by `factor`, clamped to the minimum radius.
    fn zoom(&mut self, factor: f32) {
        self.radius = (self.radius * factor).max(MIN_RADIUS);
    }

    /// Pan the focus in the camera's screen plane by pixel deltas, scaled by
    /// radius so it feels consistent at any zoom.
    fn pan(&mut self, dx: f32, dy: f32) {
        let t = self.transform();
        let scale = self.radius * PAN_SPEED;
        self.focus += (t.right() * (-dx) + t.up() * dy) * scale;
    }
}

/// Adds mouse + touch orbit controls to cameras carrying an
/// [`OrbitController`]. Add only on windowed builds (it reads the egui pointer
/// state to avoid hijacking the topics panel).
pub struct OrbitCameraPlugin;

impl Plugin for OrbitCameraPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Update, orbit_camera_input);
    }
}

/// Read mouse and touch input and drive the orbit camera.
fn orbit_camera_input(
    mut cameras: Query<(&mut Transform, &mut OrbitController)>,
    buttons: Res<ButtonInput<MouseButton>>,
    motion: Res<AccumulatedMouseMotion>,
    scroll: Res<AccumulatedMouseScroll>,
    touches: Res<Touches>,
    mut contexts: EguiContexts,
    mut prev_pinch: Local<Option<f32>>,
) {
    let Some((mut transform, mut orbit)) = cameras.iter_mut().next() else {
        return;
    };
    // Wait for the auto-frame to seed us before responding to input.
    if !orbit.initialized {
        return;
    }

    let positions: Vec<Vec2> = touches.iter().map(|t| t.position()).collect();
    let deltas: Vec<Vec2> = touches
        .iter()
        .map(|t| t.position() - t.previous_position())
        .collect();

    if positions.len() >= 2 {
        // Two fingers: pinch to zoom, drag to pan.
        let dist = positions[0].distance(positions[1]);
        if let Some(prev) = *prev_pinch {
            orbit.zoom(1.0 - (dist - prev) * PINCH_ZOOM_SPEED);
        }
        *prev_pinch = Some(dist);
        let avg = (deltas[0] + deltas[1]) * 0.5;
        orbit.pan(avg.x, avg.y);
    } else if positions.len() == 1 {
        // One finger: orbit.
        *prev_pinch = None;
        orbit.orbit(deltas[0].x, deltas[0].y);
    } else {
        *prev_pinch = None;
        // Don't steal the pointer while egui (the topics panel) is using it.
        let egui_pointer = contexts
            .ctx_mut()
            .map(|c| c.wants_pointer_input())
            .unwrap_or(false);
        if !egui_pointer {
            if buttons.pressed(MouseButton::Left) {
                orbit.orbit(motion.delta.x, motion.delta.y);
            } else if buttons.pressed(MouseButton::Right) || buttons.pressed(MouseButton::Middle) {
                orbit.pan(motion.delta.x, motion.delta.y);
            }
            if scroll.delta.y != 0.0 {
                orbit.zoom(1.0 - scroll.delta.y * ZOOM_SPEED);
            }
        }
    }

    *transform = orbit.transform();
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn eye_round_trips_through_orbit_state() {
        let focus = Vec3::new(1.0, 2.0, -3.0);
        let eye = focus + Vec3::new(2.0, 1.5, 2.0);
        let mut c = OrbitController::default();
        c.aim_from_eye(focus, eye);
        assert!(
            c.eye().distance(eye) < 1e-3,
            "eye() must reconstruct the seed eye, got {:?}",
            c.eye()
        );
        // The transform looks from the eye toward the focus.
        let t = c.transform();
        let to_focus = (focus - t.translation).normalize();
        assert!(t.forward().as_vec3().dot(to_focus) > 0.999);
    }

    #[test]
    fn zoom_clamps_to_min_radius() {
        let mut c = OrbitController {
            radius: 1.0,
            initialized: true,
            ..Default::default()
        };
        for _ in 0..100 {
            c.zoom(0.5);
        }
        assert!(c.radius >= MIN_RADIUS);
    }

    #[test]
    fn pitch_stays_off_the_poles() {
        let mut c = OrbitController {
            radius: 3.0,
            initialized: true,
            ..Default::default()
        };
        for _ in 0..1000 {
            c.orbit(0.0, 1000.0);
        }
        assert!(c.pitch.abs() <= MAX_PITCH);
        for _ in 0..1000 {
            c.orbit(0.0, -1000.0);
        }
        assert!(c.pitch.abs() <= MAX_PITCH);
    }

    #[test]
    fn pan_shifts_focus_in_screen_plane() {
        let focus = Vec3::ZERO;
        let eye = Vec3::new(0.0, 0.0, 5.0);
        let mut c = OrbitController::default();
        c.aim_from_eye(focus, eye);
        c.pan(10.0, 0.0);
        // Panning right moves the focus along the camera's -right (the world
        // slides the other way), staying in the screen plane (no depth shift).
        assert!(c.focus.x.abs() > 1e-4);
        assert!((c.focus.z - focus.z).abs() < 1e-4);
    }
}
