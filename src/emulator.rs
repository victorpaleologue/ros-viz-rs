use bevy::prelude::Resource;
use std::collections::BTreeMap;
use std::sync::{Arc, Mutex};

#[derive(Debug, Clone)]
pub struct EmulatorConfig {
    pub domain_id: u32,
    pub robot_name: String,
    pub urdf_xml: String,
}

#[derive(Debug, Clone, PartialEq)]
pub struct JointStateSnapshot {
    pub names: Vec<String>,
    pub positions: Vec<f64>,
}

#[derive(Debug, Default)]
struct EmulatorState {
    urdf_xml: String,
    joints: BTreeMap<String, f64>,
    time: f64,
}

#[derive(Debug, Clone, Resource)]
pub struct Emulator {
    state: Arc<Mutex<EmulatorState>>,
}

impl Emulator {
    pub fn start(config: EmulatorConfig, initial_joints: BTreeMap<String, f64>) -> Self {
        let state = EmulatorState {
            urdf_xml: config.urdf_xml,
            joints: initial_joints,
            time: 0.0,
        };
        Self {
            state: Arc::new(Mutex::new(state)),
        }
    }

    /// Returns the current URDF string that would be published on `/robot_description`.
    pub fn robot_description(&self) -> String {
        let guard = self.state.lock().expect("state poisoned");
        guard.urdf_xml.clone()
    }

    /// Applies joint command updates (e.g., from `/joint_commands`).
    pub fn apply_joint_commands(&self, updates: impl IntoIterator<Item = (String, f64)>) {
        let mut guard = self.state.lock().expect("state poisoned");
        for (name, value) in updates {
            guard.joints.insert(name, value);
        }
    }

    /// Produces a deterministic snapshot analogous to a `/joint_states` message (sorted by joint name).
    pub fn joint_state_snapshot(&self) -> JointStateSnapshot {
        let guard = self.state.lock().expect("state poisoned");
        let mut names: Vec<String> = guard.joints.keys().cloned().collect();
        names.sort();
        let positions = names
            .iter()
            .map(|n| guard.joints.get(n).copied().unwrap_or(0.0))
            .collect();
        JointStateSnapshot { names, positions }
    }

    /// Updates internal time and animates joints with sine waves at different frequencies
    pub fn tick(&self, delta_time: f64) {
        let mut guard = self.state.lock().expect("state poisoned");
        guard.time += delta_time;

        // Animate each joint with sine wave at different frequencies
        let joint_names: Vec<String> = guard.joints.keys().cloned().collect();
        for (idx, name) in joint_names.iter().enumerate() {
            let freq = 0.5 + (idx as f64 * 0.3);
            let amplitude = 1.0;
            let value = amplitude * (guard.time * freq).sin();
            guard.joints.insert(name.clone(), value);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const URDF: &str = "<robot name=\"test\"></robot>";

    fn cfg() -> EmulatorConfig {
        EmulatorConfig {
            domain_id: 0,
            robot_name: "dummy".into(),
            urdf_xml: URDF.into(),
        }
    }

    #[test]
    fn description_roundtrip() {
        let emu = Emulator::start(cfg(), BTreeMap::new());
        assert_eq!(emu.robot_description(), URDF);
    }

    #[test]
    fn joint_state_snapshot_sorts() {
        let mut joints = BTreeMap::new();
        joints.insert("b_joint".to_string(), 1.0);
        joints.insert("a_joint".to_string(), 0.5);
        let emu = Emulator::start(cfg(), joints);
        let snap = emu.joint_state_snapshot();
        assert_eq!(snap.names, vec!["a_joint", "b_joint"]);
        assert_eq!(snap.positions, vec![0.5, 1.0]);
    }

    #[test]
    fn apply_commands_updates_state() {
        let emu = Emulator::start(cfg(), BTreeMap::new());
        emu.apply_joint_commands(vec![("elbow".to_string(), 0.7)]);
        let snap = emu.joint_state_snapshot();
        assert_eq!(snap.names, vec!["elbow"]);
        assert_eq!(snap.positions, vec![0.7]);
    }
}
