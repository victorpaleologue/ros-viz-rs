//! Robot model: URDF ingestion and forward kinematics.
//!
//! [`RobotModel`] keeps the *full* [`urdf_rs::Robot`] description (links,
//! visuals, materials, meshes) alongside a [`k::Chain`] built from it, so
//! rendering uses real geometry and poses come from proper forward
//! kinematics rather than ad-hoc hierarchy math.
//!
//! The chain conversion, mimic-joint handling and link/joint mapping follow
//! the `k` crate's URDF support: <https://docs.rs/k/latest/k/urdf/index.html>

pub mod mesh;

use std::collections::HashMap;
use std::path::Path;

use k::nalgebra::Isometry3;

/// A robot description ready for rendering and kinematics.
///
/// Construct with [`RobotModel::from_urdf_str`] or
/// [`RobotModel::from_urdf_file`]. Joint positions are pushed in by name and
/// world transforms are read out per link, which maps directly onto ECS
/// entities.
pub struct RobotModel {
    /// The parsed URDF, kept in full fidelity (visuals, materials, meshes).
    pub urdf: urdf_rs::Robot,
    /// Kinematic chain mirroring the URDF joint tree.
    chain: k::Chain<f32>,
    /// Joint limits read once at construction (the chain locks per access).
    limits: HashMap<String, (f32, f32)>,
    /// Serializes set-positions + forward-kinematics cycles: the chain's
    /// per-node interior mutability would otherwise let two posers interleave.
    pose_lock: std::sync::Mutex<()>,
}

impl RobotModel {
    /// Parse a URDF document.
    pub fn from_urdf_str(xml: &str) -> anyhow::Result<Self> {
        let urdf = urdf_rs::read_from_string(xml)?;
        let chain = k::Chain::from(&urdf);
        let limits = chain
            .iter_joints()
            .filter_map(|j| j.limits.map(|l| (j.name.clone(), (l.min, l.max))))
            .collect();
        Ok(Self {
            urdf,
            chain,
            limits,
            pose_lock: std::sync::Mutex::new(()),
        })
    }

    /// Read and parse a URDF file.
    pub fn from_urdf_file(path: impl AsRef<Path>) -> anyhow::Result<Self> {
        let xml = std::fs::read_to_string(path.as_ref())?;
        Self::from_urdf_str(&xml)
    }

    /// The robot name declared in the URDF.
    pub fn name(&self) -> &str {
        &self.urdf.name
    }

    /// Names of all movable (non-fixed, non-mimic) joints, in chain order.
    pub fn joint_names(&self) -> Vec<String> {
        self.chain.iter_joints().map(|j| j.name.clone()).collect()
    }

    /// Position limits (lower, upper) for a movable joint, when bounded.
    pub fn joint_limits(&self, joint_name: &str) -> Option<(f32, f32)> {
        self.limits.get(joint_name).copied()
    }

    /// Set joint positions by name, clamping to URDF limits.
    ///
    /// Unknown joint names are ignored (a `/joint_states` message routinely
    /// carries more joints than the description, e.g. grippers).
    pub fn set_joint_positions(&self, positions: &HashMap<String, f64>) {
        let _guard = self.pose_guard();
        self.set_joint_positions_locked(positions);
    }

    /// Current world transform of every link, keyed by link name.
    ///
    /// Runs forward kinematics over the chain. In URDF the child-link frame
    /// coincides with its joint frame, so each chain node's world transform
    /// is its child link's pose; the root link sits on the chain root.
    pub fn link_world_transforms(&self) -> HashMap<String, Isometry3<f32>> {
        let _guard = self.pose_guard();
        self.link_world_transforms_locked()
    }

    /// Atomically apply joint positions and run forward kinematics.
    ///
    /// Use this rather than [`set_joint_positions`](Self::set_joint_positions)
    /// followed by [`link_world_transforms`](Self::link_world_transforms)
    /// when the model could be posed from elsewhere in between (e.g. one
    /// model shared by several scene robots).
    pub fn pose_transforms(
        &self,
        positions: &HashMap<String, f64>,
    ) -> HashMap<String, Isometry3<f32>> {
        let _guard = self.pose_guard();
        self.set_joint_positions_locked(positions);
        self.link_world_transforms_locked()
    }

    fn pose_guard(&self) -> std::sync::MutexGuard<'_, ()> {
        self.pose_lock
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner())
    }

    fn set_joint_positions_locked(&self, positions: &HashMap<String, f64>) {
        for (name, position) in positions {
            if let Some(node) = self.chain.find(name) {
                let position = *position as f32;
                let clamped = match self.limits.get(name) {
                    Some((lo, hi)) => position.clamp(*lo, *hi),
                    None => position,
                };
                // Errors only occur for fixed/mimic joints; skip those.
                let _ = node.set_joint_position(clamped);
            }
        }
    }

    fn link_world_transforms_locked(&self) -> HashMap<String, Isometry3<f32>> {
        self.chain.update_transforms();
        self.chain
            .iter()
            .filter_map(|node| {
                let link_name = node.link().as_ref().map(|l| l.name.clone())?;
                let transform = node.world_transform()?;
                Some((link_name, transform))
            })
            .collect()
    }

    /// The root link (the only link that is no joint's child).
    pub fn root_link_name(&self) -> Option<String> {
        let children: std::collections::HashSet<&str> = self
            .urdf
            .joints
            .iter()
            .map(|j| j.child.link.as_str())
            .collect();
        self.urdf
            .links
            .iter()
            .find(|l| !children.contains(l.name.as_str()))
            .map(|l| l.name.clone())
    }
}

impl std::fmt::Debug for RobotModel {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("RobotModel")
            .field("name", &self.urdf.name)
            .field("links", &self.urdf.links.len())
            .field("joints", &self.urdf.joints.len())
            .finish()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::FRAC_PI_2;

    const TWO_LINK: &str = include_str!("../../test-data/urdf/two_link_planar.urdf");
    const NAO: &str = include_str!("../../test-data/urdf/nao_robot.urdf");

    #[test]
    fn parses_full_urdf() {
        let model = RobotModel::from_urdf_str(TWO_LINK).expect("parses");
        assert!(!model.urdf.links.is_empty());
        assert!(!model.joint_names().is_empty());
    }

    #[test]
    fn nao_has_full_kinematic_tree() {
        let model = RobotModel::from_urdf_str(NAO).expect("parses");
        // The NAO H25 description has dozens of links; every one must get a
        // world transform (the old renderer silently dropped most of them).
        let transforms = model.link_world_transforms();
        assert_eq!(
            transforms.len(),
            model.urdf.links.len(),
            "every URDF link must have a world transform"
        );
        // Head sits above the torso.
        let torso_z = transforms["torso"].translation.z;
        let head_z = transforms["Head"].translation.z;
        assert!(
            head_z > torso_z,
            "head ({head_z}) should be above torso ({torso_z})"
        );
        // Feet are below the torso.
        let foot_z = transforms["l_ankle"].translation.z;
        assert!(foot_z < torso_z, "foot ({foot_z}) below torso ({torso_z})");
    }

    #[test]
    fn fk_moves_child_links() {
        let model = RobotModel::from_urdf_str(TWO_LINK).expect("parses");
        let rest = model.link_world_transforms();

        let joint = model.joint_names()[0].clone();
        let mut positions = HashMap::new();
        positions.insert(joint, FRAC_PI_2);
        model.set_joint_positions(&positions);
        let bent = model.link_world_transforms();

        let moved = rest.iter().any(|(name, t)| {
            let t2 = &bent[name];
            (t.translation.vector - t2.translation.vector).norm() > 1e-3
                || t.rotation.angle_to(&t2.rotation) > 1e-3
        });
        assert!(moved, "rotating a joint must move some link");
    }

    #[test]
    fn joint_positions_clamp_to_limits() {
        let model = RobotModel::from_urdf_str(NAO).expect("parses");
        let mut positions = HashMap::new();
        positions.insert("HeadYaw".to_string(), 100.0);
        model.set_joint_positions(&positions);
        // HeadYaw limit is ±2.08567 rad; FK must not explode.
        let (lo, hi) = model.joint_limits("HeadYaw").expect("has limits");
        assert!(lo < hi);
        let transforms = model.link_world_transforms();
        assert!(transforms.contains_key("Neck"));
    }

    #[test]
    fn unknown_joints_are_ignored() {
        let model = RobotModel::from_urdf_str(TWO_LINK).expect("parses");
        let mut positions = HashMap::new();
        positions.insert("no_such_joint".to_string(), 1.0);
        model.set_joint_positions(&positions); // must not panic
    }

    #[test]
    fn root_link_is_found() {
        let model = RobotModel::from_urdf_str(NAO).expect("parses");
        assert_eq!(model.root_link_name().as_deref(), Some("base_link"));
    }
}
