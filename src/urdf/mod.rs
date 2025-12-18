#[derive(Debug, Clone, PartialEq)]
pub struct JointInfo {
    pub name: String,
    pub parent: String,
    pub child: String,
    pub origin_xyz: [f64; 3],
    pub origin_rpy: [f64; 3],
    pub axis: [f64; 3],
    pub joint_type: String,
    pub limits: Option<(f64, f64)>, // (lower, upper)
}

#[derive(Debug, Clone, PartialEq)]
pub struct LinkInfo {
    pub name: String,
}

#[derive(Debug, Clone, PartialEq)]
pub struct UrdfScene {
    pub joints: Vec<JointInfo>,
    pub links: Vec<LinkInfo>,
}

impl UrdfScene {
    pub fn empty() -> Self {
        Self {
            joints: Vec::new(),
            links: Vec::new(),
        }
    }

    pub fn joint_count(&self) -> usize {
        self.joints.len()
    }

    pub fn link_count(&self) -> usize {
        self.links.len()
    }
}

/// Parse URDF text and build an internal scene representation.
/// When the `urdf` feature is disabled, returns an error to keep CI lightweight.
#[cfg(feature = "urdf")]
pub fn parse_urdf(xml: &str) -> anyhow::Result<UrdfScene> {
    let model = urdf_rs::read_from_string(xml)?;

    let links = model
        .links
        .iter()
        .map(|l| LinkInfo {
            name: l.name.clone(),
        })
        .collect();

    let joints = model
        .joints
        .iter()
        .map(|j| {
            let origin_xyz = j.origin.xyz.0;
            let origin_rpy = j.origin.rpy.0;
            let axis = j.axis.xyz.0;
            // JointLimit has lower and upper fields directly
            let limits = Some((j.limit.lower, j.limit.upper));

            JointInfo {
                name: j.name.clone(),
                parent: j.parent.link.clone(),
                child: j.child.link.clone(),
                origin_xyz,
                origin_rpy,
                axis,
                joint_type: format!("{:?}", j.joint_type),
                limits,
            }
        })
        .collect();

    Ok(UrdfScene { joints, links })
}

/// Parse URDF text and build an internal scene representation.
/// When the `urdf` feature is disabled, returns an error to keep CI lightweight.
#[cfg(not(feature = "urdf"))]
pub fn parse_urdf(_xml: &str) -> anyhow::Result<UrdfScene> {
    Err(anyhow::anyhow!(
        "Build with --features urdf to enable URDF parsing"
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stub_parse_returns_error() {
        let err = parse_urdf("<robot></robot>").unwrap_err();
        assert!(err.to_string().contains("URDF") || err.to_string().contains("urdf"));
    }

    #[test]
    fn empty_scene_helper() {
        let scene = UrdfScene::empty();
        assert_eq!(scene.joint_count(), 0);
        assert_eq!(scene.link_count(), 0);
    }
}

#[cfg(all(test, feature = "urdf"))]
mod feature_tests {
    use super::*;

    const BOX_BOT: &str = include_str!("../../test-data/urdf/box_bot.urdf");

    #[test]
    fn parses_fixture_counts() {
        let scene = parse_urdf(BOX_BOT).expect("parse succeeds");
        assert_eq!(scene.link_count(), 2);
        assert_eq!(scene.joint_count(), 1);
        assert!(scene.links.iter().any(|l| l.name == "base_link"));
        assert!(scene.joints.iter().any(|j| j.name == "shoulder"));
    }

    #[test]
    fn parses_joint_metadata() {
        let scene = parse_urdf(BOX_BOT).expect("parse succeeds");
        let shoulder = scene
            .joints
            .iter()
            .find(|j| j.name == "shoulder")
            .expect("shoulder joint");
        assert_eq!(shoulder.parent, "base_link");
        assert_eq!(shoulder.child, "tip_link");
        assert_eq!(shoulder.axis, [0.0, 0.0, 1.0]);
        assert!(shoulder.limits.is_some());
        let (lower, upper) = shoulder.limits.unwrap();
        assert!((lower - (-1.57)).abs() < 0.01);
        assert!((upper - 1.57).abs() < 0.01);
    }
}
