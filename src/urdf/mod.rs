#[derive(Debug, Clone, PartialEq)]
pub struct UrdfScene {
    pub joints: Vec<String>,
    pub links: Vec<String>,
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
    let links = model.links.iter().map(|l| l.name.clone()).collect();
    let joints = model.joints.iter().map(|j| j.name.clone()).collect();
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

    const BOX_BOT: &str = include_str!("../../assets/tests/urdf/box_bot.urdf");

    #[test]
    fn parses_fixture_counts() {
        let scene = parse_urdf(BOX_BOT).expect("parse succeeds");
        assert_eq!(scene.link_count(), 2);
        assert_eq!(scene.joint_count(), 1);
        assert!(scene.links.contains(&"base_link".to_string()));
        assert!(scene.joints.contains(&"shoulder".to_string()));
    }
}
