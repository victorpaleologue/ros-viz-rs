#[derive(Debug, Clone, PartialEq)]
pub struct UrdfScene {
    pub joints: usize,
    pub links: usize,
}

impl UrdfScene {
    pub fn empty() -> Self {
        Self {
            joints: 0,
            links: 0,
        }
    }
}

/// Parse URDF text and build an internal scene representation.
/// Currently a stub: real implementation will use `urdf-rs` + `k` + `nalgebra` + `mesh-loader`.
pub fn parse_urdf(_xml: &str) -> anyhow::Result<UrdfScene> {
    Err(anyhow::anyhow!("URDF parsing not implemented yet"))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stub_parse_returns_error() {
        let err = parse_urdf("<robot></robot>").unwrap_err();
        assert!(err.to_string().contains("URDF"));
    }

    #[test]
    fn empty_scene_helper() {
        let scene = UrdfScene::empty();
        assert_eq!(scene.joints, 0);
        assert_eq!(scene.links, 0);
    }
}
