//! Print the world position of every link of a URDF after forward
//! kinematics, optionally with joint positions applied.
//!
//! ```bash
//! cargo run --example fk_probe -- robot.urdf shoulder_lift_joint=-1.57
//! ```

use std::collections::HashMap;

use anyhow::Context;

fn main() -> anyhow::Result<()> {
    let mut args = std::env::args().skip(1);
    let path = args.next().context("usage: fk_probe <urdf> [joint=rad]…")?;
    let model = ros_viz_rs::robot::RobotModel::from_urdf_file(&path)?;

    let mut joints = HashMap::new();
    for spec in args {
        let (name, value) = spec
            .split_once('=')
            .with_context(|| format!("expected joint=radians, got '{spec}'"))?;
        joints.insert(
            name.to_string(),
            value
                .parse::<f64>()
                .with_context(|| format!("invalid number in '{spec}'"))?,
        );
    }

    let mut links: Vec<_> = model.pose_transforms(&joints).into_iter().collect();
    links.sort_by(|a, b| a.0.cmp(&b.0));
    for (name, iso) in links {
        let t = iso.translation;
        println!("{name:24} ({:+.4}, {:+.4}, {:+.4})", t.x, t.y, t.z);
    }
    Ok(())
}
