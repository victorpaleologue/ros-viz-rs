use std::collections::HashMap;

fn main() {
    let mut args = std::env::args().skip(1);
    let path = args.next().unwrap();
    let model = ros_viz_rs::robot::RobotModel::from_urdf_file(&path).unwrap();
    let mut joints = HashMap::new();
    for spec in args {
        let (name, value) = spec.split_once('=').unwrap();
        joints.insert(name.to_string(), value.parse::<f64>().unwrap());
    }
    model.set_joint_positions(&joints);
    let mut names: Vec<_> = model.link_world_transforms().into_iter().collect();
    names.sort_by(|a, b| a.0.cmp(&b.0));
    for (name, iso) in names {
        let t = iso.translation;
        println!("{name:24} ({:+.4}, {:+.4}, {:+.4})", t.x, t.y, t.z);
    }
}
