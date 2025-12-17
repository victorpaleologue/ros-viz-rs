# URDF Ingestion Plan

## Goals

- Load URDF from `/robot_description` (text) or file for tests.
- Build a kinematic tree using `urdf-rs` + `k` + `nalgebra`.
- Load meshes via `mesh-loader` (.obj/.stl/.dae) and convert to Bevy meshes/materials.
- Provide a scene builder that outputs Bevy entities with transforms per joint.

## Approach

- Parsing: use `urdf-rs` to parse XML into a model; wrap in a light `RobotModel` struct.
- Kinematics: construct `k::Chain` from the URDF joints; store joint limits for validation.
- Mesh loading: `mesh-loader` for geometry; fallback to Bevy primitive cubes when mesh missing.
- Scene graph: create Bevy entities per link with `Transform` matching joint origins; attach meshes as children.
- Updates: apply joint positions from `/joint_states` to the kinematic chain, then write transforms into Bevy components.
- Features: keep the code behind a `urdf` feature until dependencies are added to avoid CI issues; provide stub behavior when feature is off.

## Fixtures

- Place test URDFs and meshes under `assets/tests/urdf/` (e.g., `box_bot.urdf`, `box_link.obj`).
- Provide a helper that loads fixture URDFs for integration tests without ROS.

## Testing

- Unit tests on parsing helpers with fixture URDF strings.
- Integration test: load fixture URDF, spawn Bevy app headless, and assert entity counts and transforms.
- Later: golden image comparison after applying joint updates.
