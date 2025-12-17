# Emulator Plan

## Goals

- Provide a Rust ROS2-based emulator to publish `/robot_description` and `/joint_states` for tests and demos.
- Accept joint command updates to move the emulated robot state.
- Operate without external ROS graph dependencies when using the `ros` feature; fallback stubs otherwise.

## Approach

- Topics:
  - `/robot_description`: latched publication of URDF text.
  - `/joint_states`: periodic joint state publication from internal state.
  - `/joint_commands` (proposed): subscribe to joint targets to update internal state.
- Node config: domain comes from `AppConfig`; node name `ros_viz_emulator`; namespace optional.
- Timing: simple fixed-rate timer (e.g., 30 Hz) for joint_states publishing; allow immediate publish on state changes.
- Test harness: helper to spin emulator in-process for integration tests; provide deterministic seeds and fixed timestep.

## Fixtures

- Store small URDFs/meshes under `assets/tests/urdf/` shared with the URDF pipeline.
- Provide a default "box bot" with 1-2 joints for fast tests.

## Testing

- Unit: validate state update logic and topic names.
- Integration: start emulator (feature `ros`), subscribe via client stub or mock to verify messages; for CI without ROS, provide stub that returns clear errors.
