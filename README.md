# ros-viz-rs

ROS2 robot visualizer powered by Bevy. It subscribes to `/robot_description` to build a 3D model from URDF and listens to `/joint_states` to animate the robot in real time. A built-in emulator and image renderer will support automated testing.

## Status

Early scaffolding. CLI parsing, docs, and CI are being set up; ROS2 + Bevy functionality is forthcoming.

## Running (planned)

- Prereqs: Rust stable (cargo), ROS2 runtime for real robots; Bevy runs with wgpu (Vulkan/Metal/DirectX)
- CLI (planned): `cargo run -- --domain <id>` or set `ROS_DOMAIN_ID`. Defaults to env when flag is absent.
- Features: a `ros` feature will enable ROS2 networking; tests/emulator will work without a live ROS graph.

## Development workflow

- `cargo fmt`, `cargo clippy -- -D warnings`, `cargo test`
- CI mirrors these steps in GitHub Actions.
- Keep CURRENT_PLAN.md and docs/wiki/DesignDecisions.md up to date as architecture evolves.

## Documentation

- Expectations and guardrails: AGENT.md
- Active plan: CURRENT_PLAN.md
- Wiki-style docs live under docs/wiki (contributing, code organization, design decisions).

## Roadmap (abridged)

- Milestone 0: scaffolding + CLI + CI green
- Milestone 1: ROS2 domain-configurable connection layer with emulator harness
- Milestone 2: URDF ingestion + Bevy scene build with sample assets
- Milestone 3: Joint animation + image capture tooling and tests
