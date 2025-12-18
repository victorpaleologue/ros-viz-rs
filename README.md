# ros-viz-rs

ROS2 robot visualizer powered by Bevy. It subscribes to `/robot_description` to build a 3D model from URDF and listens to `/joint_states` to animate the robot in real time.

## Features

- ✅ URDF parsing with full kinematic tree extraction
- ✅ 3D visualization with Bevy (proper joint hierarchy and transforms)
- ✅ ROS2 subscriber client (connects to `/robot_description` and `/joint_states`)
- ✅ Standalone URDF testing tool with image export

## Quick Start

### ROS2 Visualization

Connect to a running ROS2 robot:

```bash
cargo run
```

By default, connects to ROS domain 0. Override with `--domain <id>` or `ROS_DOMAIN_ID` environment variable.

### URDF Testing

Test URDF parsing and visualization without ROS:

```bash
# Test with provided samples
cargo run --example urdf_view_snapshot test-data/urdf/simple_arm.urdf
cargo run --example urdf_view_snapshot test-data/urdf/two_link_planar.urdf
cargo run --example urdf_view_snapshot test-data/urdf/triple_pendulum.urdf

# Test with your own URDF
cargo run --example urdf_view_snapshot path/to/your/robot.urdf
```

Images are exported to your system temp directory for visual inspection. See [examples/README.md](examples/README.md) for details.

## Test Data

Sample URDF files for testing are in [test-data/urdf/](test-data/urdf/). See that directory's README for descriptions of each sample and expected visualizations.

## Development workflow

- `cargo fmt`, `cargo clippy -- -D warnings`, `cargo test`
- CI mirrors these steps in GitHub Actions.
- Keep CURRENT_PLAN.md and docs/wiki/DesignDecisions.md up to date as architecture evolves.

## Documentation

- Expectations and guardrails: AGENT.md
- Active plan: CURRENT_PLAN.md
- Wiki-style docs live under docs/wiki (contributing, code organization, design decisions).

## Architecture

The project is being refactored to separate concerns:

- **Core visualization module** (`src/visualization/`): Reusable URDF parsing, 3D scene building, and joint transforms (no ROS dependencies)
- **Applications**: ROS2 client, URDF test tool, future planning tools
- **Examples**: Standalone tools demonstrating specific functionality

See [AGENT.md](AGENT.md) for detailed architecture principles and [CURRENT_PLAN.md](CURRENT_PLAN.md) for current development status.

## Roadmap

- ✅ Milestone 0: Scaffolding + CLI + CI green
- ✅ Milestone 1: ROS2 connection layer with domain configuration
- ✅ Milestone 2: URDF ingestion with full kinematic data extraction
- ✅ Milestone 3: Live 3D visualization with proper joint hierarchy
- ✅ Milestone 4: Accurate kinematic tree with URDF transforms
- ✅ Milestone 5: External ROS device integration (proper QoS, latched topics)
- ⏳ Milestone 6: Architecture refactoring (core visualization module)
- 🔮 Milestone 7: Automated visual comparison tests
- 🔮 Milestone 8: Interactive joint manipulation and bidirectional ROS
