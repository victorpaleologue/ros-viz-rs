# Design Decisions

- Bevy is the core engine for 3D rendering; egui will be layered via bevy_egui only when native widgets are needed.
- Command-line parsing uses clap (no structopt) with explicit ROS domain selection via `--domain` overriding `ROS_DOMAIN_ID`.
- ROS2 networking will be isolated behind a `ros` Cargo feature to keep CI green without a ROS runtime; emulator-based tests will run without that feature.
- When the `ros` feature is absent, ROS connectivity surfaces a clear error to keep CI runnable; real transport will live behind the feature-gated path.
- URDF parsing will rely on `urdf-rs`, `k`, and `nalgebra`; mesh loading will use `mesh-loader`. Bevy meshes will be generated from these outputs.
- A lightweight robot emulator will publish `/robot_description` and `/joint_states`, and accept joint updates for tests.
- Image rendering will produce offscreen frames to files to support manual inspection and automated comparisons.
- Tests aim to be runnable with `cargo test` using the emulator and offscreen rendering; integration tests will drive the Bevy app headless where possible.
