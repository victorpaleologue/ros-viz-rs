# Code Organization

Planned layout:

- src/main.rs: entrypoint wiring CLI, feature flags, and app bootstrapping.
- src/config.rs: CLI parsing (clap) and environment fallback for ROS domain and other settings.
- src/app/mod.rs: Bevy app builder, plugin registration, and egui integration when needed.
- src/ros/mod.rs (feature `ros`): ROS2 client setup, publishers/subscribers, domain configuration.
- src/urdf_loader/: URDF parsing, mesh loading, and Bevy asset conversion.
- src/emulator/: Test-focused robot emulator publishing `/robot_description` and `/joint_states` and receiving joint commands.
- tests/: Integration tests for CLI, emulator, headless rendering and image comparison.
- assets/: Dummy URDFs and meshes for tests.
