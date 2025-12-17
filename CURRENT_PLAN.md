# Current Plan

Purpose: keep a living checklist for pausing/resuming work.

## Current objective

Bootstrap the project with clear docs, CI, and a minimal runnable crate scaffolding that already parses ROS domain CLI args and is ready for ROS2 + Bevy integration.

## Near-term steps

- [x] Add project scaffolding docs (AGENT, README, wiki-style files) and initial design decisions.
- [x] Set up crate structure with CLI domain selection (CLI flag + ROS_DOMAIN_ID fallback) and placeholder Bevy/ROS2 wiring behind features.
- [x] Add CI workflow (fmt, clippy, test) and ensure `cargo test` passes locally.
- [x] Add first test around config parsing to validate the CLI surface.
- [x] Sketch ROS2 connection module layout and begin stub subscriber/publisher interfaces (feature `ros`).
- [x] Outline emulator scaffolding and fixtures (dummy URDF + joint stream) under tests/ and assets/.
- [x] Plan Bevy scene bootstrap and offscreen image capture pathway.

Next up:

- [x] Add rendering config (headless/windowed, resolution) to CLI/config and Bevy builder skeleton.
- [x] Stub Bevy app builder with headless toggle and placeholder systems; keep tests headless-capable.
- [x] Draft URDF ingest/mesh loading API surface and fixture layout (assets/ + tests/ helpers).
- [x] Plan emulator implementation details (topics/messages, joint update handling, timing) and test harness hooks.

Next up:

- [x] Implement URDF parsing pipeline behind feature flag and add first fixture assets.
- [x] Implement emulator message flow (publish robot_description/joint_states, accept joint commands) with a basic in-process test harness.

Next up:

- [x] Wire emulator and URDF scene into Bevy app (load URDF, spawn scene placeholders).
- [x] Add ROS feature hooks to emulator (actual topic publish/subscribe via ros2-client) while keeping CI stubs intact.

Next up:

- [x] Add ROS loopback test coverage under `ros` feature (joint_commands publish/receive, joint_states roundtrip).
- [ ] Wire rendering pipeline and image capture harness.

## Upcoming milestones

- Milestone 0: Scaffolding + CLI + CI green (initial commit target).
- Milestone 1: ROS2 connection layer with domain selection and subscription stubs plus emulator harness.
- Milestone 2: URDF ingestion and Bevy scene construction with sample assets.
- Milestone 3: Live joint state updates and image capture tool + automated comparison test.

## Notes

- Keep design decisions in docs/wiki/DesignDecisions.md up to date when architecture choices evolve.
- Keep ROS2 dependencies isolated (feature flags) to avoid CI breakage without ROS runtime until emulator is ready.
