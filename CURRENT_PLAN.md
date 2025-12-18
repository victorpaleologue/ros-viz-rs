# Current Plan

Purpose: keep a living checklist for pausing/resuming work.

## Current objective

Refactor code architecture to separate reusable visualization core from application-specific code.

## Completed (Milestone 4)

- [x] Enable render, ros, and urdf features by default for easy `cargo run`
- [x] Switch from app.update() to app.run() so GUI stays open with continuous updates
- [x] Add 3D geometry (boxes for links, cylinders for joints) to scene entities
- [x] Implement automatic animation with sine-wave joint motion
- [x] Test GUI with animated 3D scene and ROS connection (domain 0)
- [x] Enhanced URDF parsing to extract joint axes, origins, parent/child relationships
- [x] Built proper kinematic tree from URDF with parent-child transforms
- [x] Use URDF joint axis data to orient joint cylinders correctly
- [x] Position links relative to their parent joints using URDF origin data
- [x] Converted from ROS publisher with emulator to ROS subscriber architecture
- [x] Connect to external ROS device and subscribe to /robot_description and /joint_states
- [x] Fixed JointStateMsg structure to match full ROS2 sensor_msgs/JointState
- [x] Added TransientLocal QoS durability for latched /robot_description topic
- [x] Successfully receive and parse NAO robot URDF (83 links, 82 joints)
- [x] Created standalone test_robot_description.rs example tool
- [x] Cleaned git history of PNG test images
- [x] Documented architecture principles in AGENT.md
- [x] Created test URDF samples: simple_arm, two_link_planar, triple_pendulum
- [x] Built test_urdf.rs example tool for URDF visualization testing
- [x] Successfully tested all 3 URDF samples with image export

## Next up (Architecture Refactoring)

- [ ] Create src/lib.rs with public API for visualization core
- [ ] Extract URDF parsing, scene building, and joint updates into src/visualization/ module
- [ ] Refactor src/app/mod.rs to consume the core visualization module
- [ ] Ensure core module has no ROS dependencies (pure URDF + Bevy)
- [ ] Update test_urdf.rs to use refactored core module
- [ ] Verify both ROS client app and test tool work with new architecture
- [ ] Visual review of exported test images against URDF samples
- [ ] Add command-line options to test_urdf for joint positions

## Upcoming milestones

- Milestone 0: ✓ Scaffolding + CLI + CI green
- Milestone 1: ✓ ROS2 connection layer with emulator
- Milestone 2: ✓ URDF ingestion and scene construction
- Milestone 3: ✓ Live 3D visualization with animated geometry and ROS connection
- Milestone 4: ✓ Proper URDF kinematic tree with accurate joint/link transforms
- Milestone 5: ✓ External ROS device integration with proper QoS settings
- Milestone 6 (current): Architecture refactoring - separate core visualization from applications
- Milestone 7: Offscreen rendering and automated visual comparison tests
- Milestone 8: Interactive joint manipulation and bidirectional ROS communication

## Notes

- Keep design decisions in docs/wiki/DesignDecisions.md up to date when architecture choices evolve.
- Keep ROS2 dependencies isolated (feature flags) to avoid CI breakage without ROS runtime until emulator is ready.
