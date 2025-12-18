# Current Plan

Purpose: keep a living checklist for pausing/resuming work.

## Current objective

Complete skeleton visualization with proper kinematic chain rendering.

## Completed (Milestone 5 - Skeleton Visualization)

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
- [x] Merged AGENT.md into AGENTS.md with comprehensive guidelines
- [x] Documented principle: always favor updated libraries over downgrading
- [x] Created test URDF samples: simple_arm, two_link_planar, triple_pendulum, box_bot
- [x] Built test_urdf.rs example tool for URDF visualization testing
- [x] Successfully tested all URDF samples with image export
- [x] Created visual regression test infrastructure with ImageMagick integration
- [x] Generated reference images for all URDF samples
- [x] Automated integration tests verify rendering consistency (0% diff)
- [x] Made image output paths explicit and user-controlled
- [x] **Created src/visualization.rs module for reusable skeleton visualization**
- [x] **Extracted shared geometry, materials, and spawn functions**
- [x] **Refactored app, test_urdf, and tests to use visualization module**
- [x] **Updated dependencies: urdf-rs 0.9, k 0.32 (latest versions)**
- [x] **Fixed urdf-rs 0.9 API compatibility (Vec3 tuple struct access)**
- [x] **All visual regression tests passing with new skeleton code**
- [x] **NAO robot (83 links) successfully renders**

## Next up (Forward Kinematics Integration)

- [ ] Integrate k crate properly for forward kinematics
  - [ ] Parse URDF XML directly to urdf_rs::Robot (not through UrdfScene)
  - [ ] Build k::Chain from urdf_rs::Robot
  - [ ] Use k::urdf::link_to_joint_map for proper mapping
  - [ ] Compute world transforms with chain.update_transforms()
  - [ ] Convert nalgebra Isometry3 to Bevy Transform
- [ ] Handle joint state updates from /joint_states topic
  - [ ] Update k::Chain joint positions from ROS messages
  - [ ] Recompute transforms and update Bevy entities
- [ ] Fix broken unit tests (app::tests need updating for new architecture)
- [ ] Add coordinate axes and ground plane to visualization for better orientation

## Upcoming milestones

- Milestone 0: ✓ Scaffolding + CLI + CI green
- Milestone 1: ✓ ROS2 connection layer with emulator
- Milestone 2: ✓ URDF ingestion and scene construction
- Milestone 3: ✓ Live 3D visualization with animated geometry and ROS connection
- Milestone 4: ✓ Proper URDF kinematic tree with accurate joint/link transforms
- Milestone 5: ✓ External ROS device integration with proper QoS settings
- Milestone 6: ✓ Visual regression test infrastructure with automated verification
- Milestone 7 (current): Architecture refactoring - separate core visualization from applications
- Milestone 8: Interactive joint manipulation and bidirectional ROS communication

## Notes

- Keep design decisions in docs/wiki/DesignDecisions.md up to date when architecture choices evolve.
- Keep ROS2 dependencies isolated (feature flags) to avoid CI breakage without ROS runtime until emulator is ready.
