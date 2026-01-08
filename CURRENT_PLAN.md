# Current Plan

Purpose: keep a living checklist for pausing/resuming work.

## Current objective

Complete skeleton visualization with proper kinematic chain rendering.

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
