# Current Plan

Purpose: keep a living checklist for pausing/resuming work.

## Current objective

Enhance URDF parsing to build proper kinematic tree and improve ROS integration for live robot visualization.

## Completed (Milestone 3)

- [x] Enable render, ros, and urdf features by default for easy `cargo run`
- [x] Switch from app.update() to app.run() so GUI stays open with continuous updates
- [x] Add 3D geometry (boxes for links, cylinders for joints) to scene entities
- [x] Implement automatic animation with sine-wave joint motion
- [x] Test GUI with animated 3D scene and ROS connection (domain 0)

## Next up (URDF & ROS focus)

- [ ] Enhance URDF parsing to extract joint axes, origins, parent/child relationships
- [ ] Build proper kinematic tree from URDF with parent-child transforms
- [ ] Use URDF joint axis data to orient joint cylinders correctly
- [ ] Position links relative to their parent joints using URDF origin data
- [ ] Verify ROS integration is publishing robot_description and joint_states correctly
- [ ] Test subscribing to external joint_commands from ROS topic
- [ ] Add CLI option to load custom URDF file path instead of using default
- [ ] Test with actual robot URDF (e.g., a simple arm or the robot from your ROS system)

## Upcoming milestones

- Milestone 0: ✓ Scaffolding + CLI + CI green
- Milestone 1: ✓ ROS2 connection layer with emulator
- Milestone 2: ✓ URDF ingestion and scene construction
- Milestone 3: ✓ Live 3D visualization with animated geometry and ROS connection
- Milestone 4 (current): Proper URDF kinematic tree with accurate joint/link transforms
- Milestone 5: External ROS device integration and bidirectional communication
- Milestone 6: Offscreen deterministic rendering and automated visual comparison tests

## Notes

- Keep design decisions in docs/wiki/DesignDecisions.md up to date when architecture choices evolve.
- Keep ROS2 dependencies isolated (feature flags) to avoid CI breakage without ROS runtime until emulator is ready.
