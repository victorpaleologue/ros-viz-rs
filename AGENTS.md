# Agent Guidelines for ros-viz-rs

## Dependency Management

**Always favor updated libraries rather than downgrading dependencies.**

When encountering dependency conflicts:

1. First check if newer versions of all dependencies are available
2. Update the conflicting dependency to a compatible newer version
3. Only consider downgrading as a last resort if no updated versions exist
4. Document why if downgrading is truly necessary

This ensures we benefit from:

- Latest bug fixes and security patches
- New features and performance improvements
- Better long-term maintainability
- Ecosystem compatibility

## Architecture Principles

**Core Visualization Module**: The 3D visualization logic (URDF parsing, scene building, joint transforms) must be in a reusable module, NOT coupled to any specific application. Multiple tools should be able to use the same core:

- ROS2 client app (subscribes to /robot_description and /joint_states)
- URDF test tool (loads URDF files from disk, exports images)
- Future tools (motion planning, trajectory visualization, etc.)

**Image Export Strategy**:

- Tools should require explicit output paths for image exports
- If output path is omitted, default to current working directory with descriptive filename
- Do not auto-export to other directories
- When you run these tools, provide output path in a temporary directory, or a hidden one that is ignored by git
- Do not commit test output images to the repository, unless they are part of a documented test case with expected results

**Application Structure**:

```text
src/
  lib.rs          - Core visualization module (public API)
  urdf/           - URDF parsing
  visualization/  - Bevy systems for 3D rendering and joint updates
  app/            - ROS2 client application (one of many possible apps)
examples/
  urdf_view.rs      - Standalone URDF viewer and snapshot tool
  get_robot_description.rs - Get a ROS robot description from topic `/robot_description`. Useful for sanity check too.
```

## Expectations from the User

- Maintain CURRENT_PLAN.md with actionable next steps and progress tracking; update as work evolves
- Commit regularly at milestones where a measurable result is achieved and covered by a test (preferably `cargo test`)
- Use Bevy for the core 3D view; if auxiliary GUI is needed, use egui via bevy_egui
- Use ROS2 through the `ros2-client` crate; domain selection comes from CLI arg or `ROS_DOMAIN_ID`
- Build a ROS robot emulator for tests (publish /robot_description and /joint_states; accept joint updates)
- Provide tooling to render a frame to an image for manual/automated checks
- Add standard CI (GitHub Actions) running fmt, clippy, and tests
- README must explain the tool, how to run it, and common recipes
- Maintain a wiki-style docs folder (contributing, code organization, design decisions) and keep design decisions updated
- Mention official documentation consulted in comments/design notes when relevant

## Development Principles

- Be autonomous: follow CURRENT_PLAN.md and make progress independently
- Plan ahead to avoid local traps; support pausing/resuming by keeping plan and docs fresh
- Unless you stumble on a problem, go on with the plan automatically. Do not stop at every step
- Test thoroughly before presenting results to the user: that means writing tests, running them with `cargo test`, and validating outputs (e.g. images) manually when needed.
- Use visual regression tests to validate skeleton rendering
- Keep code factorized and reusable (see src/visualization.rs module)
- Do not use `structopt` or `kiss3d`; prefer `clap` and Bevy
- Keep commits scoped to milestones; avoid mixing unrelated changes
- Favor tests that can run with `cargo test`; avoid flaky or environment-heavy steps when possible
- Keep ASCII unless non-ASCII is justified by existing file content
- Produce Markdown that is valid according to the `markdownlint` VSCode extension (proper list formatting, blank lines around lists, no trailing spaces, etc.)

## Pitfalls to Watch For

- Forgetting to update CURRENT_PLAN.md or design decisions when plans change
- Coupling ROS2 runtime to builds in a way that breaks CI; isolate external deps behind features when possible
- Skipping image-render test tooling or emulator scaffolding
- Neglecting CLI domain handling (argument + env fallback) or failing to surface errors clearly

## Current Architecture Status

- `src/visualization.rs`: Shared robot visualization code (geometry, materials, spawning)
- `src/app/mod.rs`: Main ROS2 application
- `examples/urdf_view.rs`: Standalone URDF viewer and snapshot tool
- `tests/visual_regression.rs`: Automated visual testing with ImageMagick

## Corrections and Adjustments

When corrected by the user, summarize the feedback and adjustments here to stay aligned.
