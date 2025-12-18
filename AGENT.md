# AGENT NOTES

## Architecture Principles

**Core Visualization Module**: The 3D visualization logic (URDF parsing, scene building, joint transforms) must be in a reusable module, NOT coupled to any specific application. Multiple tools should be able to use the same core:

- ROS2 client app (subscribes to /robot_description and /joint_states)
- URDF test tool (loads URDF files from disk, exports images)
- Future tools (motion planning, trajectory visualization, etc.)

**Image Export Strategy**:

- Tools should require explicit output paths for image exports
- If output path is omitted, default to current working directory with descriptive filename
- Do not auto-export to other directories.
- When you run these tools, provide output path in a temporary directory, or a hidden one that is ignored by git.
- Do not commit test output images to the repository, unless they are part of a documented test case with expected results.

**Application Structure**:

```text
src/
  lib.rs          - Core visualization module (public API)
  urdf/           - URDF parsing
  visualization/  - Bevy systems for 3D rendering and joint updates
  app/            - ROS2 client application (one of many possible apps)
examples/
  test_urdf.rs    - Standalone URDF testing tool
  test_robot_description.rs - ROS topic tester
```

## Expectations from the user

- Maintain CURRENT_PLAN.md with actionable next steps and progress tracking; update as work evolves.
- Commit regularly at milestones where a measurable result is achieved and covered by a test (preferably `cargo test`).
- Use Bevy for the core 3D view; if auxiliary GUI is needed, use egui via bevy_egui.
- Use ROS2 through the `ros2-client` crate; domain selection comes from CLI arg or `ROS_DOMAIN_ID`.
- Build a ROS robot emulator for tests (publish /robot_description and /joint_states; accept joint updates).
- Provide tooling to render a frame to an image for manual/automated checks.
- Add standard CI (GitHub Actions) running fmt, clippy, and tests.
- README must explain the tool, how to run it, and common recipes.
- Maintain a wiki-style docs folder (contributing, code organization, design decisions) and keep design decisions updated.
- Mention official documentation consulted in comments/design notes when relevant.

Behavior guardrails:

- Plan ahead to avoid local traps; support pausing/resuming by keeping plan and docs fresh.
- Do not use `structopt` or `kiss3d`; prefer `clap` and Bevy.
- Keep commits scoped to milestones; avoid mixing unrelated changes.
- Favor tests that can run with `cargo test`; avoid flaky or environment-heavy steps when possible.
- Keep ASCII unless non-ASCII is justified by existing file content.
- Unless you stumble on a problem, go on with the plan automatically. Do not stop at every step.
- Produce Markdown that is valid according to the `markdownlint` VSCode extension (proper list formatting, blank lines around lists, no trailing spaces, etc.).

Pitfalls to watch for:

- Forgetting to update CURRENT_PLAN.md or design decisions when plans change.
- Coupling ROS2 runtime to builds in a way that breaks CI; isolate external deps behind features when possible.
- Skipping image-render test tooling or emulator scaffolding.
- Neglecting CLI domain handling (argument + env fallback) or failing to surface errors clearly.

When corrected by the user, summarize the feedback and adjustments here to stay aligned.
