# AGENT NOTES

Expectations from the user:

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

Pitfalls to watch for:

- Forgetting to update CURRENT_PLAN.md or design decisions when plans change.
- Coupling ROS2 runtime to builds in a way that breaks CI; isolate external deps behind features when possible.
- Skipping image-render test tooling or emulator scaffolding.
- Neglecting CLI domain handling (argument + env fallback) or failing to surface errors clearly.

When corrected by the user, summarize the feedback and adjustments here to stay aligned.
