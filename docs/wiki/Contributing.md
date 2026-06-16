# Contributing

## Workflow

- Run `cargo fmt`, `cargo clippy -- -D warnings`, and `cargo test` before pushing.
- Keep commits scoped to milestones with a measurable test.
- Update docs/wiki/DesignDecisions.md when plans or architecture change.
- Prefer small, reviewable PRs and document notable choices in design notes.

## Code style

- Rust 2024 edition; favor clear, explicit code with short helper functions.
- Add succinct comments when behavior is non-obvious.
- Keep ASCII unless the file already uses non-ASCII.

## Testing

- Default to `cargo test`; integration tests should set up their own fixtures/emulators.
- Avoid test dependencies on external ROS graphs; use the in-repo emulator or feature-gated ROS interactions.
