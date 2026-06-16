# Agent guidelines for ros-viz-rs

Keep this file short. It points at the authoritative docs rather than
repeating them — when something here would duplicate another file, link that
file instead and fix it there.

## Read these first (don't restate them here)

- **Architecture & module map** — the crate-level rustdoc in
  [`src/lib.rs`](src/lib.rs) (run `cargo doc --open`) and
  [`docs/wiki/Architecture.md`](docs/wiki/Architecture.md). These are the
  source of truth for how the pieces fit; update them when the design moves.
- **What's done / in flight** — the
  [issue tracker](https://github.com/victorpaleologue/ros-viz-rs/issues).
- **User-facing usage & recipes** — [`README.md`](README.md).
- **Owner-only setup (secrets, releases)** — [`docs/MAINTAINER.md`](docs/MAINTAINER.md).
- **Design rationale per area** — module-level `//!` docs in each `src/*.rs`;
  they cite the upstream docs consulted. Add to those, not here.

## Conventions not captured elsewhere

- **Verify by rendering, not by hoping.** Behaviour that produces pixels is
  proven with the headless `snapshot` + `vision` toolkit in `cargo test`
  (see `tests/visual_regression.rs`). After an intentional rendering change,
  re-bless references with `ROS_VIZ_BLESS=1 cargo test --test visual_regression`
  and eyeball the new PNGs before committing.
- **Green gate before every commit:** `cargo fmt && cargo clippy
  --all-targets -- -D warnings && cargo test`. The only acceptable clippy
  output is the transitive `block v0.1.6` future-incompat note (a bevy/metal
  dep, not ours).
- **Releases are automatic and version-gated.** Every change to `main` that
  bumps `Cargo.toml`'s version auto-tags and releases (see
  `docs/MAINTAINER.md`). Bump once per intended release; commit unrelated
  fixes without bumping to avoid spurious releases.
- **Dependencies:** prefer updating over downgrading; keep native-only deps
  (DDS, snapshot) behind features/`cfg` so the wasm build stays lean. `clap`
  and Bevy, never `structopt`/`kiss3d`.
- **Licensing:** this repo is MIT. Never vendor incompatible assets — e.g.
  NAO meshes are CC BY-NC-ND and are fetched at test time, never committed
  (see issue #10).
- **Style:** ASCII unless the file already uses otherwise; Markdown valid for
  `markdownlint`. Comments describe current state, not the change.

## Corrections and adjustments

When the user corrects course, record the durable lesson here in a line or
two (and in the relevant doc above), not a transcript.
