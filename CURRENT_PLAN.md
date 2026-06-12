# Current Plan

Purpose: keep a living checklist for pausing/resuming work.
Issue tracker: <https://github.com/victorpaleologue/ros-viz-rs/issues>

## Done

- [x] FK robot model (full URDF + k chain), real geometry incl. COLLADA
  meshes (upstream mesh-loader merge bug found + worked around) (#3, #4)
- [x] True headless GPU snapshots + pure-Rust visual checks; visual
  regression and end-to-end (DDS→pixels) test suites (#1, #2)
- [x] Message registry: 50 standard types, generic egui view/edit (#5)
- [x] Emulator over real DDS with joint scripts (#6)
- [x] rosbridge backend (JSON/WebSocket, ewebsock) sharing the topics.rs
  seam; tested against an in-process fake rosbridge server (#7)
- [x] wasm build + web demo page (NAO waving, verified in headless
  Chromium); GitHub Pages enabled + deploy workflow (#8, #15)
- [x] Docker integration tests: ur5e (robot_state_publisher) and
  naoqi_driver2 `fake_naoqi`, Linux-CI host networking (#9)
- [x] Demo mode (`--demo`), DDS multicast preflight for tests
- [x] CI (fmt/clippy/tests/feature-matrix/wasm), version gate, auto-tagged
  releases: crates.io + npm jobs, .dmg/.deb/.exe artifacts (#11–#14)
- [x] Crate-level rustdoc for docs.rs (#16)

## Open

- [ ] #10 NAO meshes: license-gated installer (CC BY-NC-ND) — fetch in CI
  container via the official package, and/or HTTP mesh resolver for wasm
- [ ] #17 Zenoh / Foxglove WebSocket transport candidates
- [ ] #18 O(1) topic discovery bookkeeping
- [ ] #19 MeshResolver trust boundary (sandbox_root, wasm allowlist)
- [ ] #20 unify env-var test locking
- [ ] #21 web polish: brighter WebGL lighting, egui panel on wasm

## Notes

- NAO meshes are CC BY-NC-ND 4.0 — never vendored into this MIT repo.
- rustdds skips loopback: local DDS needs working multicast on the default
  interface (macOS: 'Local Network' permission). Tests skip loudly when
  absent; rosbridge is unaffected.
- Keep design decisions in docs/wiki/Architecture.md and the crate docs.
