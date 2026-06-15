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

## Recently shipped

- [x] #18 O(1) topic discovery; #19 sandbox_root; #20 env-test lock; #21 web
  lighting; #22 maintainer doc; #23 loading indicator; #24 npm README;
  #25 .rpm; #26 egui panel on web via dual WebGL2+WebGPU bundles; #30 robot
  switcher with hosted UR5e meshes (UR5e default, verified in-browser)
- [x] Browser **mesh upload** (drop in license-bound meshes locally)
- [x] Design decisions recapped (docs/wiki/DesignDecisions.md); native⇒web
  testing axiom documented

## Open

- [ ] #17 **Zenoh** (rmw_zenoh) transport — full plan + key formats in the
  issue; next-release headline. Needs a Docker rmw_zenoh test stack.
- [ ] #28 RON value-tree (avoid JSON data loss in the editor)
- [ ] #29 evaluate roslibrust_codegen vs hand-rolled messages
- [ ] #27 remainder: generic *remote* HTTP mesh fetch + origin allowlist
  (hosted UR5e + upload already cover the common cases)
- [ ] #10 NAO-with-meshes by default on the public demo — license-blocked;
  mitigated by upload + the switcher

## Notes

- NAO meshes are CC BY-NC-ND 4.0 — never vendored into this MIT repo.
- rustdds skips loopback: local DDS needs working multicast on the default
  interface (macOS: 'Local Network' permission). Tests skip loudly when
  absent; rosbridge is unaffected.
- Keep design decisions in docs/wiki/Architecture.md and the crate docs.
