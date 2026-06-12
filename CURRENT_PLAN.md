# Current Plan

Purpose: keep a living checklist for pausing/resuming work.
Issue tracker: <https://github.com/victorpaleologue/ros-viz-rs/issues>

## Done

- [x] Robot model keeps full `urdf_rs::Robot` + `k::Chain` FK (#3)
  - all links get world transforms (NAO: 83/83; the old renderer dropped
    entire subtrees and rendered NAO as a single box)
- [x] Real URDF geometry: box/cylinder/capsule/sphere + mesh loading via
  mesh-loader, `package://` resolution, URDF materials, skeleton fallback
  markers for links without visuals (#4, meshes pending real-robot check)
- [x] True headless snapshots: offscreen render-to-texture + GPU readback,
  no window, works inside `cargo test` (#1)
- [x] Pure-Rust visual checks: RMSE, diff images, silhouette/coverage,
  reference blessing via `ROS_VIZ_BLESS=1` (#2)
- [x] Headless visual regression suite: 8 tests rendering URDF fixtures and
  comparing against `test-data/reference/` (#2)
- [x] App reworked onto the new core; `--snapshot-to` now renders real
  pixels of the robot received from ROS; topics panel in the main app

## Next

- [ ] Message registry + generic egui view/edit for standard messages (#5)
- [ ] Emulator publishing over real DDS (#6)
- [ ] rosbridge WebSocket backend (#7), then wasm build (#8)
- [ ] Docker integration tests, incl. naoqi_driver2 `fake_naoqi` (#9)
- [ ] NAO with real meshes + waving animation (#10)
- [ ] CI + version-bump gate + releases + packaging (#11–#14)
- [ ] GitHub Pages demo site (#15), docs overhaul (#16)

## Notes

- NAO meshes are CC BY-NC-ND 4.0 — fetched from ros-naoqi/nao_meshes2 at
  test/demo time, never vendored into this MIT repo.
- Keep design decisions in docs/wiki/Architecture.md up to date.
