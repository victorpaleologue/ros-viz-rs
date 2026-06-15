# Design Decisions

The load-bearing choices and *why*, including the ones that were bold enough
to be worth a conversation. See [Architecture.md](Architecture.md) for how
the pieces fit; this file is the rationale log.

## Settled foundations

- **Bevy for 3D, egui (via bevy_egui) for widgets.** One engine, native and
  wasm from the same source.
- **URDF via `urdf-rs`, kinematics via `k`, meshes via `mesh-loader`.** We
  keep the full `urdf_rs::Robot` and drive poses through a `k::Chain` rather
  than hand-rolling hierarchy math (matches RViz, supports mimic joints).
- **clap, `--domain` overriding `ROS_DOMAIN_ID`.** No structopt.
- **Native-only deps behind features** (`ros2`, and `webgpu` for wasm) so the
  wasm build stays lean and CI stays green without a ROS runtime.
- **Headless GPU snapshot + pure-Rust image checks are the correctness gate.**
  `cargo test` renders robots offscreen and compares pixels (RMSE +
  silhouette), no ImageMagick. References are blessed with `ROS_VIZ_BLESS=1`.

## Bold decisions (discussed, with rulings)

1. **No `Transport` trait — ECS components *are* the interface.** Each backend
   just populates `TopicInfo` entities + type-erased handles; consumers never
   see transport types. **Ruling: keep it loose.** No abstract trait; the
   modularity lives in the ECS seam (see `src/topics.rs`). Adding a transport
   is a recipe, not an interface to implement (Architecture.md → *Adding a
   transport*).

2. **Reflected value-tree for the generic message UI.** One editor handles all
   message types by reflecting to a value tree. **Ruling: the value tree is a
   debug viewer, that's fine — but JSON is not**, because it loses data (int
   widths, NaN/inf). Move the *internal* representation to RON for fidelity;
   JSON stays only where the wire demands it (the rosbridge protocol is JSON).
   Tracked in #28.

3. **"Native correctness ⇒ web correctness" testing axiom.** The web build adds
   exactly one variable (the wgpu backend); everything visual is shared and
   tested natively, so we don't re-qualify the whole app in a browser per
   change. **Ruling: acceptable; revisit only if a WebGPU-only bug ever
   appears.** Documented in Architecture.md.

4. **Hero demo robot.** NAO meshes are CC BY-NC-ND, so the NAO demo is a
   skeleton. **Ruling: not blocked — let the demo switch between robots, show
   UR5e by default (BSD meshes we can ship), and show NAO first once its
   meshes are available.** Tracked alongside #27.

5. **Hand-rolled message structs vs codegen.** 50 ROS types are hand-written to
   avoid a ROS install / build-time codegen. **Ruling: fine at this size; look
   for an existing Rust ROS-message codegen crate and, regardless, file an
   issue to revisit codegen as coverage grows.** Tracked in #29.

6. **GPU snapshot references: macOS-strict, Linux-informational** (Metal vs
   lavapipe RMSE drift). **Ruling: leave it until an issue arises; what
   matters is that it works on CI.**

## Web backend

- **Two bundles, same source** (Architecture.md → *Two web bundles*): WebGL2
  is universal (3D view everywhere); WebGPU adds the egui topics panel.
  Selected by a real `requestAdapter()` probe, so adapterless browsers fall
  back instead of panicking.

## Licensing

- MIT, free project. Never vendor incompatible assets: NAO meshes (CC
  BY-NC-ND) are user-provided or fetched at test time, never committed. BSD
  robot descriptions (e.g. UR) *may* be shipped, which is why UR is the
  default hero once meshes render on the web.
