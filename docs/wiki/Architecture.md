# Architecture

This document describes the target architecture of `ros-viz-rs`: a generic
ROS 2 robot visualizer built on Bevy + egui, designed to run natively
(macOS, Linux, Windows) and in the browser (wasm).

## Guiding constraints

- **Small, elegant core.** The robot model, kinematics, and rendering are a
  reusable library; applications (native viewer, web viewer, snapshot tool)
  are thin shells over it.
- **Transport flexibility without premature abstraction.** All transport
  code lives in `connection/`; the rest of the app consumes
  transport-agnostic ECS components and resources. Adding Zenoh or raw DDS
  later means adding one module, not changing consumers.
- **Headless verifiability.** Every visual behavior must be checkable by
  `cargo test` without a window: real GPU offscreen rendering, pixels read
  back and compared in pure Rust.
- **One message model.** ROS messages are plain serde structs. The same
  definitions serialize to CDR for DDS and to JSON for rosbridge, and reflect
  into a generic value tree for the UI.

## Module layout

```text
src/
  lib.rs            Public API, crate docs
  robot/            Robot model (no Bevy types)
    mod.rs          RobotModel: full urdf_rs::Robot + k::Chain FK
    mesh.rs         package:// resolution + STL/OBJ/COLLADA loading
  scene.rs          Bevy plugin: spawn link/visual entities, FK transform
                    sync, auto-framing camera, lights
  snapshot.rs       Offscreen render-to-texture + GPU readback (headless)
  vision.rs         Pure-Rust image checks (RMSE, silhouette, references)
  ros_msgs.rs       serde structs for standard ROS messages
  ros_plugin.rs     DDS transport: discovery -> ECS topic entities
  topics_io.rs      Auto subscribe/publish for discovered topics
  topics_view.rs    egui topic tree with value view/edit
  emulator.rs       Fake robot publishing over real DDS (tests/demos)
  app.rs            Composition: CLI options -> plugins
  main.rs           Native binary entry point
```

Planned moves as transports multiply (#7): `ros_plugin.rs`/`topics_io.rs`
become `connection/ros2.rs`, joined by `connection/rosbridge.rs`.

## Data flow

```text
transport (ros2 | rosbridge | emulator)
   └─ discovers topics ──> ECS entities: TopicInfo + ReadersAndWriters
   └─ /robot_description ─> RobotDescription resource (URDF XML)
   └─ /joint_states ──────> JointPositions resource
robot/
   └─ URDF XML ─> RobotModel { urdf_rs::Robot, k::Chain, geometry }
scene/
   └─ RobotModel ─> link entities with meshes/primitives
   └─ JointPositions ─> FK (k) ─> link world transforms each frame
ui/
   └─ TopicInfo tree ─> generic value view/edit via messages registry
snapshot/
   └─ offscreen camera ─> GPU readback ─> PNG bytes (tests, CLI)
```

## Key decisions

- **ECS as the integration surface.** Topics are entities; components mark
  capabilities (subscribed, publishable, latest value, edit buffer). UI and
  transports never call each other directly.
- **Forward kinematics with `k`.** Joint transforms come from
  `k::Chain::update_transforms()`, not hand-rolled hierarchy rotation. The
  Bevy entity tree is flat per-link; world transforms are written directly.
  This matches RViz semantics and supports mimic joints for free.
- **Reflection-based message UI.** Each registered message type can convert
  to/from a `Value` tree (via serde). The egui inspector renders any
  registered type with editable fields; publishing converts the edited tree
  back to the typed struct. Supporting a new message = one struct + one
  registry line.
- **Visual tests compare real renders.** Image comparison (RMSE + region
  checks) is implemented in Rust with the `image` crate; no ImageMagick.
  Reference images live in `test-data/reference/`. Tests also assert
  structural properties (robot silhouette bounding box, non-background
  coverage) so failures are diagnosable.
- **NAO meshes are not redistributed.** They are CC BY-NC-ND 4.0; tests and
  demos fetch them from `ros-naoqi/nao_meshes2` and cache locally.

## Web (wasm) strategy

The browser cannot speak DDS. The standard bridges, in order of ubiquity:

1. **rosbridge_suite** (JSON over WebSocket) — implemented here first.
2. **Foxglove WebSocket protocol** (CDR over WebSocket) — candidate later;
   reuses our CDR message structs.
3. **Zenoh RMW** — official RMW with a wasm-capable transport; candidate
   once zenoh-wasm stabilizes.

The wasm build compiles the same `scene/`, `robot/`, `ui/` code with the
`rosbridge` connection only, plus an embedded-demo mode that drives the
emulator locally (used by the GitHub Pages demo).

### Two web bundles (WebGL2 + WebGPU)

The page ships both backends and picks at load time (`web/index.html`):

- **WebGL2** (`web/pkg`, default features) — the universal bundle. Renders
  the 3D view in any browser. bevy_egui can't render here (WebGL2 lacks
  `TEXTURE_BINDING_ARRAY`), so the topics panel is absent.
- **WebGPU** (`web/pkg-webgpu`, `--features webgpu`) — adds the egui topics
  panel (view + edit). Chosen only when `navigator.gpu.requestAdapter()`
  actually returns an adapter, so browsers/devices without working WebGPU
  fall back to WebGL2 instead of hard-panicking.

Both bundles are the *same Rust source*; the only difference is the wgpu
backend feature.

### Testing axiom: native correctness ⇒ web correctness

The web build adds exactly one variable over the native build — the wgpu
backend. Everything that decides what's on screen (URDF parsing, FK, scene
construction, the egui panel's layout/widgets, the rosbridge transport) is
backend-agnostic and shared. So instead of re-qualifying the whole app in a
browser on every change, the suite is targeted at the seams:

| Claim | Verified by | Where |
|---|---|---|
| Robot parses, poses (FK), renders | headless GPU snapshot + vision | `tests/visual_regression.rs` (native Metal/Vulkan) |
| rosbridge transport → topics/values | in-process fake server | rosbridge e2e test (native) |
| egui panel layout (view + edit) | egui shape-text capture | `src/topics_view.rs` tests (native) |
| wasm actually renders in a browser | Playwright on the WebGL2 bundle | the NAO demo + a mock-rosbridge connection render in headless Chromium |
| WebGPU bundle is selected & boots | adapter detection + bundle builds | `web/index.html`; both bundles compile in CI |

The standing assumption, made explicit so it can be re-checked when it
changes: **if the shared code renders correctly natively, and the chosen
wgpu backend initializes in the browser, the web build renders correctly.**
The only thing this leaves unverifiable in headless CI is the *pixels* of
the WebGPU bundle (no GPU adapter in CI) — covered by the WebGL2 in-browser
render plus the native egui-panel test, with a final human glance at the
deployed demo on a WebGPU browser.

## Adding a transport (issue #17)

The seam is `src/topics.rs`: a backend's only job is to discover topics into
`TopicInfo` entities and attach the type-erased subscription/publisher
handles plus reflected `serde_json::Value`s. Nothing downstream (UI, scene,
your code) knows which transport produced them. To add Zenoh, the Foxglove
WebSocket protocol, or raw DDS:

1. Add a feature flag in `Cargo.toml` (mirror `rosbridge`), behind which the
   transport's deps live.
2. Add a `Plugin` (mirror `src/rosbridge.rs` or `src/ros_plugin.rs`) that, on
   `Update`, reconciles the live topic set into `TopicInfo` entities and
   feeds `/robot_description` + `/joint_states` into the existing resources.
3. Reuse `ros_msgs`/`messages` for payloads — the structs already serialize
   to both CDR (DDS/Foxglove) and JSON (rosbridge), so no new schema work.
4. Wire it into `app::build_app` next to the existing backends.

No abstract `Transport` trait is introduced on purpose: the ECS components
*are* the interface. This keeps each backend free to use its natural API
(async sockets, DDS callbacks) without a lowest-common-denominator wrapper.
