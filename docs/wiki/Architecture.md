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
  robot/            Robot model: URDF ingestion, kinematics (k), geometry
    urdf.rs         urdf-rs parsing + package:// mesh resolution
    kinematics.rs   k::Chain construction, FK from joint positions
  scene/            Bevy plugin: spawn robot entities, sync transforms,
                    camera, lighting, ground grid
  snapshot/         Offscreen render-to-texture + PNG readback (headless)
  messages/         serde structs for standard ROS messages + registry
                    (type name -> subscribe/publish/reflect vtable)
  connection/       Transports producing/consuming ECS topic entities
    ros2.rs         DDS via ros2-client (native only)
    rosbridge.rs    rosbridge JSON/WebSocket (native + wasm)
  ui/               egui panels: topic tree, generic message view/edit,
                    joint controls
  emulator/         In-process fake robot publishing /robot_description and
                    /joint_states over a real transport (for tests/demos)
  app.rs            Composition: CLI options -> plugins
  main.rs           Native binary entry point
```

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
