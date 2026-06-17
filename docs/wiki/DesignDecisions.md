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

## Android windowing & safe-area insets

- **Standalone `NativeActivity`, not GameActivity or an embedded view.** The
  Android build *is* the activity: Bevy/winit own the `NativeActivity` and the
  event loop (the `#[bevy_main]` entry point forwards the `AndroidApp` into
  `bevy_winit`). We deliberately build the `android-native-activity` backend,
  not Bevy's default GameActivity (see `src/android.rs`, `Cargo.toml`).

- **Bevy has no safe-area / insets API — this is an upstream limitation, not a
  config we missed.** `NativeActivity` lays its surface out edge-to-edge under
  the status/navigation bars, so the egui UI collided with the status bar. A
  portable fix does not exist yet:
  - [`bevy#23003`](https://github.com/bevyengine/bevy/issues/23003) (a built-in
    `SafeAreaInsets`) is **open and `S-Blocked`**.
  - It's blocked on winit's `Window::safe_area`
    ([`winit#3890`](https://github.com/rust-windowing/winit/pull/3890)), which
    is only in the **unreleased winit 0.31**, and whose **Android
    implementation is still an unmerged PR**
    ([`winit#4506`](https://github.com/rust-windowing/winit/pull/4506)).
  - Bevy 0.18 is still on **winit 0.30**, so even the primitive isn't in our
    tree.

  **Interim solution (`src/android.rs::apply_safe_area_insets`):** read the
  inset-aware content rectangle ourselves via
  `AndroidApp::content_rect()` and reserve each edge with empty egui spacer
  panels, registered before the connection bar and topics panel. This is
  *exactly* the data source `winit#4506` turns into `Window::safe_area`, so it
  is the sanctioned interim approach — migrating later is a swap to the upstream
  API, not a rewrite. **Ruling: keep the spacers; only add a JNI `WindowInsets`
  read if a device reports a full-screen content rect; adopt `safe_area` once
  Bevy upgrades to winit 0.31.**

- **Why not GameActivity.** Its `SurfaceView` does not honor
  `setDecorFitsSystemWindows`, so the system bars overlay content anyway —
  reported specifically with egui
  ([`android-activity#96`](https://github.com/rust-mobile/android-activity/issues/96)).
  It wouldn't fix insets and costs the games-activity AAR, so NativeActivity
  stays.

- **Why not embed in a "normal" Activity / Jetpack Compose.** Hosting the
  render surface in a regular Activity (or Compose `AndroidExternalSurface`)
  *is* the clean Android idiom — you get real `WindowInsets`,
  `enableEdgeToEdge`, IME handling for free. But Bevy/winit insist on **owning**
  the Activity, and Bevy has **no first-party API to render into an
  externally-owned surface** ([discussion
  `#10900`](https://github.com/bevyengine/bevy/discussions/10900)). The only
  proven path bypasses `WinitPlugin` entirely
  ([`jinleili/bevy-in-app`](https://github.com/jinleili/bevy-in-app), which
  supports Bevy 0.18) and pulls in a full Gradle/Kotlin project plus manual
  surface, lifecycle, and input plumbing — losing the single `cargo-ndk` build.
  **Ruling: deferred. It does not fix insets any better than the spacers for a
  standalone app; reconsider only if shipping ros-viz-rs as an embedded
  component inside a larger Android app becomes a goal.**

## Licensing

- MIT, free project. Never vendor incompatible assets: NAO meshes (CC
  BY-NC-ND) are user-provided or fetched at test time, never committed. BSD
  robot descriptions (e.g. UR) *may* be shipped, which is why UR is the
  default hero once meshes render on the web.
