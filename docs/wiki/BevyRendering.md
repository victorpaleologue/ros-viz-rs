# Bevy Rendering Plan

This document outlines how rendering and offscreen capture will be integrated.

## Goals

- Windowed mode for interactive visualization.
- Headless mode for CI/testing to render a single frame to disk without a window.
- Shared Bevy scene setup (camera, lights, robot scene graph) across both modes.

## Approach

- Use Bevy 0.15 with `DefaultPlugins` in windowed runs; swap to `MinimalPlugins + RenderPlugin + ImagePlugin + AssetPlugin + PbrPlugin + CorePipelinePlugin` for headless.
- Control mode via CLI flags `--headless` and `--output-image <PATH>`; headless requires wgpu with `HeadlessSurface` (supported on Vulkan/Metal/DX12) and skips `WindowPlugin`.
- Render graph hook: add a custom node after main pass to copy the primary color target into a GPU buffer, then map to CPU and write PNG. References:
  - Bevy render graph docs: <https://bevyengine.org/learn/rendering/>
  - Wgpu readback: <https://docs.rs/wgpu/latest/wgpu/struct.Buffer.html#method.slice>
- Use deterministic camera transform and lighting for reproducible tests; fix resolution via config (default 800x600, overridable later).
- Ensure `output_image` triggers a one-shot capture after the first frame settles; exit cleanly afterward.

## Testing strategy

- In tests, run the app headless with a tiny fixed resolution (e.g., 320x240) and a static scene fixture; compare rendered image against a golden PNG (byte-for-byte) or hash.
- Provide helper to load a dummy URDF/mesh into the scene; for early tests, a simple cube mesh stands in for the robot.
- Keep rendering code feature-independent so CI can run without ROS; ROS feature only affects data sources, not rendering.

## Open items

- Define a small rendering config struct (resolution, msaa, clear color) and expose via CLI/env.
- Implement the render-graph capture node and image writer (png crate) behind a feature flag if needed to trim deps.
- Add a Bevy app builder function that accepts `AppConfig` and returns an `App` ready for tests (headless/offscreen) or runtime (windowed).
