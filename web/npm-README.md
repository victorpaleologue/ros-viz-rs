# ros-viz-rs

A generic **ROS 2 robot visualizer** compiled to WebAssembly — render any
URDF robot in the browser and drive it live from a robot through
[rosbridge](https://github.com/RobotWebTools/rosbridge_suite). Built in Rust
with [Bevy](https://bevy.org).

**[Live demo](https://victorpaleologue.github.io/ros-viz-rs/)** — a NAO
robot waving hello.

## Install

```bash
npm install ros-viz-rs
```

## Use

The package is a `wasm-bindgen` ES module. Call the default export to load
the wasm, then `start()` to launch the visualizer into a `<canvas>`:

```html
<canvas id="ros-viz-canvas"></canvas>
<script type="module">
  import init, { start } from "ros-viz-rs";

  await init();           // download + instantiate the wasm
  // Connect to a robot's rosbridge server:
  start("ws://localhost:9090");
  // …or omit the URL to play the built-in NAO demo:
  // start();
</script>
```

Rendering goes into the element with id `ros-viz-canvas`. See the
[demo page source](https://github.com/victorpaleologue/ros-viz-rs/blob/main/web/index.html)
for a complete example with a connection form.

## Notes

- The browser build connects via **rosbridge** (JSON/WebSocket); it does not
  speak DDS directly (browsers can't). Point it at a `rosbridge_server`.
- Best rendering and the topic inspector need a WebGPU-capable browser; on
  WebGL2 the 3D view works but some UI is limited.

MIT licensed. Sources and the native desktop app (macOS/Windows/Linux) are
on [GitHub](https://github.com/victorpaleologue/ros-viz-rs).
