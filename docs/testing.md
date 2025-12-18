# Visual Regression Testing

Automated integration tests that verify URDF rendering produces consistent visual output.

## Overview

Visual regression tests render URDF files and compare them against reference images using ImageMagick's `compare` tool. This ensures:

- URDF parsing is consistent
- 3D scene construction matches expected output
- Rendering pipeline produces deterministic results
- Breaking changes to visualization are caught automatically

## Running Tests

```bash
# Run all visual regression tests
cargo test --test visual_regression

# Run with output visible
cargo test --test visual_regression -- --nocapture
```

## Requirements

**ImageMagick** is required for image comparison:

```bash
# macOS
brew install imagemagick

# Ubuntu/Debian
sudo apt install imagemagick

# Arch Linux
sudo pacman -S imagemagick
```

If ImageMagick is not available, tests will print a warning and pass (skipped).

## Test Samples

Current test suite covers:

- `simple_arm` - 3-DOF robot arm with shoulder and elbow joints
- `two_link_planar` - 2-link planar arm configuration
- `triple_pendulum` - Triple pendulum with hanging links

Each test:

1. Renders the URDF to a temporary PNG
2. Compares against reference in `assets/tests/reference/`
3. Calculates RMSE (Root Mean Square Error) difference
4. Passes if difference < 0.1%

## Test Output

Passing test:

```text
🔍 Testing: simple_arm
   Rendering URDF...
   Comparing with reference...
   ✓ PASS (diff: 0.0000%)
```

Failing test (hypothetical):

```text
🔍 Testing: simple_arm
   Rendering URDF...
   Comparing with reference...
   ✗ FAIL (diff: 2.3456% > 0.1000%)
   Reference: assets/tests/reference/simple_arm.png
   Generated: /tmp/ros-viz-rs-visual-tests/simple_arm.png
   Diff image: /tmp/ros-viz-rs-visual-tests/simple_arm_diff.png
```

The diff image shows pixel-by-pixel differences in red.

## Updating References

When intentionally changing rendering (camera, lighting, geometry, etc.), regenerate references:

```bash
cargo run --example urdf_view test-data/urdf/simple_arm.urdf --export-snapshot test-data/reference/simple_arm.png
cargo run --example urdf_view test-data/urdf/two_link_planar.urdf --export-snapshot test-data/reference/two_link_planar.png
cargo run --example urdf_view test-data/urdf/triple_pendulum.urdf --export-snapshot test-data/reference/triple_pendulum.png
```

**Important**: Visually inspect the new images before committing! Reference images define "correct" output.

## Adding New Tests

1. Create a new URDF in `test-data/urdf/`
2. Generate reference image:

   ```bash
   cargo run --example urdf_view test-data/urdf/your_robot.urdf --export-snapshot test-data/reference/your_robot.png
   ```

3. Add test entry to `VISUAL_TESTS` array in `tests/visual_regression.rs`:

   ```rust
   VisualTest {
       name: "your_robot",
       urdf_path: "test-data/urdf/your_robot.urdf",
       reference_image: "test-data/reference/your_robot.png",
   },
   ```

4. Run tests to verify:

   ```bash
   cargo test --test visual_regression -- --nocapture
   ```

## CI Integration

Visual regression tests are part of the standard test suite:

```bash
cargo test
```

CI environments must have ImageMagick installed. If missing, tests are skipped with a warning.

## Troubleshooting

### Tests fail with "Could not parse RMSE"

ImageMagick output format may vary. Check the compare command works:

```bash
magick compare -metric RMSE ref.png test.png diff.png
```

Expected output: `1234.56 (0.0123)`

### Tests fail after code changes

1. **Verify the change is intentional** - view diff images in `/tmp/ros-viz-rs-visual-tests/`
2. If correct, update references (see above)
3. If incorrect, fix the rendering bug

### Flaky tests (diff varies slightly)

- Increase `MAX_DIFF_THRESHOLD` in `tests/visual_regression.rs`
- Current: 0.001 (0.1%)
- Small differences may occur due to GPU driver variations

## Architecture

The test infrastructure:

```text
test-data/
  urdf/                     - URDF test samples
  reference/                - Reference images (committed)
tests/visual_regression.rs  - Integration test harness
examples/urdf_view.rs - Rendering tool (deterministic output, interactive viewer)
```

Tests run the `urdf_view` example with --export-snapshot, ensuring integration of:

- URDF parsing (`src/urdf/`)
- Scene construction (entity hierarchy)
- 3D rendering (Bevy systems)
- Image export (offscreen rendering)
