# Visual Regression Reference Images

This directory contains reference images for visual regression testing.

## Reference Images

- `simple_arm.png` - 3-DOF robot arm with shoulder and elbow joints
- `two_link_planar.png` - 2-link planar arm configuration
- `triple_pendulum.png` - Triple pendulum system with hanging links

## Updating References

If the rendering code changes intentionally (improved visuals, camera positioning, etc.), regenerate references:

```bash
cargo run --example urdf_view test-data/urdf/simple_arm.urdf --export-snapshot test-data/reference/simple_arm.png
cargo run --example urdf_view test-data/urdf/two_link_planar.urdf --export-snapshot test-data/reference/two_link_planar.png
cargo run --example urdf_view test-data/urdf/triple_pendulum.urdf --export-snapshot test-data/reference/triple_pendulum.png
```

**Important**: Only update these after visually confirming the new renders are correct!

## Testing

Visual regression tests in `tests/visual_regression.rs` compare fresh renders against these references using ImageMagick's `compare` tool.
