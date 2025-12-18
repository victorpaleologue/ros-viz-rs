# URDF Test Data

This directory contains simple URDF files for testing the visualization system.

## Test Files

### simple_arm.urdf
A 3-DOF robot arm with:
- Base link (box)
- Shoulder joint (revolute, Z-axis)
- Elbow joint (revolute, Z-axis)
- End effector (box)

Expected visualization: Vertical arm with two rotational joints.

### two_link_planar.urdf
A classic 2-link planar arm:
- 2 links represented as boxes
- Both joints rotate around Z-axis
- Links are 1.0m and 0.8m long

Expected visualization: Horizontal configuration with two links that can rotate in a plane.

### triple_pendulum.urdf
A triple pendulum system:
- 3 cylindrical links of decreasing size
- All joints rotate around Y-axis
- Hanging downward configuration

Expected visualization: Three cylinders hanging vertically, each can swing like a pendulum.

## Testing

Run the URDF test tool:

```bash
cargo run --example urdf_view test-data/urdf/simple_arm.urdf --export-snapshot simple_arm.png
```

This will parse the URDF, render it, and export an image to the specified path for visual inspection.

If you omit the output path, it defaults to `<urdf_name>.png` in the current directory.
