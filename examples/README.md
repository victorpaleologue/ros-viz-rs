# URDF Test Tool

Test URDF parsing and visualization by loading URDF files from disk and exporting rendered images.

## Usage

```bash
cargo run --example test_urdf <path-to-urdf-file>
```

## Examples

Test with the provided sample URDFs:

```bash
# Simple 3-DOF arm
cargo run --example test_urdf test-data/urdf/simple_arm.urdf

# Two-link planar arm
cargo run --example test_urdf test-data/urdf/two_link_planar.urdf

# Triple pendulum
cargo run --example test_urdf test-data/urdf/triple_pendulum.urdf
```

## Output

The tool will:

1. Parse the URDF file and extract kinematic data
2. Render the robot model in 3D with proper joint hierarchy
3. Export an image to:
   - System temp directory (preferred): `/tmp/<name>_test.png` on Unix
   - Fallback: `.test_outputs/<name>_test.png` (gitignored)

## Visual Inspection

After running the tool, check the exported images to verify:

- Links are properly positioned and oriented
- Joints are correctly placed between parent and child links
- Joint axes are aligned as specified in the URDF
- The kinematic tree hierarchy matches the URDF structure

See [test-data/urdf/README.md](../test-data/urdf/README.md) for expected visualizations of each sample.
