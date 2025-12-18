# URDF Test Tool

Test URDF parsing and visualization by loading URDF files from disk and exporting rendered images.

## Usage

```bash
cargo run --example urdf_view_snapshot <urdf_file> [output_image]
```

If `output_image` is omitted, saves to `<urdf_name>.png` in the current directory.

## Examples

Test with the provided sample URDFs:

```bash
# Default output (simple_arm.png in current directory)
cargo run --example test_urdf test-data/urdf/simple_arm.urdf

# Explicit output path
cargo run --example test_urdf test-data/urdf/simple_arm.urdf output/my_arm.png

# Multiple tests
cargo run --example test_urdf test-data/urdf/two_link_planar.urdf two_link_planar.png
cargo run --example test_urdf test-data/urdf/triple_pendulum.urdf triple_pendulum.png
```

## Output

The tool will:

1. Parse the URDF file and extract kinematic data
2. Render the robot model in 3D with proper joint hierarchy
3. Export an image to the specified path (or `<urdf_name>.png` in current directory)

## Visual Inspection

After running the tool, check the exported images to verify:

- Links are properly positioned and oriented
- Joints are correctly placed between parent and child links
- Joint axes are aligned as specified in the URDF
- The kinematic tree hierarchy matches the URDF structure

See [test-data/urdf/README.md](../test-data/urdf/README.md) for expected visualizations of each sample.
