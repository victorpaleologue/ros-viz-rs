# URDF Test Tool

Test URDF parsing and visualization by loading URDF files from disk and exporting rendered images.

## Usage

```bash
cargo run --example urdf_view <urdf_file> [--export-snapshot <output_image>]
```

Without `--export-snapshot`, the model opens in an interactive viewer window.

## Examples

Test with the provided sample URDFs:

```bash
# Interactive viewer
cargo run --example urdf_view test-data/urdf/simple_arm.urdf

# Export a snapshot to a given path
cargo run --example urdf_view test-data/urdf/simple_arm.urdf --export-snapshot output/my_arm.png

# Multiple tests
cargo run --example urdf_view test-data/urdf/two_link_planar.urdf --export-snapshot two_link_planar.png
cargo run --example urdf_view test-data/urdf/triple_pendulum.urdf --export-snapshot triple_pendulum.png
```

## Output

The tool will:

1. Parse the URDF file and extract kinematic data
2. Render the robot model in 3D with proper joint hierarchy
3. Export an image to the path given by `--export-snapshot`, or open an interactive viewer window if none is given

## Visual Inspection

After running the tool, check the exported images to verify:

- Links are properly positioned and oriented
- Joints are correctly placed between parent and child links
- Joint axes are aligned as specified in the URDF
- The kinematic tree hierarchy matches the URDF structure

See [test-data/urdf/README.md](../test-data/urdf/README.md) for expected visualizations of each sample.

---

## Topics Tree Viewer

Display ROS 2 topics discovered live via DDS in an interactive, collapsible tree widget.

### Topics View Usage

```bash
cargo run --example view_topics --features ros,ui
```

Topics are discovered from the DDS graph on ROS domain 0. Click on a branch node
to expand or collapse it.
