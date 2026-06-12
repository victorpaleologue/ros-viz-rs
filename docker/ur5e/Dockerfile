# syntax=docker/dockerfile:1
# A real UR5e ROS 2 stack in a box, used by tests/docker_integration.rs.
#
# Publishes, like a real Universal Robots driver would:
#   - /robot_description (std_msgs/String, transient-local "latched")
#     via robot_state_publisher
#   - /joint_states (sensor_msgs/JointState) via joint_state_publisher
#     (which reads the URDF from the /robot_description topic and publishes
#     every joint at its default position)
#
# The UR5e URDF is generated once at image build time from the official
# ur_description xacro (https://github.com/UniversalRobots/Universal_Robots_ROS2_Description).
#
# Build & run (Linux; see the note below for macOS):
#
#   docker build -t ros-viz-test-ur5e docker/ur5e
#   docker run -d --name ur5e --network host -e ROS_DOMAIN_ID=0 ros-viz-test-ur5e
#
# MUST run with --network host so DDS discovery (UDP multicast on
# 239.255.0.1, ports 7400+) reaches participants on the host. This works on
# Linux only: on macOS/Windows, Docker runs containers inside a VM and
# "host" networking attaches to the VM's stack, not the real host
# (https://docs.docker.com/engine/network/drivers/host/), so the
# end-to-end test is Linux-only.
FROM ros:jazzy

# ros:jazzy is multi-arch (amd64 + arm64); all three packages are released
# for both architectures on packages.ros.org.
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-ur-description \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-xacro \
    && rm -rf /var/lib/apt/lists/*

# Generate the UR5e URDF at build time. `ur_type` selects the model and
# `name` is a mandatory argument of ur.urdf.xacro.
RUN bash -c "source /opt/ros/jazzy/setup.bash \
    && xacro /opt/ros/jazzy/share/ur_description/urdf/ur.urdf.xacro \
       ur_type:=ur5e name:=ur5e > /ur5e.urdf"

# Minimal launch: robot_state_publisher latches /robot_description;
# joint_state_publisher picks the URDF up from that topic and publishes
# /joint_states (all joints at 0) at its default rate.
COPY <<'EOF' /ur5e_launch.py
import launch
import launch_ros.actions


def generate_launch_description():
    with open('/ur5e.urdf') as f:
        robot_description = f.read()
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'),
        launch_ros.actions.Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            output='screen'),
    ])
EOF

COPY <<'EOF' /entrypoint.sh
#!/bin/bash
set -e
source /opt/ros/jazzy/setup.bash
exec ros2 launch /ur5e_launch.py
EOF
RUN chmod +x /entrypoint.sh

# Healthy once both topics are discoverable from inside the container.
HEALTHCHECK --interval=5s --timeout=20s --start-period=10s --retries=30 \
    CMD bash -c "source /opt/ros/jazzy/setup.bash \
        && ros2 topic list | grep -qx /robot_description \
        && ros2 topic list | grep -qx /joint_states"

ENTRYPOINT ["/entrypoint.sh"]
