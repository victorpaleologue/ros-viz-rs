# syntax=docker/dockerfile:1
# A fake NAO robot in a box, used by tests/docker_integration.rs.
#
# Builds naoqi_driver2 from the `fake_naoqi` branch
# (https://github.com/ros-naoqi/naoqi_driver2/tree/fake_naoqi), whose
# emulation mode runs the full driver against in-memory fake NAOqi services
# (src/fake_naoqi/) — no real robot or network connection needed. The driver
# then publishes the same topics as with a real NAO, notably:
#   - /robot_description (std_msgs/String, transient-local; the bundled
#     share/urdf/nao.urdf, published by src/tools/robot_description.cpp)
#   - /joint_states (sensor_msgs/JointState, reflecting the fake ALMotion
#     joint state, interpolated at 100 Hz)
#
# ROS distro choice: Jazzy. The branch uses `rclcpp::Node::create_timer`
# with a duration argument, which only exists since Iron, so Humble (where
# all naoqi deps are apt binaries) does not compile it. On Jazzy only
# ros-jazzy-naoqi-libqi is released on packages.ros.org (checked 2026-06:
# no -libqicore or -bridge-msgs for noble), so those two are built from
# source here, exactly as the branch's own dependencies.repos pins them.
#
# nao_meshes/pepper_meshes are deliberately not installed: the driver does
# not need them, and the host visualizer renders mesh-less links as skeleton
# markers (see src/scene.rs in ros-viz-rs).
#
# Build & run (Linux; macOS cannot do --network host to the real host, see
# docker/ur5e/Dockerfile):
#
#   docker build -t ros-viz-test-fake-nao docker/fake_nao
#   docker run -d --name fake_nao --network host -e ROS_DOMAIN_ID=0 ros-viz-test-fake-nao
FROM ros:jazzy

RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    ros-jazzy-naoqi-libqi \
    && rm -rf /var/lib/apt/lists/*

# The driver plus the dependencies that have no Jazzy binaries, at the same
# sources/branches as the fake_naoqi branch's dependencies.repos (meshes
# skipped).
WORKDIR /ws/src
RUN git clone --branch fake_naoqi --depth 1 \
    https://github.com/ros-naoqi/naoqi_driver2.git naoqi_driver \
    && git clone --branch ros2 --depth 1 \
    https://github.com/ros-naoqi/libqicore.git naoqi_libqicore \
    && git clone --branch main --depth 1 \
    https://github.com/ros-naoqi/naoqi_bridge_msgs2.git naoqi_bridge_msgs

# Remaining dependencies (cv_bridge, kdl_parser, diagnostic_updater, ...)
# come from rosdep; naoqi_libqi is already satisfied via apt.
WORKDIR /ws
RUN bash -c "source /opt/ros/jazzy/setup.bash \
    && apt-get update \
    && rosdep update --rosdistro jazzy \
    && rosdep install --from-paths src --ignore-src --rosdistro jazzy -y \
    && rm -rf /var/lib/apt/lists/*"

RUN bash -c "source /opt/ros/jazzy/setup.bash \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF"

# Emulation-mode launch, per the branch README ("Emulation mode (without a
# robot)"). qi_listen_url is pinned to loopback so the fake driver does not
# open a public libQi endpoint on the (host) network; this only disables
# audio extraction.
COPY <<'EOF' /entrypoint.sh
#!/bin/bash
set -e
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash
exec ros2 launch naoqi_driver naoqi_driver.launch.py \
    emulation_mode:=true robot_type:=nao qi_listen_url:=tcp://127.0.0.1:0
EOF
RUN chmod +x /entrypoint.sh

# Healthy once both topics are discoverable from inside the container.
HEALTHCHECK --interval=5s --timeout=20s --start-period=15s --retries=30 \
    CMD bash -c "source /opt/ros/jazzy/setup.bash \
        && ros2 topic list | grep -qx /robot_description \
        && ros2 topic list | grep -qx /joint_states"

ENTRYPOINT ["/entrypoint.sh"]
