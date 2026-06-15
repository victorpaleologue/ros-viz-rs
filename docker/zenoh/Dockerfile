# A ROS 2 Jazzy robot speaking native Zenoh (rmw_zenoh), for the Zenoh
# integration test. Brings up the Zenoh router plus robot_state_publisher and
# a joint-state source over rmw_zenoh; the host visualizer connects to the
# router (TCP 7447) and renders the robot. Test-only — never a runtime dep.
FROM ros:jazzy-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-rmw-zenoh-cpp \
        ros-jazzy-robot-state-publisher \
        ros-jazzy-joint-state-publisher \
        python3-minimal \
    && rm -rf /var/lib/apt/lists/*

ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp
# Let the router accept connections from the host visualizer.
ENV ZENOH_ROUTER_CONFIG_URI=/zenoh-router.json5

# A tiny two-link URDF (no meshes — renders as a skeleton, which the vision
# toolkit asserts on, same as the other integration tests).
COPY robot.urdf /robot.urdf
COPY zenoh-router.json5 /zenoh-router.json5
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

EXPOSE 7447
ENTRYPOINT ["/entrypoint.sh"]
