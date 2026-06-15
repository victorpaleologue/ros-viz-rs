#!/bin/bash
# Bring up the Zenoh router and a ROS 2 robot speaking rmw_zenoh:
# robot_state_publisher latches /robot_description, joint_state_publisher
# streams /joint_states. The host visualizer connects to the router on 7447.
set -e
source /opt/ros/jazzy/setup.bash

URDF="$(cat /robot.urdf)"

# The Zenoh router (rmw_zenoh's discovery + data broker).
ros2 run rmw_zenoh_cpp rmw_zenohd &
sleep 5

ros2 run robot_state_publisher robot_state_publisher \
    --ros-args -p robot_description:="$URDF" &

# Publish joint states (default positions) so /joint_states exists.
ros2 run joint_state_publisher joint_state_publisher \
    --ros-args -p robot_description:="$URDF" &

wait
