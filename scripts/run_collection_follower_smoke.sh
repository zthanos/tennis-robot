#!/usr/bin/env bash
# Phase 6D.3 container construction smoke — node factory vs real controller.
#
# Run from the host with:
#
#   docker run --rm -v "$PWD":/workspace -w /workspace \
#     tennis-robot-gazebo bash scripts/run_collection_follower_smoke.sh
#
# It (1) colcon-builds tennis_robot_msgs + tennis_robot_collection_controller +
# tennis_robot into an isolated overlay, then (2) runs a launch_test that brings
# up a real nav2 controller_server with the CollectionFollowPath plugin and lets
# CollectionExecutorNodeFactory construct every ROS handle, then drives a real
# curved plan through Load -> FollowPath -> terminal -> Finalize.
#
# NOTE: no `set -u` — the ROS setup.sh scripts reference unbound vars.
set -eo pipefail

WS=/workspace/ros2_ws

echo "== [1/2] build overlay (msgs + collection controller + tennis_robot) =="
. /opt/ros/humble/setup.sh
. /ros2_ws/install/setup.sh
cd "${WS}"
colcon build \
  --build-base build_smoke --install-base install_smoke \
  --packages-select tennis_robot_msgs tennis_robot_collection_controller tennis_robot \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
. install_smoke/setup.sh

echo "== [2/2] launch_test: Python follower drives the real controller =="
launch_test /workspace/scripts/collection_follower_smoke.launch.py
