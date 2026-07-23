#!/usr/bin/env bash
# Phase 6D.4 container node-startup smoke.
set -eo pipefail

WS=/workspace/ros2_ws

echo "== [1/2] build overlay (controller + messages + real collection controller) =="
. /opt/ros/humble/setup.sh
. /ros2_ws/install/setup.sh
cd "${WS}"
colcon build \
  --build-base build_smoke --install-base install_smoke \
  --packages-select tennis_robot_msgs tennis_robot_collection_controller tennis_robot \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
. install_smoke/setup.sh

echo "== [2/2] launch_test: real controller collect_route empty scan =="
launch_test /workspace/scripts/collection_route_node_startup_smoke.launch.py
