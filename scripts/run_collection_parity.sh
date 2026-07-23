#!/usr/bin/env bash
# Phase 6B Part 4 — cross-language parity harness (runs INSIDE the container).
#
# Proves the REAL C++ collection controller accepts a CollectionExecutionContext
# + nav_msgs/Path produced by the pure Python Phase 6B serializer.
#
# Run from the host with:
#
#   docker run --rm -v "$PWD":/workspace -w /workspace \
#     tennis-robot-gazebo bash scripts/run_collection_parity.sh
#
# It (1) colcon-builds tennis_robot_msgs + tennis_robot_collection_controller
# into an isolated build/install base, (2) emits the parity fixture from a real
# plan_collection_route plan (pure Python), and (3) runs the parity gtest which
# reconstructs the wire messages and drives the real controller's Load service
# + setPlan. Success = sha256 match + Load ACCEPTED + setPlan accepted.
# NOTE: no `set -u` — the ROS setup.sh scripts reference unbound vars.
set -eo pipefail

WS=/workspace/ros2_ws
FIXTURE=/workspace/runtime/collection_parity_fixture.json

echo "== [1/3] source ROS + build controller overlay =="
. /opt/ros/humble/setup.sh
. /ros2_ws/install/setup.sh          # baked tennis_robot_msgs + tennis_robot
cd "${WS}"
colcon build \
  --build-base build_parity --install-base install_parity \
  --packages-select tennis_robot_msgs tennis_robot_collection_controller \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
. install_parity/setup.sh

echo "== [2/3] emit parity fixture from a real plan (pure Python) =="
mkdir -p /workspace/runtime
python3 /workspace/scripts/emit_collection_parity_fixture.py "${FIXTURE}"

echo "== [3/3] run parity gtest against the real C++ controller =="
GTEST_BIN="${WS}/build_parity/tennis_robot_collection_controller/test_collection_execution_context_parity"
if [ ! -x "${GTEST_BIN}" ]; then
  echo "parity gtest binary not found at ${GTEST_BIN}" >&2
  exit 1
fi
COLLECTION_PARITY_FIXTURE="${FIXTURE}" "${GTEST_BIN}"
