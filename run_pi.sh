#!/usr/bin/env bash
# WS3 — Pi (brain) side of the distributed setup. The Gazebo simulation runs on
# the PC (`TENNIS_LAUNCH_BRAIN=false ./run_native.sh`); this brings up the
# control/perception/Nav2/SLAM stack + the exposed web console on the Pi and
# drives the PC sim over ROS 2 DDS. Both machines run ROS 2 Jazzy, share
# ROS_DOMAIN_ID, and sit on the same LAN.
#
#   ./run_pi.sh                     # brain stack (controller+perception+Nav2+SLAM+panel)
#   ROS_DOMAIN_ID=42 ./run_pi.sh    # must match the PC
#   SLAM_MODE=localization ./run_pi.sh   # default: mapping (Pi has no saved map)
#   BUILD=true ./run_pi.sh          # colcon-build first
#
# Start the PC sim FIRST, then this — SLAM/Nav2 need the PC's /scan, /clock and
# odom->base_footprint TF to be flowing before they come up.
# NOTE: no `set -u` — ROS setup.bash references unbound env vars.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Shared with the PC. DDS auto-discovers peers on the same domain + LAN.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
# Pi has no saved posegraph, so build the map live from the PC's /scan.
export SLAM_MODE="${SLAM_MODE:-mapping}"
export WORKSPACE="$SCRIPT_DIR"
export TENNIS_ROBOT_ROOT="$SCRIPT_DIR"
# Brain only — the sim (Gazebo + robot abstraction) lives on the PC.
export TENNIS_LAUNCH_SIM=false
export TENNIS_LAUNCH_BRAIN=true
# Distributed sim: perception runs on the PC (the Gazebo camera lives there), so
# it is NOT launched here. A real robot with a Pi-side camera would set this
# false to run perception on the Pi.
export TENNIS_PERCEPTION_ON_PC="${TENNIS_PERCEPTION_ON_PC:-true}"

# System Python owns the ROS/entry-point toolchain (a uv python3.12 shadows it).
export PATH="/usr/bin:$PATH"

# shellcheck disable=SC1091
. /opt/ros/jazzy/setup.bash

WS="ros2_ws"
if [ "${BUILD:-false}" = "true" ] || [ ! -f "$WS/install_jazzy/setup.bash" ]; then
    echo "[run_pi] colcon build (Jazzy)…"
    ( cd "$WS" && colcon --log-base log_jazzy build \
        --build-base build_jazzy --install-base install_jazzy \
        --packages-select tennis_robot_msgs tennis_robot_collection_controller tennis_robot )
fi
# shellcheck disable=SC1091
. "$WS/install_jazzy/setup.bash"

pids=()
cleanup() {
    trap - EXIT INT TERM
    for pid in "${pids[@]}"; do kill -- "-$pid" 2>/dev/null || kill "$pid" 2>/dev/null || true; done
}
trap cleanup EXIT INT TERM

echo "[run_pi] ROS_DOMAIN_ID=$ROS_DOMAIN_ID  SLAM_MODE=$SLAM_MODE  (brain only — sim on the PC)"

# Brain nodes: controller + perception + navigation_node + command_bridge +
# sensor_snapshot + exposed panel (sim.launch.py with TENNIS_LAUNCH_SIM=false).
setsid ros2 launch tennis_robot sim.launch.py "headless:=true" &
pids+=($!)
# Let the brain nodes discover the PC sensors, then bring up SLAM (map->odom
# from the PC's /scan), then Nav2.
sleep "${BRAIN_START_DELAY_S:-8}"
setsid ros2 launch tennis_robot "slam_${SLAM_MODE}.launch.py" &
pids+=($!)
sleep "${NAV2_START_DELAY_S:-15}"

ros2 launch tennis_robot navigation.launch.py
