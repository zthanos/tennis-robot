#!/usr/bin/env bash
# Native ROS 2 Jazzy bring-up (no Docker). This is the Jazzy replacement for the
# Humble-container run_ubuntu.sh path — both the dev PC and the Pi are
# Ubuntu 24.04 / ROS 2 Jazzy, so the stack runs natively.
#
#   ./run_native.sh                     # Gazebo GUI + full stack (sim + SLAM + Nav2)
#   GAZEBO_HEADLESS=true ./run_native.sh
#   SLAM_MODE=mapping ./run_native.sh   # default: localization
#   BUILD=true ./run_native.sh          # colcon-build the workspace first
#
# Runtime Python deps (were baked into the old Humble image):
#   pip install --user --break-system-packages duckdb onnxruntime
# NOTE: no `set -u` — the ROS 2 setup.bash files reference unbound env vars.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

export GAZEBO_HEADLESS="${GAZEBO_HEADLESS:-false}"
export SLAM_MODE="${SLAM_MODE:-localization}"
export WORKSPACE="$SCRIPT_DIR"
# Nodes default their runtime-file paths under $TENNIS_ROBOT_ROOT (the old
# container mount was /workspace, which is not writable natively).
export TENNIS_ROBOT_ROOT="$SCRIPT_DIR"
# Shared domain so a distributed Pi (run_pi.sh) auto-discovers this sim over DDS.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
RUN_LOCK_FILE="/tmp/tennis_robot_native_domain_${ROS_DOMAIN_ID}.lock"
exec 9>"$RUN_LOCK_FILE"
if ! flock -n 9; then
    echo "ERROR: another run_native.sh is already active for ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
    echo "Stop the existing PC stack before starting a second simulation."
    exit 1
fi
# TENNIS_LAUNCH_BRAIN=false → distributed mode: run ONLY the sim here; the
# control/Nav2/SLAM stack runs on the Pi (run_pi.sh).
export TENNIS_LAUNCH_BRAIN="${TENNIS_LAUNCH_BRAIN:-true}"
# Distributed sim: run perception on the PC (next to the Gazebo camera) so only
# BallDetectionArray crosses to the Pi — streaming raw RGB/depth over DDS starves
# the scan. The all-in-one run keeps perception in the brain group (local anyway).
if [ "$TENNIS_LAUNCH_BRAIN" = "false" ]; then
    export TENNIS_PERCEPTION_ON_PC="${TENNIS_PERCEPTION_ON_PC:-true}"
    required_udp_rmem=4194304
    actual_udp_rmem="$(sysctl -n net.core.rmem_default)"
    if [ "$actual_udp_rmem" -lt "$required_udp_rmem" ]; then
        echo "ERROR: distributed simulation requires net.core.rmem_default >= $required_udp_rmem"
        echo "Install it with: sudo ./scripts/network/install_udp_buffer_profile.sh"
        exit 1
    fi
fi

# ROS builds and node entry points must use the SYSTEM Python. A uv-managed
# python3.12 under ~/.local/bin shadows it in PATH and lacks the ROS/build
# modules (empy, lark, rclpy, ...), so put /usr/bin first.
export PATH="/usr/bin:$PATH"

# shellcheck disable=SC1091
. /opt/ros/jazzy/setup.bash

WS="ros2_ws"
if [ "${BUILD:-false}" = "true" ] || [ ! -f "$WS/install_jazzy/setup.bash" ]; then
    echo "[run_native] colcon build (Jazzy)…"
    ( cd "$WS" && colcon --log-base log_jazzy build \
        --build-base build_jazzy --install-base install_jazzy \
        --packages-select tennis_robot_msgs tennis_robot_collection_controller tennis_robot )
fi
# shellcheck disable=SC1091
. "$WS/install_jazzy/setup.bash"

# Neural perception is mandatory on whichever machine owns perception. In both
# all-in-one and distributed-PC mode that is this process.
BALL_MODEL="${BALL_MODEL_PATH:-$SCRIPT_DIR/models/yolov8n.onnx}"
COURT_SCENE_MODEL="${COURT_SCENE_MODEL_PATH:-$SCRIPT_DIR/models/court_scene_yolov8n.onnx}"
if [ ! -s "$BALL_MODEL" ]; then
    echo "ERROR: required neural ball model missing or empty: $BALL_MODEL"
    exit 1
fi
if [ ! -s "$COURT_SCENE_MODEL" ]; then
    echo "ERROR: required neural court-scene model missing or empty: $COURT_SCENE_MODEL"
    echo "Train/export it with scripts/train_court_scene_yolo.py."
    exit 1
fi

# sim (Gazebo + panel + nodes) and SLAM in the background, Nav2 core in the
# foreground so the navigate_to_pose action server is up — mirrors
# scripts/docker_dev_entry.sh. Kill the whole process group on exit.
pids=()
cleanup() {
    trap - EXIT INT TERM
    for pid in "${pids[@]}"; do
        kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
    done
    # Gazebo may ignore TERM during renderer shutdown. Scope the fallback to
    # launch-owned process groups; never use a host-wide `pkill gz sim`.
    sleep 1
    for pid in "${pids[@]}"; do
        kill -KILL -- "-$pid" 2>/dev/null || true
    done
}
trap cleanup EXIT INT TERM

if [ "$TENNIS_LAUNCH_BRAIN" = "false" ]; then
    # Distributed PC side: run ONLY the sim (Gazebo + robot abstraction). SLAM +
    # Nav2 + the control stack run on the Pi (run_pi.sh) and reach this over DDS.
    echo "[run_native] distributed PC mode — sim only (control stack on the Pi)"
    setsid ros2 launch tennis_robot sim.launch.py "headless:=${GAZEBO_HEADLESS}" &
    sim_only_pid="$!"
    pids+=("$sim_only_pid")
    wait "$sim_only_pid"
    exit $?
fi

setsid ros2 launch tennis_robot sim.launch.py "headless:=${GAZEBO_HEADLESS}" &
pids+=($!)
# Let Gazebo + the robot spawn and start publishing TF before SLAM starts, then
# let SLAM publish map->odom before Nav2 configures its costmaps — otherwise the
# planner_server global_costmap times out waiting for the `map` frame and Nav2
# never reaches "active" (mirrors scripts/docker_dev_entry.sh's sleeps).
sleep "${SIM_START_DELAY_S:-12}"
setsid ros2 launch tennis_robot "slam_${SLAM_MODE}.launch.py" &
pids+=($!)
sleep "${NAV2_START_DELAY_S:-25}"

ros2 launch tennis_robot navigation.launch.py
