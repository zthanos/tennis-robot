#!/usr/bin/env bash
# Launch Gazebo Harmonic + ROS 2 Humble simulation (native Linux, no Docker)
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ── ROS 2 + workspace ────────────────────────────────────────────────────────
source /opt/ros/humble/setup.bash
source "$SCRIPT_DIR/ros2_ws/install/setup.bash"

# ── Environment ──────────────────────────────────────────────────────────────
export WORKSPACE="$SCRIPT_DIR"
export GZ_SIM_RESOURCE_PATH="$SCRIPT_DIR/gazebo/models"
export ROBOT_COMMAND_FILE="$SCRIPT_DIR/runtime/robot_command.json"
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

mkdir -p "$SCRIPT_DIR/runtime"

# ── Launch ───────────────────────────────────────────────────────────────────
HEADLESS="${1:-false}"
echo "Starting simulation (headless=$HEADLESS)..."
ros2 launch tennis_robot sim.launch.py headless:="$HEADLESS"
