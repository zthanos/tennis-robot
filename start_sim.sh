#!/usr/bin/env bash
# Launch Gazebo Harmonic + ROS 2 Humble simulation (native Linux, no Docker)
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ── ROS 2 + workspace ────────────────────────────────────────────────────────
source /opt/ros/humble/setup.bash
source "$SCRIPT_DIR/ros2_ws/install/setup.bash"

# ── Environment ──────────────────────────────────────────────────────────────
export WORKSPACE="$SCRIPT_DIR"
export ROS2_INSTALL="$SCRIPT_DIR/ros2_ws/install"
export GZ_SIM_RESOURCE_PATH="$SCRIPT_DIR/gazebo/models"
export ROBOT_COMMAND_FILE="$SCRIPT_DIR/runtime/robot_command.json"
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Add tennis_robot Python package to PYTHONPATH (glob handles any python3.x version)
SITE_PKG=$(ls -d "$ROS2_INSTALL/tennis_robot/lib/python"*/site-packages 2>/dev/null | head -1)
if [ -n "$SITE_PKG" ]; then
    export PYTHONPATH="$SITE_PKG:${PYTHONPATH:-}"
else
    echo "WARNING: tennis_robot site-packages not found — did you run colcon build?"
fi

mkdir -p "$SCRIPT_DIR/runtime"

# ── Launch ───────────────────────────────────────────────────────────────────
HEADLESS="${1:-false}"
echo "Starting simulation (headless=$HEADLESS)..."
ros2 launch tennis_robot sim.launch.py headless:="$HEADLESS"
