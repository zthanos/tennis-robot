k#!/usr/bin/env bash
# Ubuntu 24.04 defaults to ROS 2 Jazzy; override ROS_DISTRO_TARGET if needed.
# Launch Gazebo Harmonic + ROS 2 simulation (native Linux, no Docker).
#
# Robot control now runs through ros2_control: the gz_ros2_control plugin hosts
# the controller_manager inside Gazebo and sim.launch.py spawns
# joint_state_broadcaster + diff_drive_controller + lift_wheel_velocity_controller.
#
# Native Jazzy prerequisites:
#   sudo apt install ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
#                    ros-jazzy-controller-manager ros-jazzy-gz-ros2-control \
#                    ros-jazzy-robot-state-publisher
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ── ROS 2 + workspace ────────────────────────────────────────────────────────
ROS_DISTRO_TARGET="${ROS_DISTRO_TARGET:-jazzy}"
ROS_SETUP="/opt/ros/$ROS_DISTRO_TARGET/setup.bash"
WORKSPACE_SETUP="$SCRIPT_DIR/ros2_ws/install/setup.bash"

if [ ! -r "$ROS_SETUP" ]; then
    echo "ERROR: ROS setup not found: $ROS_SETUP"
    exit 1
fi
if [ ! -r "$WORKSPACE_SETUP" ]; then
    echo "ERROR: Workspace is not built. Run:"
    echo "  cd \"$SCRIPT_DIR/ros2_ws\""
    echo "  source \"$ROS_SETUP\""
    echo "  colcon build --symlink-install"
    exit 1
fi

source "$ROS_SETUP"
source "$WORKSPACE_SETUP"

# ── ros2_control sanity check ────────────────────────────────────────────────
if ! ros2 pkg prefix gz_ros2_control >/dev/null 2>&1; then
    echo "ERROR: gz_ros2_control is not installed for ROS $ROS_DISTRO_TARGET."
    exit 1
fi

# ── Environment ──────────────────────────────────────────────────────────────
export WORKSPACE="$SCRIPT_DIR"
export ROS2_INSTALL="$SCRIPT_DIR/ros2_ws/install"
export GZ_SIM_RESOURCE_PATH="$SCRIPT_DIR/gazebo/models:$SCRIPT_DIR/ros2_ws/src"
export ROBOT_COMMAND_FILE="$SCRIPT_DIR/runtime/robot_command.json"
export ROS_LOG_DIR="${ROS_LOG_DIR:-$SCRIPT_DIR/runtime/ros_logs}"
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

# Gazebo does not automatically search the ROS prefix for system plugins when
# launched directly with `gz sim`. Include Jazzy's binary plugin directory and
# then any source-built workspace plugin.
export GZ_SIM_SYSTEM_PLUGIN_PATH="/opt/ros/$ROS_DISTRO_TARGET/lib:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
if [ -d "$ROS2_INSTALL/gz_ros2_control/lib" ]; then
    export GZ_SIM_SYSTEM_PLUGIN_PATH="$ROS2_INSTALL/gz_ros2_control/lib:$GZ_SIM_SYSTEM_PLUGIN_PATH"
fi

# Add tennis_robot Python package to PYTHONPATH (glob handles any python3.x version)
SITE_PKG=$(ls -d "$ROS2_INSTALL/tennis_robot/lib/python"*/site-packages 2>/dev/null | head -1)
if [ -n "$SITE_PKG" ]; then
    export PYTHONPATH="$SITE_PKG:${PYTHONPATH:-}"
else
    echo "WARNING: tennis_robot site-packages not found — did you run colcon build?"
fi

# Reuse the project environment for non-ROS Python dependencies. Jazzy and the
# project environment both use Python 3.12 on Ubuntu 24.04.
VENV_SITE=$(ls -d "$SCRIPT_DIR/.venv/lib/python"*/site-packages 2>/dev/null | head -1)
if [ -n "$VENV_SITE" ]; then
    export PYTHONPATH="$VENV_SITE:${PYTHONPATH:-}"
fi

mkdir -p "$SCRIPT_DIR/runtime" "$ROS_LOG_DIR"

# ── Launch ───────────────────────────────────────────────────────────────────
HEADLESS="${1:-false}"
echo "Starting simulation (headless=$HEADLESS)..."
ros2 launch tennis_robot sim.launch.py headless:="$HEADLESS"
