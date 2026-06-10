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

# ROS 2 Humble uses Python 3.10; create a dedicated venv to avoid version mismatch
VENV_ROS="$SCRIPT_DIR/.venv-ros"
if [ ! -d "$VENV_ROS" ]; then
    echo "Creating Python 3.10 venv for ROS scripts..."
    uv venv --python 3.10 "$VENV_ROS"
    uv pip install --python "$VENV_ROS/bin/python" \
        "duckdb>=1.5.3" "numpy>=1.26" "opencv-python-headless>=4.9" \
        "matplotlib>=3.10" \
        "opentelemetry-api>=1.34" "opentelemetry-sdk>=1.34" \
        "opentelemetry-exporter-otlp-proto-http>=1.34"
fi
VENV_SITE=$(ls -d "$VENV_ROS/lib/python"*/site-packages 2>/dev/null | head -1)
if [ -n "$VENV_SITE" ]; then
    export PYTHONPATH="$VENV_SITE:${PYTHONPATH:-}"
fi

mkdir -p "$SCRIPT_DIR/runtime"

# ── Launch ───────────────────────────────────────────────────────────────────
HEADLESS="${1:-false}"
echo "Starting simulation (headless=$HEADLESS)..."
ros2 launch tennis_robot sim.launch.py headless:="$HEADLESS"
