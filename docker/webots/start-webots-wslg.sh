#!/usr/bin/env bash
set -euo pipefail

WORLD_FILE="${WEBOTS_WORLD:-/workspace/worlds/tennis_court.wbt}"
WEBOTS_MODE="${WEBOTS_MODE:-realtime}"

unset LIBGL_ALWAYS_SOFTWARE
export DISPLAY="${DISPLAY:-:0}"
export WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-wayland-0}"
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/mnt/wslg/runtime-dir}"

# Add ROS 2 Python packages to PYTHONPATH without sourcing the full ROS setup.
# Sourcing setup.bash modifies LD_LIBRARY_PATH heavily; for WSLg we keep the
# Webots/WSL OpenGL libraries first and add only the rclpy extension path.
ROS_PYTHON_DIR=/opt/ros/humble/local/lib/python3.10/dist-packages
ROS2_WS_MSGS=/ros2_ws/install/tennis_robot_msgs/local/lib/python3.10/dist-packages
CTRL_MODULES=/workspace/controllers/ball_detector
export PYTHONPATH="${CTRL_MODULES}:${ROS_PYTHON_DIR}:${ROS2_WS_MSGS}:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="/usr/lib/wsl/lib:/opt/ros/humble/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"

# Overwrite runtime.ini so Webots doesn't use a Windows host path written by
# start_local_webots_control.ps1.
for CTRL_DIR in /workspace/controllers/ball_detector /workspace/controllers/webots_bridge; do
  mkdir -p "$CTRL_DIR"
  printf '[python]\nCOMMAND = python3\n' > "$CTRL_DIR/runtime.ini"
done

echo "Opening Webots through WSLg display:"
echo "  DISPLAY=${DISPLAY:-<unset>}"
echo "  WAYLAND_DISPLAY=${WAYLAND_DISPLAY:-<unset>}"
echo "  XDG_RUNTIME_DIR=${XDG_RUNTIME_DIR:-<unset>}"
echo "Opening world: ${WORLD_FILE}"

webots --stdout --stderr "--mode=${WEBOTS_MODE}" "${WORLD_FILE}"
