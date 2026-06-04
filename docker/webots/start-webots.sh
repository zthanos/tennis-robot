#!/usr/bin/env bash
set -euo pipefail

WORLD_FILE="${WEBOTS_WORLD:-/workspace/worlds/tennis_court.wbt}"
DISPLAY_SIZE="${DISPLAY_SIZE:-1280x800x24}"
NOVNC_PORT="${NOVNC_PORT:-6080}"
VNC_PORT="${VNC_PORT:-5900}"

# Add ROS 2 Python packages to PYTHONPATH without sourcing the full ROS setup.
# Sourcing setup.bash modifies LD_LIBRARY_PATH which overrides Webots' Mesa
# libraries and causes OpenGL initialization failures.
ROS_PYTHON_DIR=/opt/ros/humble/local/lib/python3.10/dist-packages
ROS2_WS_MSGS=/ros2_ws/install/tennis_robot_msgs/local/lib/python3.10/dist-packages
CTRL_MODULES=/workspace/controllers/ball_detector
export PYTHONPATH="${CTRL_MODULES}:${ROS_PYTHON_DIR}:${ROS2_WS_MSGS}:${PYTHONPATH:-}"
# rclpy C extensions need their .so directory — add only this, not all of /opt/ros/humble/lib
export LD_LIBRARY_PATH="/opt/ros/humble/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"

# Overwrite runtime.ini so Webots doesn't use a Windows host path written by
# start_local_webots_control.ps1.
for CTRL_DIR in /workspace/controllers/ball_detector /workspace/controllers/webots_bridge; do
  mkdir -p "$CTRL_DIR"
  printf '[python]\nCOMMAND = python3\n' > "$CTRL_DIR/runtime.ini"
done

cleanup() {
  pkill -f "Xvfb ${DISPLAY}" >/dev/null 2>&1 || true
  rm -f "/tmp/.X${DISPLAY#:}-lock"
}

trap cleanup EXIT
cleanup

Xvfb "${DISPLAY}" -screen 0 "${DISPLAY_SIZE}" &

fluxbox >/tmp/fluxbox.log 2>&1 &
x11vnc -display "${DISPLAY}" -forever -shared -nopw -rfbport "${VNC_PORT}" >/tmp/x11vnc.log 2>&1 &
websockify --web=/usr/share/novnc "${NOVNC_PORT}" "localhost:${VNC_PORT}" >/tmp/novnc.log 2>&1 &

echo "Webots noVNC is available at http://localhost:${NOVNC_PORT}/vnc.html"
echo "Opening world: ${WORLD_FILE}"

webots --stdout --stderr --mode=fast "${WORLD_FILE}"
