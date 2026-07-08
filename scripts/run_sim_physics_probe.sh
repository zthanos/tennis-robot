#!/usr/bin/env bash
# Run the live intake / roller physics probe inside the Gazebo container.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$SCRIPT_DIR"

COMPOSE=(
    docker compose
    -f docker-compose.yml
    -f docker-compose.ubuntu.yml
)

if [ "${UBUNTU_GPU:-true}" = "true" ]; then
    COMPOSE+=(-f docker-compose.ubuntu-gpu.yml)
fi

if [ "$#" -eq 0 ]; then
    set -- --duration 20
fi

"${COMPOSE[@]}" --profile gazebo exec gazebo bash -lc '
if [ -f /opt/ros/jazzy/setup.sh ]; then
    . /opt/ros/jazzy/setup.sh
else
    . /opt/ros/humble/setup.sh
fi
. /ros2_ws/install/setup.sh
export PYTHONPATH="/workspace/ros2_ws/src/tennis_robot:/workspace/ros2_ws/src/tennis_robot_msgs:${PYTHONPATH:-}"
python3 -m tennis_robot.sim_physics_probe "$@"
' bash "$@"
