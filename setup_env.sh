#!/usr/bin/env bash
# Source this in EVERY terminal for native (non-Docker) runs, so all nodes share
# the same ROS domain + DDS config as start_sim.sh and can see each other:
#
#   source setup_env.sh
#
# Then e.g.:
#   ros2 launch tennis_robot autonomous_survey.launch.py
#   ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel_teleop
#   rviz2

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/humble/setup.bash
if [ -f "$SCRIPT_DIR/ros2_ws/install/setup.bash" ]; then
    source "$SCRIPT_DIR/ros2_ws/install/setup.bash"
else
    echo "WARNING: ros2_ws/install not found — run 'colcon build' in ros2_ws first."
fi

export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="$SCRIPT_DIR/docker/ros2/cyclonedds-gazebo.xml"
export WORKSPACE="$SCRIPT_DIR"
export ROS2_INSTALL="$SCRIPT_DIR/ros2_ws/install"
export GZ_SIM_RESOURCE_PATH="$SCRIPT_DIR/gazebo/models"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$ROS2_INSTALL/gz_ros2_control/lib:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"

echo "tennis-robot env ready (domain 42, cyclonedds)"
