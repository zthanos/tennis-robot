"""Milestone A — court mapping with slam_toolbox.

Brings up SLAM mapping + the canonical cmd_vel arbiter (twist_mux). Assumes the
robot is already running (sim.launch.py or the real robot) and publishing:
    /scan          sensor_msgs/LaserScan   (frame lidar_link)
    /odom + TF     odom -> base_link        (diff_drive_controller)

Drive the robot to map the court (teleop in a separate terminal):
    ros2 run teleop_twist_keyboard teleop_twist_keyboard \\
        --ros-args -r /cmd_vel:=/cmd_vel_teleop

Save the map when done:
    ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \\
        "{name: {data: 'runtime/court_map'}}"

See docs/survey/slam-mapping-el.md.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("tennis_robot")
    slam_config = os.path.join(pkg_share, "config", "slam_toolbox.yaml")
    twist_mux_config = os.path.join(pkg_share, "config", "twist_mux.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")

    args = [
        DeclareLaunchArgument("use_sim_time", default_value="true"),
    ]

    # ── SLAM (mapping) ────────────────────────────────────────────────────────
    slam_toolbox = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[slam_config, {"use_sim_time": use_sim_time}],
    )

    # NOTE: twist_mux (the cmd_vel arbiter) now lives in the base bring-up
    # (sim.launch.py / real robot), so the web D-pad and teleop work without
    # SLAM running. This launch only adds the mapping.

    return LaunchDescription(args + [slam_toolbox])
