"""Nav2 core bring-up (SLAM-based, no AMCL/map_server).

Starts the controller, planner, behavior and BT-navigator servers + a lifecycle
manager. Assumes slam_toolbox is providing map -> odom (see slam_mapping.launch.py).

The controller's velocity command is remapped to /cmd_vel_nav so twist_mux
arbitrates it before it reaches the diff_drive_controller.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("tennis_robot")
    default_params = os.path.join(pkg_share, "config", "nav2_params.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")

    args = [
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("params_file", default_value=default_params),
    ]

    lifecycle_nodes = [
        "controller_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
    ]

    controller = Node(
        package="nav2_controller",
        executable="controller_server",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
        remappings=[("cmd_vel", "/cmd_vel_nav")],   # -> twist_mux input
    )

    planner = Node(
        package="nav2_planner",
        executable="planner_server",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
    )

    behaviors = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
        remappings=[("cmd_vel", "/cmd_vel_nav")],   # same channel as controller_server
    )

    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "autostart": True,
            "node_names": lifecycle_nodes,
        }],
    )

    return LaunchDescription(args + [
        controller, planner, behaviors, bt_navigator, lifecycle_manager,
    ])
