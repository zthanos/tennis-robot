"""Milestone B — autonomous court survey.

One command that brings up the whole autonomous-mapping brain on top of an
already-running robot (sim.launch.py or the real robot):

    slam_toolbox (mapping)      -> /map + map->odom
    twist_mux (cmd_vel arbiter) -> /diff_drive_controller/cmd_vel_unstamped
    Nav2 (controller/planner/behaviors/bt) -> follows goals, avoids obstacles
    explore_lite (frontier)     -> sends goals to unknown space until mapped

When the court is fully explored, explore_lite returns the robot home. Save the
map with the slam_toolbox service (see docs/autonomous-survey-el.md).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("tennis_robot")
    launch_dir = os.path.join(pkg_share, "launch")
    explore_params = os.path.join(pkg_share, "config", "explore.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    args = [DeclareLaunchArgument("use_sim_time", default_value="true")]

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "slam_mapping.launch.py")),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "navigation.launch.py")),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    # explore_lite (m-explore-ros2)
    explore = Node(
        package="explore_lite",
        executable="explore",
        name="explore_node",
        output="screen",
        parameters=[explore_params, {"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(args + [slam, navigation, explore])
