"""Court Survey launch — brings up the full Court Knowledge Model stack.

Assumes the robot base is already running (sim.launch.py or real robot):
  /scan, /camera/image_raw, /camera/depth, /odom, TF map→odom must be live.

Starts:
  navigation          (Nav2 controller/planner/behaviors/bt_navigator)
  court_survey_mission_node  (neural /survey/vision + LiDAR → court_boundary.json)

The primary perception node must already be running.  It owns both neural
models and publishes the timestamp-matched ``/survey/vision`` heartbeat.  This
launch deliberately does not start a second raw-camera landmark detector.

SLAM must already be publishing map→odom. The Docker Gazebo service starts
slam_mapping.launch.py already; pass start_slam:=true only for standalone use.

Usage (sim):
  docker compose --profile gazebo up gazebo
  ros2 launch tennis_robot court_survey.launch.py

Usage (real robot):
  ros2 launch tennis_robot court_survey.launch.py use_sim_time:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


WORKSPACE = os.environ.get("WORKSPACE", "/workspace")
SRC_PKG = os.path.join(WORKSPACE, "ros2_ws", "src", "tennis_robot")
SRC_MSGS = os.path.join(WORKSPACE, "ros2_ws", "src", "tennis_robot_msgs")


def _prefer_source(path_in_pkg: str, pkg_share: str) -> str:
    source_path = os.path.join(SRC_PKG, path_in_pkg)
    if os.path.exists(source_path):
        return source_path
    return os.path.join(pkg_share, path_in_pkg)


SOURCE_PYTHONPATH = ":".join(
    p for p in [
        SRC_PKG if os.path.exists(SRC_PKG) else "",
        SRC_MSGS if os.path.exists(SRC_MSGS) else "",
        os.environ.get("PYTHONPATH", ""),
    ] if p
)


def generate_launch_description():
    pkg_share = get_package_share_directory("tennis_robot")
    launch_dir = _prefer_source("launch", pkg_share)
    config_dir = _prefer_source("config", pkg_share)

    use_sim_time = LaunchConfiguration("use_sim_time")
    start_slam = LaunchConfiguration("start_slam")
    args = [
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "start_slam",
            default_value="false",
            description="Start slam_toolbox here. Keep false when Docker gazebo already runs slam_mapping.launch.py.",
        ),
    ]

    # ── SLAM (map builder) ──────────────────────────────────────────────
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "slam_mapping.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
        condition=IfCondition(start_slam),
    )

    # ── Nav2 (path planner + BT navigator) ─────────────────────────────
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "navigation.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": os.path.join(config_dir, "nav2_params.yaml"),
        }.items(),
    )

    # ── Survey mission brain ────────────────────────────────────────────
    court_survey_mission = Node(
        package="tennis_robot",
        executable="court_survey_mission_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        # Use additional_env (not env=) so PYTHONPATH is inherited
        additional_env={
            "PYTHONPATH": SOURCE_PYTHONPATH,
            "COURT_SURVEY_BT_XML": os.path.join(config_dir, "court_survey.xml"),
        },
    )

    return LaunchDescription(args + [slam, navigation, court_survey_mission])
