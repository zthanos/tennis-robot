"""Court Survey launch — brings up the full Court Knowledge Model stack.

Assumes the robot base is already running (sim.launch.py or real robot):
  /scan, /camera/image_raw, /camera/depth, /odom, TF map→odom must be live.

Starts:
  slam_mapping        (LiDAR SLAM — builds /map + map→odom TF)
  navigation          (Nav2 controller/planner/behaviors/bt_navigator)
  court_landmarks_node  (OAK-D landmark detection → /court_landmarks)
  court_survey_mission_node  (FSM brain → NavigateToPose → court_boundary.json)

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
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("tennis_robot")
    launch_dir = os.path.join(pkg_share, "launch")

    use_sim_time = LaunchConfiguration("use_sim_time")
    args = [DeclareLaunchArgument("use_sim_time", default_value="true")]

    # ── SLAM (map builder) ──────────────────────────────────────────────
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "slam_mapping.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    # ── Nav2 (path planner + BT navigator) ─────────────────────────────
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "navigation.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    # ── Camera landmark detector ────────────────────────────────────────
    court_landmarks = Node(
        package="tennis_robot",
        executable="court_landmarks_node",
        name="court_landmarks_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # ── Survey mission brain ────────────────────────────────────────────
    court_survey_mission = Node(
        package="tennis_robot",
        executable="court_survey_mission_node",
        name="court_survey_mission_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        # Use additional_env (not env=) so PYTHONPATH is inherited
        additional_env={
            "COURT_SURVEY_BT_XML": os.path.join(pkg_share, "config", "court_survey.xml"),
        },
    )

    return LaunchDescription(args + [slam, navigation, court_landmarks, court_survey_mission])
