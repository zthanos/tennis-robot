"""ros2_control bring-up for the tennis robot.

Starts robot_state_publisher and spawns the controllers against the
controller_manager hosted by the gz_ros2_control plugin (sim) or a standalone
controller_manager (real robot).

Include this from sim.launch.py (after the robot is spawned into Gazebo) or run
standalone on the real robot. The actuation node (drive_actuator_node) is the
only thing that talks to the controller command topics.
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("tennis_robot")
    default_xacro = os.path.join(pkg_share, "urdf", "tennis_robot.urdf.xacro")
    default_controllers = os.path.join(pkg_share, "config", "controllers.yaml")

    sim_mode = LaunchConfiguration("sim_mode")
    controllers_config = LaunchConfiguration("controllers_config")
    xacro_file = LaunchConfiguration("xacro_file")

    args = [
        DeclareLaunchArgument("sim_mode", default_value="true"),
        DeclareLaunchArgument("controllers_config", default_value=default_controllers),
        DeclareLaunchArgument("xacro_file", default_value=default_xacro),
    ]

    # robot_description is rendered from xacro with the same sim_mode /
    # controllers_config so the <ros2_control> block matches the controllers.
    robot_description = {
        "robot_description": Command([
            "xacro ", xacro_file,
            " sim_mode:=", sim_mode,
            " controllers_config:=", controllers_config,
        ])
    }

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": True}],
    )

    # Spawners. joint_state_broadcaster first, then the command controllers.
    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    lift_wheel_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["lift_wheel_velocity_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # Chain: jsb -> diff_drive -> lift_wheel (avoids racing the controller_manager).
    chain = [
        robot_state_publisher,
        jsb_spawner,
        RegisterEventHandler(
            OnProcessExit(target_action=jsb_spawner, on_exit=[diff_drive_spawner])
        ),
        RegisterEventHandler(
            OnProcessExit(target_action=diff_drive_spawner, on_exit=[lift_wheel_spawner])
        ),
    ]

    return LaunchDescription(args + chain)
