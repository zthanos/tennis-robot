"""Launch Gazebo Harmonic + ros_gz bridge + all ROS 2 nodes."""

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# WORKSPACE env var is set in Dockerfile.gazebo and docker-compose.yml.
# Fallback: resolve from the source tree (works when run directly from gazebo/launch/).
WORKSPACE = os.environ.get(
    "WORKSPACE",
    str(Path(__file__).resolve().parents[2]),  # gazebo/launch/../.. = project root
)
GZ_MODELS = f"{WORKSPACE}/gazebo/models"
GZ_WORLD = f"{WORKSPACE}/gazebo/worlds/tennis_court.sdf"
BRIDGE_CONFIG = f"{WORKSPACE}/gazebo/bridge_config.yaml"
CONTROLLERS_PATH = f"{WORKSPACE}/controllers/ball_detector"


def generate_launch_description():
    headless_arg = DeclareLaunchArgument(
        "headless", default_value="false",
        description="Run Gazebo without GUI (headless mode)"
    )
    headless = LaunchConfiguration("headless")

    # ── Gazebo ──────────────────────────────────────────────────────────────
    gz_server = ExecuteProcess(
        cmd=["gz", "sim", "-r", "-s", GZ_WORLD],
        additional_env={"GZ_SIM_RESOURCE_PATH": GZ_MODELS},
        output="screen",
    )

    gz_gui = ExecuteProcess(
        cmd=["gz", "sim", "-g"],
        output="screen",
    )

    # ── ros_gz bridge ────────────────────────────────────────────────────────
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_bridge",
        parameters=[{"config_file": BRIDGE_CONFIG}],
        output="screen",
    )

    # ── ROS 2 nodes (same as before — interface unchanged) ──────────────────
    perception = Node(
        package="tennis_robot",
        executable="perception_node",
        name="perception_node",
        output="screen",
        additional_env={"PYTHONPATH": CONTROLLERS_PATH},
    )

    controller = Node(
        package="tennis_robot",
        executable="controller_node",
        name="controller_node",
        output="screen",
        additional_env={
            "PYTHONPATH": CONTROLLERS_PATH,
            "ROBOT_COMMAND_FILE": f"{WORKSPACE}/runtime/robot_command.json",
        },
    )

    command_bridge = Node(
        package="tennis_robot",
        executable="command_bridge_node",
        name="command_bridge_node",
        output="screen",
        additional_env={
            "ROBOT_COMMAND_FILE": f"{WORKSPACE}/runtime/robot_command.json",
        },
    )

    # Converts IR LaserScan + pose info → /ir/readings + /sim/balls
    gz_extras = Node(
        package="tennis_robot",
        executable="gazebo_extras_node",
        name="gazebo_extras_node",
        output="screen",
    )

    # Web control panel — http://localhost:8081
    control_panel = ExecuteProcess(
        cmd=["python3", f"{WORKSPACE}/scripts/control_panel.py"],
        additional_env={
            "PYTHONPATH": f"{CONTROLLERS_PATH}:{WORKSPACE}/scripts",
            "ROBOT_COMMAND_FILE": f"{WORKSPACE}/runtime/robot_command.json",
            "ROBOT_STATUS_FILE": f"{WORKSPACE}/runtime/robot_status.json",
        },
        output="screen",
    )

    # Delay ROS nodes until Gazebo + bridge are up
    delayed_nodes = TimerAction(
        period=4.0,
        actions=[bridge, perception, controller, command_bridge, gz_extras],
    )

    return LaunchDescription([
        headless_arg,
        gz_server,
        gz_gui,
        control_panel,
        delayed_nodes,
    ])
