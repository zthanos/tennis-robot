"""Minimal physical-drive bring-up: control panel, mux, and Mega bridge only."""

import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    repository_root = os.getenv(
        "TENNIS_ROBOT_ROOT",
        os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../..")),
    )
    manual_linear_speed = os.getenv("MANUAL_LINEAR_SPEED_M_S", "0.20")
    manual_turn_speed = os.getenv("MANUAL_TURN_SPEED_RAD_S", "0.40")
    shared_environment = {
        "MANUAL_LINEAR_SPEED_M_S": manual_linear_speed,
        "MANUAL_TURN_SPEED_RAD_S": manual_turn_speed,
        "ROBOT_COMMAND_FILE": f"{repository_root}/runtime/robot_command.json",
        "ROBOT_STATUS_FILE": f"{repository_root}/runtime/robot_status.json",
    }

    controller = Node(
        package="tennis_robot",
        executable="controller_node",
        name="controller_node",
        output="screen",
        additional_env=shared_environment,
    )
    navigation = Node(
        package="tennis_robot",
        executable="navigation_node",
        name="navigation_node",
        output="screen",
    )
    command_bridge = Node(
        package="tennis_robot",
        executable="command_bridge_node",
        name="command_bridge_node",
        output="screen",
        additional_env=shared_environment,
    )
    stamp_relay = Node(
        package="tennis_robot",
        executable="cmd_vel_stamp_relay",
        name="drive_test_stamp_relay",
        output="screen",
        additional_env={
            "CMD_VEL_RELAY_IN": "/cmd_vel_collection",
            "CMD_VEL_RELAY_OUT": "/cmd_vel_collection_stamped",
        },
    )
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        name="twist_mux",
        output="screen",
        parameters=[
            f"{repository_root}/ros2_ws/src/tennis_robot/config/"
            "twist_mux_drive_test.yaml"
        ],
        remappings=[("cmd_vel_out", "/drive_test/cmd_vel")],
    )
    serial_bridge = Node(
        package="tennis_robot",
        executable="base_serial_bridge",
        name="base_serial_bridge",
        output="screen",
        parameters=[{
            "port": os.getenv("DRIVE_SERIAL_PORT", "/dev/ttyACM0"),
            "baud": int(os.getenv("DRIVE_SERIAL_BAUD", "500000")),
            "counts_per_rev": float(os.environ["DRIVE_COUNTS_PER_REV"]),
            "wheel_radius_m": float(os.getenv("DRIVE_WHEEL_RADIUS_M", "0.085")),
            "track_m": 0.585,
            "pwm_ceiling": float(os.getenv("DRIVE_PWM_CEILING", "1.0")),
            "no_load_speed_rad_s": float(
                os.getenv("DRIVE_NOLOAD_RAD_S", "12.775810")
            ),
            "max_linear_m_s": float(manual_linear_speed),
            "max_angular_rad_s": float(manual_turn_speed),
        }],
    )
    control_panel = ExecuteProcess(
        cmd=[
            "python3",
            f"{repository_root}/scripts/control_panel.py",
            "--host",
            "0.0.0.0",
            "--port",
            os.getenv("CONTROL_PANEL_PORT", "8081"),
        ],
        output="screen",
        additional_env=shared_environment,
    )

    return LaunchDescription([
        controller,
        navigation,
        command_bridge,
        stamp_relay,
        twist_mux,
        serial_bridge,
        control_panel,
    ])
