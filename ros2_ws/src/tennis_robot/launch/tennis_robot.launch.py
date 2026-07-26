import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    workspace = os.environ.get("TENNIS_ROBOT_ROOT", "/workspace")
    return LaunchDescription([
        Node(
            package="tennis_robot",
            executable="perception_node",
            name="perception_node",
            output="screen",
            additional_env={
                "PERCEPTION_CALIBRATION_PLATFORM": "oak_d",
                "BALL_MODEL_PATH": os.getenv(
                    "BALL_MODEL_PATH", f"{workspace}/models/yolov8n.onnx"
                ),
                "COURT_SCENE_MODEL_PATH": os.getenv(
                    "COURT_SCENE_MODEL_PATH",
                    f"{workspace}/models/court_scene_yolov8n.onnx",
                ),
                "COURT_SCENE_CLASS_MAP": os.getenv(
                    "COURT_SCENE_CLASS_MAP", "0:net,1:fence"
                ),
            },
        ),
        Node(
            package="tennis_robot",
            executable="controller_node",
            name="tennis_robot_controller",
            output="screen",
        ),
        Node(
            package="tennis_robot",
            executable="navigation_node",
            name="navigation_node",
            output="screen",
        ),
        Node(
            package="tennis_robot",
            executable="command_bridge_node",
            name="command_bridge",
            output="screen",
        ),
    ])
