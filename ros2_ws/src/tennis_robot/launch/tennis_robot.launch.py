from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package="tennis_robot",
            executable="perception_node",
            name="perception_node",
            output="screen",
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
