"""Dedicated ROS 2 navigation/motor adapter node."""

from __future__ import annotations

import os

import rclpy
from rclpy.node import Node

from tennis_robot.motion_controller import Ros2MotorAdapter


class NavigationNode(Node):
    def __init__(self) -> None:
        super().__init__("navigation_node")
        timeout_s = float(os.getenv("NAVIGATION_COMMAND_TIMEOUT_S", "0.25"))
        self._adapter = Ros2MotorAdapter(self, command_timeout_s=timeout_s)
        self.get_logger().info("navigation_node started")


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = NavigationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
