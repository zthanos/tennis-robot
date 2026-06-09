"""Shared motion helpers for ROS 2 robot navigation.

This module intentionally keeps the behavior-level survey code away from raw
motor publishing details while still being usable in deterministic replay tests.
"""

from __future__ import annotations

import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

from tennis_robot import yaw_from_quaternion

MOTION_COMMAND_TOPIC = "/navigation/cmd_vel"
CMD_VEL_TOPIC = "/cmd_vel"
NAV_STATUS_TOPIC = "/navigation/status"


class Ros2MotorAdapter:
    """ROS 2 adapter that owns the actual /cmd_vel motor command topic."""

    def __init__(
        self,
        node: Node,
        command_topic: str = MOTION_COMMAND_TOPIC,
        cmd_vel_topic: str = CMD_VEL_TOPIC,
        status_topic: str = NAV_STATUS_TOPIC,
        command_timeout_s: float = 0.25,
    ) -> None:
        self._node = node
        self._command_timeout_s = command_timeout_s
        self._last_command = Twist()
        self._last_command_at = 0.0
        self._robot_x = 0.0
        self._robot_y = 0.0
        self._robot_yaw = 0.0

        self._pub_cmd_vel = node.create_publisher(Twist, cmd_vel_topic, 1)
        self._pub_status = node.create_publisher(String, status_topic, 10)
        node.create_subscription(Twist, command_topic, self._on_command, 1)
        node.create_subscription(Odometry, "/odom", self._on_odom, 10)
        node.create_timer(0.032, self._step)

    def _on_command(self, msg: Twist) -> None:
        self._last_command = msg
        self._last_command_at = time.time()

    def _on_odom(self, msg: Odometry) -> None:
        self._robot_x = msg.pose.pose.position.x
        self._robot_y = msg.pose.pose.position.y
        self._robot_yaw = yaw_from_quaternion(msg.pose.pose.orientation)

    def _step(self) -> None:
        command = self._last_command
        if time.time() - self._last_command_at > self._command_timeout_s:
            command = Twist()
        self._pub_cmd_vel.publish(command)
        self._publish_status(command)

    def _publish_status(self, command: Twist) -> None:
        msg = String()
        msg.data = (
            f'{{"cmd_linear_m_s":{command.linear.x:.3f},'
            f'"cmd_angular_rad_s":{command.angular.z:.3f},'
            f'"robot_x_m":{self._robot_x:.3f},'
            f'"robot_y_m":{self._robot_y:.3f},'
            f'"robot_yaw_rad":{self._robot_yaw:.4f}}}'
        )
        self._pub_status.publish(msg)
