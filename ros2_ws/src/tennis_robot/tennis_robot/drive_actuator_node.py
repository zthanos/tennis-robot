"""Actuation layer for the ros2_control stack.

This is the ONLY node that talks to the ros2_control command topics. Everything
above it (planning, target selection, collection state machine) speaks a neutral
robot-level command contract and stays unaware of which controllers or hardware
back-end are in use. To move from Gazebo to the real robot, nothing here changes.

Neutral command contract (subscribed):
    /tennis_robot/cmd_drive      geometry_msgs/Twist     base velocity command
    /tennis_robot/cmd_collector  std_msgs/Float64        intake roller velocity (rad/s)

ros2_control contract (published):
    /diff_drive_controller/cmd_vel_unstamped       geometry_msgs/Twist
    /lift_wheel_velocity_controller/commands       std_msgs/Float64MultiArray

A watchdog stops the robot if no drive command arrives within
DRIVE_TIMEOUT_S, so a stalled upstream node can never leave the base driving.

All topic names and the timeout are tunable via environment variables, matching
the project's "configurable without code changes" constraint.
"""

from __future__ import annotations

import os

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray


def _env(name: str, default: str) -> str:
    return os.environ.get(name, default)


def _env_float(name: str, default: float) -> float:
    try:
        return float(os.environ.get(name, default))
    except (TypeError, ValueError):
        return default


class DriveActuatorNode(Node):
    def __init__(self) -> None:
        super().__init__("drive_actuator_node")

        # Neutral (upstream) command topics
        cmd_drive_topic = _env("ACTUATOR_CMD_DRIVE_TOPIC", "/tennis_robot/cmd_drive")
        cmd_collector_topic = _env("ACTUATOR_CMD_COLLECTOR_TOPIC", "/tennis_robot/cmd_collector")

        # ros2_control command topics
        diff_drive_topic = _env(
            "ACTUATOR_DIFF_DRIVE_TOPIC", "/diff_drive_controller/cmd_vel_unstamped"
        )
        lift_wheel_topic = _env(
            "ACTUATOR_LIFT_WHEEL_TOPIC", "/lift_wheel_velocity_controller/commands"
        )

        self._drive_timeout_s = _env_float("ACTUATOR_DRIVE_TIMEOUT_S", 0.5)

        # Publishers to ros2_control
        self._drive_pub = self.create_publisher(Twist, diff_drive_topic, 10)
        self._lift_pub = self.create_publisher(Float64MultiArray, lift_wheel_topic, 10)

        # Subscriptions from the behavior stack
        self.create_subscription(Twist, cmd_drive_topic, self._on_drive_cmd, 10)
        self.create_subscription(Float64, cmd_collector_topic, self._on_collector_cmd, 10)

        # Watchdog: stop the base if upstream goes silent
        self._last_drive_stamp = self.get_clock().now()
        self._drive_active = False
        self.create_timer(0.1, self._watchdog_tick)

        self.get_logger().info(
            "drive_actuator_node up: "
            f"{cmd_drive_topic} -> {diff_drive_topic}, "
            f"{cmd_collector_topic} -> {lift_wheel_topic}, "
            f"watchdog={self._drive_timeout_s}s"
        )

    def _on_drive_cmd(self, msg: Twist) -> None:
        self._last_drive_stamp = self.get_clock().now()
        self._drive_active = True
        self._drive_pub.publish(msg)

    def _on_collector_cmd(self, msg: Float64) -> None:
        self._lift_pub.publish(Float64MultiArray(data=[float(msg.data)]))

    def _watchdog_tick(self) -> None:
        if not self._drive_active:
            return
        elapsed = (self.get_clock().now() - self._last_drive_stamp).nanoseconds * 1e-9
        if elapsed > self._drive_timeout_s:
            self._drive_pub.publish(Twist())  # all-zero stop
            self._drive_active = False
            self.get_logger().warn(
                f"drive watchdog: no command for {elapsed:.2f}s, stopping base"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DriveActuatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
