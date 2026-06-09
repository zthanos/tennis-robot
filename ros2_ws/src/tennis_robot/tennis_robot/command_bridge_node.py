"""Bridges the file-based IPC (runtime/robot_command.json) to/from ROS 2 topics.

This lets the existing web control panel (scripts/control_panel.py) work
unchanged while the controller moves to ROS 2. On real hardware this node
can be replaced with a proper UI that publishes directly to /robot/command.

Publishes:
  /robot/command  (tennis_robot_msgs/RobotCommand)  — polls command file

Subscribes:
  /robot/status   (std_msgs/String, JSON)  — writes to runtime/robot_status.json
"""

from __future__ import annotations

import json
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from tennis_robot.control_bus import RobotCommandStore, RobotStatusStore
from tennis_robot_msgs.msg import RobotCommand

POLL_INTERVAL_S = 0.05
REPUBLISH_INTERVAL_S = 0.5


class CommandBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("command_bridge")

        self._command_store = RobotCommandStore.from_env()
        self._status_store = RobotStatusStore.from_env()
        self._last_sequence = -1
        self._last_publish_s = 0.0

        self._pub_command = self.create_publisher(RobotCommand, "/robot/command", 10)
        self.create_subscription(String, "/robot/status", self._on_status, 10)
        self.create_timer(POLL_INTERVAL_S, self._poll_command_file)

        self.get_logger().info("command_bridge started")

    def _poll_command_file(self) -> None:
        cmd = self._command_store.read()
        now = time.time()
        should_publish = cmd.sequence != self._last_sequence
        if not should_publish and cmd.mode != "idle":
            should_publish = now - self._last_publish_s >= REPUBLISH_INTERVAL_S
        if not should_publish:
            return
        self._last_sequence = cmd.sequence
        self._last_publish_s = now
        msg = RobotCommand()
        msg.mode = cmd.mode
        msg.source = cmd.source
        self._pub_command.publish(msg)
        self.get_logger().debug(f"command → {cmd.mode} (seq={cmd.sequence})")

    def _on_status(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            return
        self._status_store.write(payload)


def main(args=None) -> None:
    rclpy.init(args=args)
    rclpy.spin(CommandBridgeNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
