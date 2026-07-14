"""Single owner of collector output: automatic commands plus manual override."""

from __future__ import annotations

import json
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64, Float64MultiArray, String

from tennis_robot.collector_driver import GazeboCollectorDriver, SerialCollectorDriver
from tennis_robot.collector_interface import CollectorInterface
from tennis_robot_msgs.msg import CollectorCmd


class CollectorLogicNode(Node):
    def __init__(self) -> None:
        super().__init__("collector_logic")
        backend = os.getenv("COLLECTOR_BACKEND", "gazebo")
        self._gazebo_pub = self.create_publisher(
            Float64MultiArray, "/intake_wheel_velocity_controller/commands", 10
        )
        if backend == "serial":
            driver = SerialCollectorDriver(
                os.getenv("COLLECTOR_SERIAL_PORT", "/dev/ttyACM0"),
                int(os.getenv("COLLECTOR_SERIAL_BAUD", "9600")),
            )
            self._collector = CollectorInterface(driver, default_speed=75.0, max_speed=255.0)
        else:
            # Dual-wheel intake: one motor per wheel, opposite spin so both
            # inner faces drive rearward. Forward intake speed v maps to
            # [left, right] = [-v, +v]; reverse (eject) flips both.
            driver = GazeboCollectorDriver(
                lambda speed: self._gazebo_pub.publish(
                    Float64MultiArray(data=[-speed, speed])
                )
            )
            self._collector = CollectorInterface(driver)
        self._status_pub = self.create_publisher(String, "/collector/status", 10)
        self._intake_beam_pub = self.create_publisher(
            Bool, "/collector/intake_beam_broken", 10
        )
        self.create_subscription(Float64, "/tennis_robot/cmd_collector", self._automatic, 10)
        self.create_subscription(CollectorCmd, "/collector/cmd", self._collector_command, 10)
        self.create_subscription(String, "/collector/manual_control", self._manual, 10)
        self.create_timer(0.5, self._publish_status)
        self.get_logger().info(f"collector logic ready (backend={backend})")

    def _automatic(self, msg: Float64) -> None:
        self._collector.apply_automatic(msg.data)

    def _collector_command(self, msg: CollectorCmd) -> None:
        self._collector.apply_automatic(msg.lift_wheel_speed if msg.intake_enabled else 0.0)

    def _manual(self, msg: String) -> None:
        try:
            action = str(json.loads(msg.data).get("action", ""))
        except (json.JSONDecodeError, AttributeError):
            return
        if action == "start":
            self._collector.start()
        elif action == "stop":
            self._collector.stop()
        elif action == "speed_up":
            self._collector.adjust_speed(2.0)
        elif action == "speed_down":
            self._collector.adjust_speed(-2.0)
        elif action == "release":
            self._collector.release_manual()
        self._publish_status()

    def _publish_status(self) -> None:
        status = self._collector.status
        self._status_pub.publish(String(data=json.dumps(status.__dict__, separators=(",", ":"))))
        if status.entry_beam_broken is not None:
            self._intake_beam_pub.publish(Bool(data=status.entry_beam_broken))


def main(args=None) -> None:
    rclpy.init(args=args)
    rclpy.spin(CollectorLogicNode())
    rclpy.shutdown()
