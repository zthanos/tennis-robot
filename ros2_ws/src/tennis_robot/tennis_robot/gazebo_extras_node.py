"""Gazebo adapter node: IR LaserScan → IrReadings, pose info → /sim/balls.

Subscribes:
  /gz/ir_left/scan   (sensor_msgs/LaserScan, 1 ray)
  /gz/ir_right/scan  (sensor_msgs/LaserScan, 1 ray)
  /gz/pose_info      (tf2_msgs/TFMessage — world model poses from Gazebo)

Publishes:
  /ir/readings       (tennis_robot_msgs/IrReadings)
  /sim/balls         (std_msgs/String, JSON list of {name, x, y})
"""

from __future__ import annotations

import json
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

from tennis_robot_msgs.msg import IrReadings

# IR beam-break: range <= threshold means a ball is present → value 1000
_IR_MAX_RANGE_M = 0.22
_BALL_PREFIX = "ball_"


def _range_to_ir_value(range_m: float) -> float:
    """Convert a proximity range reading to a 0–1000 IR value (1000 = object present)."""
    if not math.isfinite(range_m) or range_m > _IR_MAX_RANGE_M:
        return 0.0
    return 1000.0 * max(0.0, 1.0 - range_m / _IR_MAX_RANGE_M)


class GazeboExtrasNode(Node):
    def __init__(self) -> None:
        super().__init__("gazebo_extras_node")

        self._ir_left = 0.0
        self._ir_right = 0.0
        self._balls: list[dict] = []

        self.create_subscription(LaserScan, "/gz/ir_left/scan", self._on_ir_left, 10)
        self.create_subscription(LaserScan, "/gz/ir_right/scan", self._on_ir_right, 10)
        self.create_subscription(TFMessage, "/gz/pose_info", self._on_pose_info, 10)

        self._pub_ir = self.create_publisher(IrReadings, "/ir/readings", 10)
        self._pub_balls = self.create_publisher(String, "/sim/balls", 1)

        self.create_timer(0.05, self._publish)
        self.get_logger().info("gazebo_extras_node started")

    def _on_ir_left(self, msg: LaserScan) -> None:
        r = msg.ranges[0] if msg.ranges else float("inf")
        self._ir_left = _range_to_ir_value(r)

    def _on_ir_right(self, msg: LaserScan) -> None:
        r = msg.ranges[0] if msg.ranges else float("inf")
        self._ir_right = _range_to_ir_value(r)

    def _on_pose_info(self, msg: TFMessage) -> None:
        balls: list[dict] = []
        for transform in msg.transforms:
            name = transform.child_frame_id
            if not name.startswith(_BALL_PREFIX):
                continue
            t = transform.transform.translation
            balls.append({"def": name, "x": round(t.x, 4), "y": round(t.y, 4)})
        self._balls = balls

    def _publish(self) -> None:
        ir_msg = IrReadings()
        ir_msg.left = self._ir_left
        ir_msg.right = self._ir_right
        self._pub_ir.publish(ir_msg)

        balls_msg = String()
        balls_msg.data = json.dumps(self._balls)
        self._pub_balls.publish(balls_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GazeboExtrasNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
