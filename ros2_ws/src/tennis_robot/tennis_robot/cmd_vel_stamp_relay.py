"""Relay unstamped base commands to the Jazzy diff_drive_controller.

Every producer in the stack (drive_actuator_node, twist_mux, teleop) publishes
plain geometry_msgs/Twist on /diff_drive_controller/cmd_vel_unstamped, matching
the `use_stamped_vel: false` in controllers.yaml. On ROS 2 Jazzy that parameter
is gone: DiffDriveController only subscribes to TwistStamped on ~/cmd_vel, so
those commands land on a topic nobody reads and the base never moves (found
live in debug-log #27 for the bench, #43 for the collect_one flow).

This node is the single choke point that restamps them: Twist in, TwistStamped
out with a live clock stamp (a zero/stale stamp is treated as expired and the
command is zeroed by the controller).
"""

from __future__ import annotations

import os

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node


class CmdVelStampRelay(Node):
    def __init__(self) -> None:
        super().__init__("cmd_vel_stamp_relay")
        in_topic = os.environ.get(
            "CMD_VEL_RELAY_IN", "/diff_drive_controller/cmd_vel_unstamped"
        )
        out_topic = os.environ.get(
            "CMD_VEL_RELAY_OUT", "/diff_drive_controller/cmd_vel"
        )
        self._pub = self.create_publisher(TwistStamped, out_topic, 10)
        self.create_subscription(Twist, in_topic, self._on_twist, 10)
        self.get_logger().info(f"restamping {in_topic} -> {out_topic}")

    def _on_twist(self, msg: Twist) -> None:
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "base_footprint"
        out.twist = msg
        self._pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    rclpy.spin(CmdVelStampRelay())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
