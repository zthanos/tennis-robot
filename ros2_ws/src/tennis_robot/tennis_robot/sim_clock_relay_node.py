"""Publish a bounded-rate ROS simulation clock from Gazebo's physics clock.

Gazebo advances physics at 500 Hz for stable wheel/contact simulation. ROS
control and navigation do not need 500 clock updates per second; forwarding
every update across DDS wakes every ``use_sim_time`` node and needlessly loads
the Pi and the LAN endpoint. Sensor messages retain their original Gazebo
timestamps, so this relay changes clock notification rate, not sensor data.
"""

from __future__ import annotations

import math
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rosgraph_msgs.msg import Clock


def configured_publish_hz() -> float:
    raw = os.getenv("SIM_CLOCK_PUBLISH_HZ", "50")
    try:
        value = float(raw)
    except ValueError as exc:
        raise ValueError(f"SIM_CLOCK_PUBLISH_HZ must be numeric, got {raw!r}") from exc
    if not math.isfinite(value) or not 1.0 <= value <= 200.0:
        raise ValueError(
            f"SIM_CLOCK_PUBLISH_HZ must be finite and within [1, 200], got {raw!r}"
        )
    return value


class SimClockRelayNode(Node):
    def __init__(self) -> None:
        super().__init__("sim_clock_relay")
        publish_hz = configured_publish_hz()
        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            # ROS clock consumers (controller_manager and domain_bridge) request
            # reliable delivery. At the bounded 50 Hz rate this cannot recreate
            # the former 500 Hz backpressure problem.
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self._latest: Clock | None = None
        self._last_published_ns: int | None = None
        self._publisher = self.create_publisher(Clock, "/clock", qos)
        self._subscription = self.create_subscription(
            Clock, "/clock_raw", self._on_raw_clock, qos
        )
        self.create_timer(1.0 / publish_hz, self._publish_latest)
        self.get_logger().info(
            f"sim clock relay ready (/clock_raw -> /clock at <= {publish_hz:g} Hz)"
        )

    def _on_raw_clock(self, message: Clock) -> None:
        self._latest = message

    def _publish_latest(self) -> None:
        message = self._latest
        if message is None:
            return
        stamp_ns = int(message.clock.sec) * 1_000_000_000 + int(message.clock.nanosec)
        if stamp_ns == self._last_published_ns:
            return
        self._publisher.publish(message)
        self._last_published_ns = stamp_ns


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimClockRelayNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
