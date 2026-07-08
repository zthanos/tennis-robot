import sys
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as RosImage


class Probe(Node):
    def __init__(self):
        super().__init__("intake_depth_probe")
        self.done = False
        self.create_subscription(RosImage, "/camera/intake_debug/depth", self._on_depth, 10)

    def _on_depth(self, msg):
        arr = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
        finite = arr[np.isfinite(arr)]
        finite = finite[(finite > 0.0)]
        if finite.size == 0:
            self.get_logger().info(f"DEPTH {msg.width}x{msg.height}: NO finite/positive values (all inf/nan/0) -> nothing in frustum")
        else:
            self.get_logger().info(
                f"DEPTH {msg.width}x{msg.height}: finite={finite.size}/{arr.size} "
                f"min={finite.min():.3f} max={finite.max():.3f} mean={finite.mean():.3f} "
                f"median={np.median(finite):.3f}"
            )
        self.done = True


def main():
    rclpy.init()
    node = Probe()
    start = node.get_clock().now()
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=0.2)
        if (node.get_clock().now() - start).nanoseconds > 10e9:
            node.get_logger().error("timed out waiting for depth image")
            break
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
