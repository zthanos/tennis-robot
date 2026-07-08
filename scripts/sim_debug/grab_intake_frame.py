import sys
import numpy as np
from PIL import Image
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as RosImage


class Grabber(Node):
    def __init__(self, out_path: str):
        super().__init__("intake_frame_grabber")
        self.out_path = out_path
        self.done = False
        self.create_subscription(RosImage, "/camera/intake_debug/image_raw", self._on_image, 10)

    def _on_image(self, msg: RosImage) -> None:
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        Image.fromarray(arr, "RGB").save(self.out_path)
        self.get_logger().info(f"saved {self.out_path} ({msg.width}x{msg.height})")
        self.done = True


def main():
    out_path = sys.argv[1] if len(sys.argv) > 1 else "/tmp/intake_frame.png"
    rclpy.init()
    node = Grabber(out_path)
    start = node.get_clock().now()
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=0.2)
        if (node.get_clock().now() - start).nanoseconds > 10e9:
            node.get_logger().error("timed out waiting for image")
            break
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
