"""Continuously dump intake debug camera frames as timestamped PNGs.

Kept OUT of the rosbag on purpose: 31 Hz raw RGB would be ~7 GB per test run;
0.5 s PNGs are ~30 MB and the analyzer matches them to the bag by wall time.
"""
import os
import sys
import time

import numpy as np
from PIL import Image
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as RosImage

OUT_DIR = sys.argv[1] if len(sys.argv) > 1 else "/tmp/intake_frames"
PERIOD_S = 0.5


class Dumper(Node):
    def __init__(self):
        super().__init__("intake_frame_dumper")
        os.makedirs(OUT_DIR, exist_ok=True)
        self._last = 0.0
        self.create_subscription(RosImage, "/camera/intake_debug/image_raw", self._on_image, 10)

    def _on_image(self, msg):
        now = time.time()
        if now - self._last < PERIOD_S:
            return
        self._last = now
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        Image.fromarray(arr, "RGB").save(os.path.join(OUT_DIR, f"f_{now:.2f}.png"))


def main():
    rclpy.init()
    node = Dumper()
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
