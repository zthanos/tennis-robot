#!/usr/bin/env python3
"""Print the raw aligned-depth samples used by each live ball detection."""

from __future__ import annotations

import json

import message_filters
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from tennis_robot_msgs.msg import BallDetectionArray

from tennis_robot.perception import depth_fusion_roi_bounds


class DepthRoiInspector(Node):
    def __init__(self) -> None:
        super().__init__("ball_depth_roi_inspector")
        self._done = False
        detections = message_filters.Subscriber(
            self, BallDetectionArray, "/perception/ball_detections", qos_profile=1
        )
        depth = message_filters.Subscriber(
            self, Image, "/camera/depth", qos_profile=1
        )
        sync = message_filters.ApproximateTimeSynchronizer(
            [detections, depth], queue_size=10, slop=0.01
        )
        sync.registerCallback(self._inspect)
        self._sync = sync

    def _inspect(self, detections: BallDetectionArray, depth_msg: Image) -> None:
        depth = np.frombuffer(bytes(depth_msg.data), dtype=np.float32).reshape(
            (depth_msg.height, depth_msg.width)
        )
        records = []
        for index, detection in enumerate(detections.detections):
            proxy = type(
                "DetectionBox",
                (),
                {
                    "center_x": detection.bbox_center_x,
                    "center_y": detection.bbox_center_y,
                    "width": detection.bbox_width,
                    "height": detection.bbox_height,
                },
            )()
            x0, x1, y0, y1 = depth_fusion_roi_bounds(
                proxy, depth, depth_msg.width, depth_msg.height
            )
            roi = depth[y0:y1, x0:x1]
            valid = np.sort(roi[np.isfinite(roi) & (roi > 0)])
            records.append(
                {
                    "index": index,
                    "bbox": [
                        float(detection.bbox_center_x),
                        float(detection.bbox_center_y),
                        float(detection.bbox_width),
                        float(detection.bbox_height),
                    ],
                    "roi": [x0, x1, y0, y1],
                    "published_has_spatial": bool(detection.has_spatial),
                    "published_distance_m": (
                        float(detection.distance_m)
                        if detection.has_spatial
                        else None
                    ),
                    "valid_depth_m": [round(float(value), 4) for value in valid],
                    "percentiles_m": (
                        {
                            str(percentile): round(
                                float(np.percentile(valid, percentile)), 4
                            )
                            for percentile in (0, 20, 40, 50, 60, 80, 100)
                        }
                        if valid.size
                        else {}
                    ),
                }
            )
        print(json.dumps(records, indent=2, sort_keys=True), flush=True)
        self._done = True


def main() -> None:
    rclpy.init()
    node = DepthRoiInspector()
    while rclpy.ok() and not node._done:
        rclpy.spin_once(node, timeout_sec=2.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
