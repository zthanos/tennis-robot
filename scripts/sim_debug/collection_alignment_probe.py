#!/usr/bin/env python3
"""Read-only collection perception alignment probe for distributed Gazebo.

The probe subscribes to the canonical BallDetectionArray and Gazebo truth,
then compares every spatial detection with the nearest truth ball in the
camera optical frame at the RGB timestamp.  It publishes nothing and has no
planner/controller dependency.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import time


def transform_point(
    point_xyz: tuple[float, float, float],
    translation_xyz: tuple[float, float, float],
    rotation_xyzw: tuple[float, float, float, float],
) -> tuple[float, float, float]:
    """Apply one quaternion rigid transform to a 3D point."""
    x, y, z = point_xyz
    tx, ty, tz = translation_xyz
    qx, qy, qz, qw = rotation_xyzw
    return (
        tx
        + (1 - 2 * (qy * qy + qz * qz)) * x
        + 2 * (qx * qy - qw * qz) * y
        + 2 * (qx * qz + qw * qy) * z,
        ty
        + 2 * (qx * qy + qw * qz) * x
        + (1 - 2 * (qx * qx + qz * qz)) * y
        + 2 * (qy * qz - qw * qx) * z,
        tz
        + 2 * (qx * qz - qw * qy) * x
        + 2 * (qy * qz + qw * qx) * y
        + (1 - 2 * (qx * qx + qy * qy)) * z,
    )


def associate_nearest(
    measured_xyz: tuple[float, float, float],
    truth_camera_xyz: dict[str, tuple[float, float, float]],
    *,
    gate_m: float,
    ambiguity_margin_m: float,
) -> dict:
    """Return deterministic nearest-truth association evidence."""
    ranked = sorted(
        (
            math.dist(measured_xyz, truth_xyz),
            ball_id,
            truth_xyz,
        )
        for ball_id, truth_xyz in truth_camera_xyz.items()
    )
    if not ranked or ranked[0][0] > gate_m:
        return {
            "status": "unmatched",
            "ball_id": ranked[0][1] if ranked else None,
            "distance_m": ranked[0][0] if ranked else None,
        }
    if (
        len(ranked) > 1
        and ranked[1][0] - ranked[0][0] < ambiguity_margin_m
    ):
        return {
            "status": "ambiguous",
            "ball_id": ranked[0][1],
            "distance_m": ranked[0][0],
            "second_distance_m": ranked[1][0],
        }
    distance_m, ball_id, truth_xyz = ranked[0]
    residual = tuple(
        measured_xyz[index] - truth_xyz[index] for index in range(3)
    )
    return {
        "status": "associated",
        "ball_id": ball_id,
        "distance_m": distance_m,
        "truth_camera_xyz_m": truth_xyz,
        "residual_camera_xyz_m": residual,
    }


def _stamp_s(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _transform_payload(transform) -> tuple[
    tuple[float, float, float],
    tuple[float, float, float, float],
    dict,
]:
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    translation_xyz = (translation.x, translation.y, translation.z)
    rotation_xyzw = (rotation.x, rotation.y, rotation.z, rotation.w)
    payload = {
        "translation_xyz_m": translation_xyz,
        "rotation_xyzw": rotation_xyzw,
        "stamp_s": _stamp_s(transform.header.stamp),
        "target_frame": transform.header.frame_id,
        "source_frame": transform.child_frame_id,
    }
    return translation_xyz, rotation_xyzw, payload


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--duration-s", type=float, default=45.0)
    parser.add_argument("--max-records", type=int, default=5000)
    parser.add_argument("--association-gate-m", type=float, default=0.50)
    parser.add_argument("--ambiguity-margin-m", type=float, default=0.02)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if args.duration_s <= 0 or args.max_records <= 0:
        raise SystemExit("duration and max-records must be positive")
    if args.association_gate_m <= 0 or args.ambiguity_margin_m < 0:
        raise SystemExit("invalid association thresholds")

    import rclpy
    from rclpy.duration import Duration
    from rclpy.node import Node
    from rclpy.time import Time
    from std_msgs.msg import String
    from tennis_robot_msgs.msg import BallDetectionArray
    from tf2_ros import Buffer, TransformListener

    class AlignmentProbe(Node):
        def __init__(self) -> None:
            super().__init__("collection_alignment_probe")
            args.output.parent.mkdir(parents=True, exist_ok=True)
            self.output = args.output.open("w", encoding="utf-8")
            self.truth: list[dict] = []
            self.records = 0
            self.started = time.monotonic()
            self.complete = False
            self.buffer = Buffer()
            self.listener = TransformListener(self.buffer, self)
            self.create_subscription(String, "/sim/balls", self.on_truth, 10)
            self.create_subscription(
                BallDetectionArray,
                "/perception/ball_detections",
                self.on_detections,
                10,
            )

        def on_truth(self, message: String) -> None:
            try:
                decoded = json.loads(message.data)
                self.truth = [
                    item
                    for item in decoded
                    if isinstance(item, dict)
                    and {"def", "x", "y", "z"} <= set(item)
                ]
            except (json.JSONDecodeError, TypeError):
                self.truth = []

        def on_detections(self, frame: BallDetectionArray) -> None:
            if self.complete or not self.truth:
                return
            rgb_stamp_s = _stamp_s(frame.header.stamp)
            try:
                camera_from_odom = self.buffer.lookup_transform(
                    frame.header.frame_id,
                    "odom",
                    Time.from_msg(frame.header.stamp),
                    timeout=Duration(seconds=0.1),
                )
                map_from_camera = self.buffer.lookup_transform(
                    "map",
                    frame.header.frame_id,
                    Time.from_msg(frame.header.stamp),
                    timeout=Duration(seconds=0.1),
                )
            except Exception as exc:
                self.write(
                    {
                        "schema_version": 1,
                        "event": "tf_rejected",
                        "rgb_stamp_s": rgb_stamp_s,
                        "detail": str(exc),
                    }
                )
                return
            translation, rotation, camera_tf_payload = _transform_payload(
                camera_from_odom
            )
            _, _, map_tf_payload = _transform_payload(map_from_camera)
            truth_camera = {
                str(item["def"]): transform_point(
                    (float(item["x"]), float(item["y"]), float(item["z"])),
                    translation,
                    rotation,
                )
                for item in self.truth
            }
            for detection_index, detection in enumerate(frame.detections):
                if not detection.has_spatial:
                    continue
                measured = (
                    float(detection.position_x),
                    float(detection.position_y),
                    float(detection.position_z),
                )
                association = associate_nearest(
                    measured,
                    truth_camera,
                    gate_m=args.association_gate_m,
                    ambiguity_margin_m=args.ambiguity_margin_m,
                )
                self.write(
                    {
                        "schema_version": 1,
                        "event": "spatial_detection",
                        "rgb_stamp_s": rgb_stamp_s,
                        "matched_depth_stamp_s": _stamp_s(
                            detection.matched_depth_stamp
                        ),
                        "rgb_depth_delta_s": abs(
                            rgb_stamp_s
                            - _stamp_s(detection.matched_depth_stamp)
                        ),
                        "detection_index": detection_index,
                        "confidence": float(detection.confidence),
                        "bbox": {
                            "center_x": float(detection.bbox_center_x),
                            "center_y": float(detection.bbox_center_y),
                            "width": float(detection.bbox_width),
                            "height": float(detection.bbox_height),
                        },
                        "bearing_rad": float(detection.bearing_rad),
                        "distance_m": float(detection.distance_m),
                        "measured_camera_xyz_m": measured,
                        "association": association,
                        "camera_from_odom": camera_tf_payload,
                        "map_from_camera": map_tf_payload,
                    }
                )

        def write(self, payload: dict) -> None:
            self.output.write(json.dumps(payload, sort_keys=True) + "\n")
            self.output.flush()
            self.records += 1
            if self.records >= args.max_records:
                self.complete = True

        def close(self) -> None:
            self.output.close()

    rclpy.init()
    node = AlignmentProbe()
    try:
        while (
            rclpy.ok()
            and not node.complete
            and time.monotonic() - node.started < args.duration_s
        ):
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()
    print(
        json.dumps(
            {
                "output": str(args.output),
                "records": node.records,
                "duration_s": args.duration_s,
            },
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
