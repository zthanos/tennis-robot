#!/usr/bin/env python3
"""P2-99c (b): OAK-D view on the court and ball detections from the new pose.

Saves one RGB frame (/camera/image_raw) as PNG, collects
/perception/ball_detections for --seconds, converts every spatial detection
to the world frame (TF base_link <- camera_link_optical_frame, robot standing
at the spawn pose) and matches it to the ball poses of the Gazebo world.
"""

from __future__ import annotations

import argparse
import json
import math
import re
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from PIL import Image as PILImage
from sensor_msgs.msg import Image
from tf2_ros import Buffer, TransformListener
from tennis_robot_msgs.msg import BallDetectionArray


def quat_to_mat(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--tag", required=True)
    ap.add_argument("--seconds", type=float, default=4.0)
    ap.add_argument("--world", type=Path, default=Path("gazebo/worlds/tennis_court.sdf"))
    ap.add_argument("--spawn", type=float, nargs=3, default=(-8.0, 0.0, 0.0))
    a = ap.parse_args()
    a.out.mkdir(parents=True, exist_ok=True)
    balls = [(float(x), float(y)) for x, y in re.findall(r'<model name="ball_\d+"><pose>([-\d.]+) ([-\d.]+) ', a.world.read_text())]
    rclpy.init()
    n = rclpy.create_node("p299c_oak_view_probe")
    n.set_parameters([rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)])
    st = {"img": None, "dets": []}
    n.create_subscription(Image, "/camera/image_raw", lambda m: st.__setitem__("img", m), 2)
    n.create_subscription(BallDetectionArray, "/perception/ball_detections", lambda m: st["dets"].append(m), 10)
    buf = Buffer(); TransformListener(buf, n)
    t0 = time.time()
    while time.time() - t0 < a.seconds or st["img"] is None:
        rclpy.spin_once(n, timeout_sec=0.05)
        if time.time() - t0 > 30:
            break
    res = {"tag": a.tag, "frames_with_detections": sum(1 for d in st["dets"] if d.detections),
           "frames": len(st["dets"])}
    m = st["img"]
    if m is not None:
        arr = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.width, -1)[..., :3]
        if m.encoding.lower().startswith("bgr"):
            arr = arr[..., ::-1]
        PILImage.fromarray(arr).save(a.out / f"oak_{a.tag}.png")
        res["image"] = f"oak_{a.tag}.png"
    try:
        tf = buf.lookup_transform("base_link", "camera_link_optical_frame", rclpy.time.Time())
        R = quat_to_mat(tf.transform.rotation)
        T = np.array([tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z])
    except Exception as exc:  # noqa: BLE001
        res["tf_error"] = repr(exc)
        R = None
    sx, sy, syaw = a.spawn
    matched = {}
    if R is not None:
        for arr_msg in st["dets"]:
            for d in arr_msg.detections:
                if not d.has_spatial:
                    continue
                pb = R @ np.array([d.position_x, d.position_y, d.position_z]) + T
                wx = sx + math.cos(syaw) * pb[0] - math.sin(syaw) * pb[1]
                wy = sy + math.sin(syaw) * pb[0] + math.cos(syaw) * pb[1]
                i = int(np.argmin([math.hypot(wx - bx, wy - by) for bx, by in balls]))
                matched.setdefault(i, []).append((math.hypot(wx - balls[i][0], wy - balls[i][1]),
                                                  math.hypot(balls[i][0] - sx, balls[i][1] - sy), d.confidence))
    res["balls_detected"] = [{"ball": f"ball_{i:02d}", "truth_xy": balls[i],
                              "range_from_robot_m": round(v[0][1], 2), "n": len(v),
                              "median_xy_error_m": round(float(np.median([e for e, _, _ in v])), 3),
                              "mean_conf": round(float(np.mean([c for _, _, c in v])), 2)}
                             for i, v in sorted(matched.items())]
    (a.out / f"oak_{a.tag}.json").write_text(json.dumps(res, indent=1))
    print(json.dumps(res))
    n.destroy_node(); rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
