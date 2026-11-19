#!/usr/bin/env python3
"""P2-99c step (a) gates 1-2: LiDAR TF/frame and static /scan self-hit audit.

Run inside the Jazzy container while the sim is up (robot standing still):

    python3 scripts/sim_debug/p299c_scan_gate.py --out runtime/p299c/scan_gate

Gate 1: TF base_link -> lidar_link equals the expected D1 pose and every
        /scan header.frame_id is lidar_link.
Gate 2: over N scans, returns closer than --self-hit-m (finite) and
        below-range (-inf) returns are self-hit candidates; a beam that is a
        candidate in most scans is a blind angle. Writes JSON + a polar PNG.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


class ScanGate(Node):
    def __init__(self, scans: int) -> None:
        super().__init__("p299c_scan_gate")
        self.set_parameters([rclpy.parameter.Parameter(
            "use_sim_time", rclpy.Parameter.Type.BOOL, True)])
        self.want = scans
        self.scans: list[LaserScan] = []
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(LaserScan, "/scan", self._cb, qos_profile_sensor_data)

    def _cb(self, msg: LaserScan) -> None:
        if len(self.scans) < self.want:
            self.scans.append(msg)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--scans", type=int, default=50)
    ap.add_argument("--self-hit-m", type=float, default=0.20)
    ap.add_argument("--expect", type=float, nargs=3, default=(-0.316, 0.110, 0.345))
    ap.add_argument("--timeout-s", type=float, default=60.0)
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = ScanGate(args.scans)
    t0 = time.time()
    tf = None
    while time.time() - t0 < args.timeout_s and (len(node.scans) < args.scans or tf is None):
        rclpy.spin_once(node, timeout_sec=0.1)
        if tf is None:
            try:
                tf = node.tf_buffer.lookup_transform("base_link", "lidar_link", rclpy.time.Time())
            except Exception:  # noqa: BLE001 - keep waiting for TF
                tf = None
    scans = node.scans
    node.destroy_node()
    rclpy.shutdown()

    result: dict = {"scans": len(scans)}
    ok = True
    if tf is None:
        result["gate1"] = {"pass": False, "error": "no TF base_link->lidar_link"}
        ok = False
    else:
        t, q = tf.transform.translation, tf.transform.rotation
        xyz = [t.x, t.y, t.z]
        err = max(abs(a - b) for a, b in zip(xyz, args.expect))
        rot = max(abs(q.x), abs(q.y), abs(q.z))
        frames = sorted({s.header.frame_id for s in scans})
        g1 = err < 1e-4 and rot < 1e-6 and frames == ["lidar_link"]
        result["gate1"] = {"pass": g1, "tf_xyz_m": xyz, "expected_xyz_m": list(args.expect),
                           "max_abs_err_m": err, "quat_xyz": [q.x, q.y, q.z],
                           "scan_frame_ids": frames}
        ok &= g1
    if not scans:
        result["gate2"] = {"pass": False, "error": "no /scan received"}
        ok = False
    else:
        n = len(scans[0].ranges)
        a0, inc = scans[0].angle_min, scans[0].angle_increment
        close = [0] * n
        below = [0] * n
        none = [0] * n
        for s in scans:
            for i, r in enumerate(s.ranges[:n]):
                if math.isinf(r) and r < 0:
                    below[i] += 1
                elif math.isfinite(r) and r < args.self_hit_m:
                    close[i] += 1
                elif not math.isfinite(r):
                    none[i] += 1
        half = len(scans) / 2.0
        blind = [i for i in range(n) if close[i] + below[i] > half]
        deg = 360.0 / n
        result["gate2"] = {
            "pass": sum(close) == 0 and sum(below) == 0,
            "beams_per_scan": n, "angle_min_rad": a0, "angle_increment_rad": inc,
            "range_min_m": scans[0].range_min, "self_hit_threshold_m": args.self_hit_m,
            "finite_returns_below_threshold": sum(close),
            "below_range_min_returns": sum(below),
            "no_return_beams_max_fraction": max(none) / len(scans),
            "blind_beams": len(blind), "blind_deg": round(len(blind) * deg, 2),
            "blind_angles_deg": [round(math.degrees(a0 + i * inc), 1) for i in blind],
        }
        ok &= result["gate2"]["pass"]
        last = scans[-1]
        result["last_scan_ranges"] = [None if not math.isfinite(r) else round(r, 3) for r in last.ranges]
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
            fig = plt.figure(figsize=(7, 7))
            ax = fig.add_subplot(111, projection="polar")
            ang = [a0 + i * inc for i in range(n)]
            rr = [r if math.isfinite(r) else float("nan") for r in last.ranges[:n]]
            ax.scatter(ang, rr, s=3)
            for i in blind:
                ax.plot([ang[i], ang[i]], [0, 0.5], color="red", lw=1)
            ax.set_rmax(12.5)
            ax.set_theta_zero_location("N")
            ax.set_title(f"P2-99c /scan (frame lidar_link), blind={result['gate2']['blind_deg']} deg")
            fig.savefig(args.out / "scan_polar.png", dpi=110)
            fig2 = plt.figure(figsize=(7, 7))
            ax2 = fig2.add_subplot(111, projection="polar")
            ax2.scatter(ang, rr, s=4)
            ax2.set_rmax(1.0)
            ax2.set_theta_zero_location("N")
            ax2.set_title("near field (<=1.0 m)")
            fig2.savefig(args.out / "scan_polar_near.png", dpi=110)
        except Exception as exc:  # noqa: BLE001 - image is a convenience
            result["image_error"] = repr(exc)
    result["pass"] = bool(ok)
    (args.out / "scan_gate.json").write_text(json.dumps(result, indent=2))
    print(json.dumps({k: v for k, v in result.items() if k != "last_scan_ranges"}, indent=2))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
