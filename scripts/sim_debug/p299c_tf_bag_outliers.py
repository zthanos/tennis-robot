#!/usr/bin/env python3
"""P2-99c: find TF outliers in a bag recorded with --use-sim-time.

For every /tf transform touching map / odom / base_footprint / base_link it
computes stamp - receive_time (the bag receive time is sim time) and reports
per (parent, child): count, stamp range, offset range, and every message whose
offset exceeds --limit-s, plus the first/last stamps. Run inside the Jazzy
container: python3 p299c_tf_bag_outliers.py <bag_dir> [--freeze-s 37.0]
"""

from __future__ import annotations

import argparse
import collections
import json

import rosbag2_py
from rclpy.serialization import deserialize_message
from tf2_msgs.msg import TFMessage

FRAMES = {"map", "odom", "base_footprint", "base_link"}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--limit-s", type=float, default=1.0)
    ap.add_argument("--freeze-s", type=float, default=None)
    a = ap.parse_args()
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=a.bag, storage_id="mcap"),
           rosbag2_py.ConverterOptions("cdr", "cdr"))
    stats = collections.defaultdict(lambda: {"n": 0, "smin": 1e18, "smax": -1e18,
                                             "omin": 1e18, "omax": -1e18})
    outliers = []
    around = []
    while r.has_next():
        topic, data, t_ns = r.read_next()
        if topic != "/tf":
            continue
        recv = t_ns * 1e-9
        for tf in deserialize_message(data, TFMessage).transforms:
            p, c = tf.header.frame_id, tf.child_frame_id
            if p not in FRAMES and c not in FRAMES:
                continue
            s = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
            off = s - recv
            k = f"{p}->{c}"
            st = stats[k]
            st["n"] += 1
            st["smin"], st["smax"] = min(st["smin"], s), max(st["smax"], s)
            st["omin"], st["omax"] = min(st["omin"], off), max(st["omax"], off)
            if abs(off) > a.limit_s:
                outliers.append({"tf": k, "stamp": round(s, 3), "recv": round(recv, 3), "offset": round(off, 3)})
            if a.freeze_s is not None and abs(s - a.freeze_s) < 0.0005 and k == "map->odom":
                around.append({"stamp": round(s, 4), "recv": round(recv, 3)})
    print(json.dumps({"per_transform": {k: {kk: round(v, 3) if isinstance(v, float) else v
                                            for kk, v in st.items()} for k, st in stats.items()},
                      "outliers_n": len(outliers), "outliers_first": outliers[:40],
                      "map_odom_at_freeze_stamp": around[:10]}, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
