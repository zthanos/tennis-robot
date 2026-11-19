#!/usr/bin/env python3
"""P2-99c Nav2 stall diagnosis: per-second TF stamp AGE, clock health, /scan flow.

Run inside the sim container. Every --period s writes one CSV row with:
  wall_s, clock_s, rtf (d clock / d wall), clock_jumps (backwards or > 0.5 s),
  map_odom age (clock - newest map->odom stamp), map_odom msgs,
  odom_base age (clock - newest odom->base_footprint stamp), odom_base msgs,
  scan msgs, scan age (clock - newest /scan stamp).
Ages are taken at the probe; a growing map->odom age with a healthy clock is
"fresh rate, old stamp" (the user's hypothesis).
Optionally sends NavigateToPose goals at fixed probe-relative times.
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
import time
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener


def stamp(s) -> float:
    return s.sec + s.nanosec * 1e-9


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--duration-s", type=float, default=180.0)
    ap.add_argument("--period", type=float, default=1.0)
    ap.add_argument("--goal", action="append", default=[],
                    help="t_s,x,y,yaw : send a goal t_s seconds after start (wall)")
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    n = rclpy.create_node("p299c_tf_age_probe")
    n.set_parameters([rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)])
    st = {"clock": None, "map_odom": None, "odom_base": None, "scan": None,
          "n_mo": 0, "n_ob": 0, "n_scan": 0, "jumps": 0, "last_clock": None}

    def on_clock(m):
        c = stamp(m.clock)
        if st["last_clock"] is not None and (c < st["last_clock"] or c - st["last_clock"] > 0.5):
            st["jumps"] += 1
        st["last_clock"] = st["clock"] = c

    def on_tf(m):
        for t in m.transforms:
            s = stamp(t.header.stamp)
            if t.header.frame_id == "map" and t.child_frame_id == "odom":
                st["n_mo"] += 1
                st["map_odom"] = s if st["map_odom"] is None else max(st["map_odom"], s)
            elif t.header.frame_id == "odom" and t.child_frame_id in ("base_footprint", "base_link"):
                st["n_ob"] += 1
                st["odom_base"] = s if st["odom_base"] is None else max(st["odom_base"], s)

    def on_scan(m):
        st["n_scan"] += 1
        st["scan"] = stamp(m.header.stamp)

    n.create_subscription(Clock, "/clock", on_clock, 10)
    n.create_subscription(TFMessage, "/tf", on_tf, 200)
    n.create_subscription(LaserScan, "/scan", on_scan, qos_profile_sensor_data)
    nav = ActionClient(n, NavigateToPose, "navigate_to_pose")
    # A normal tf2 buffer, like the one inside every Nav2 server: if raw /tf is
    # fresh but this buffer's latest map->odom is old, tf2 itself drops data.
    buf = Buffer()
    TransformListener(buf, n)

    def buffer_age(target, source, clock):
        try:
            t = buf.lookup_transform(target, source, rclpy.time.Time())
        except Exception:  # noqa: BLE001
            return None
        return None if clock is None else round(clock - stamp(t.header.stamp), 3)

    goals = []
    for g in args.goal:
        t, x, y, yaw = (float(v) for v in g.split(","))
        goals.append({"t": t, "x": x, "y": y, "yaw": yaw, "sent": False, "handle": None,
                      "result": None, "status": None, "sent_clock": None})

    f = open(args.out / "tf_age.csv", "w", newline="")
    w = csv.writer(f)
    w.writerow(["wall_s", "clock_s", "rtf", "clock_jumps", "map_odom_age_s", "map_odom_msgs",
                "odom_base_age_s", "odom_base_msgs", "scan_msgs", "scan_age_s",
                "buf_map_odom_age_s", "buf_map_base_age_s", "goal_events"])
    t0 = time.time()
    next_t = t0 + args.period
    prev_wall, prev_clock = t0, None
    events: list[str] = []          # accumulated until the next CSV row
    while time.time() - t0 < args.duration_s:
        rclpy.spin_once(n, timeout_sec=0.02)
        now = time.time()
        for g in goals:
            if not g["sent"] and now - t0 >= g["t"] and nav.server_is_ready():
                msg = NavigateToPose.Goal()
                msg.pose = PoseStamped()
                msg.pose.header.frame_id = "map"
                msg.pose.header.stamp = n.get_clock().now().to_msg()
                msg.pose.pose.position.x, msg.pose.pose.position.y = g["x"], g["y"]
                msg.pose.pose.orientation.z = math.sin(g["yaw"] / 2)
                msg.pose.pose.orientation.w = math.cos(g["yaw"] / 2)
                g["future"] = nav.send_goal_async(msg)
                g["sent"], g["sent_clock"] = True, st["clock"]
                events.append(f"sent({g['x']},{g['y']})")
            if g["sent"] and g["handle"] is None and g.get("future") is not None and g["future"].done():
                g["handle"] = g["future"].result()
                if g["handle"] is not None and g["handle"].accepted:
                    g["result"] = g["handle"].get_result_async()
                else:
                    g["status"] = "rejected"
                    events.append("rejected")
            if g["result"] is not None and g["status"] is None and g["result"].done():
                g["status"] = g["result"].result().status
                events.append(f"done({g['x']},{g['y']})={g['status']}")
        if now >= next_t:
            next_t += args.period
            c = st["clock"]
            rtf = None
            if c is not None and prev_clock is not None and now > prev_wall:
                rtf = (c - prev_clock) / (now - prev_wall)
            prev_wall, prev_clock = now, c
            age = lambda k: None if (c is None or st[k] is None) else round(c - st[k], 3)
            w.writerow([round(now - t0, 2), c, None if rtf is None else round(rtf, 3), st["jumps"],
                        age("map_odom"), st["n_mo"], age("odom_base"), st["n_ob"],
                        st["n_scan"], age("scan"), buffer_age("map", "odom", c),
                        buffer_age("map", "base_link", c), ";".join(events)])
            f.flush()
            events = []
            for k in ("n_mo", "n_ob", "n_scan"):
                st[k] = 0
    f.close()
    summary = [{"goal": [g["x"], g["y"]], "t": g["t"], "status": g["status"],
                "succeeded": g["status"] == GoalStatus.STATUS_SUCCEEDED} for g in goals]
    (args.out / "goals.txt").write_text(repr(summary))
    print(summary)
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
