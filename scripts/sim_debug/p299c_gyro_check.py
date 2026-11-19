#!/usr/bin/env python3
"""P2-99c: is the IMU gyro actually used for yaw? (sim, robot idle -> spin).

Commands an in-place turn on /cmd_vel_teleop (twist_mux) for --spin-s and compares, per sample,
the gyro z (/imu/data), the wheel-odometry yaw rate (/diff_drive_controller/odom)
and the EKF yaw rate (/odometry/filtered). The EKF fuses the gyro only (wheel
yaw is excluded in ekf.yaml), so its yaw rate must track the gyro.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--spin-s", type=float, default=6.0)
    ap.add_argument("--wz", type=float, default=0.6)
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    n = Node("p299c_gyro_check")
    n.set_parameters([rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)])
    last: dict = {"imu": None, "wheel": None, "ekf": None}
    count = {"imu": 0, "wheel": 0, "ekf": 0}
    frames: dict = {}

    def cb(key):
        def f(m):
            count[key] += 1
            frames[key] = m.header.frame_id
            last[key] = m.angular_velocity.z if key == "imu" else m.twist.twist.angular.z
        return f

    n.create_subscription(Imu, "/imu/data", cb("imu"), qos_profile_sensor_data)
    n.create_subscription(Odometry, "/diff_drive_controller/odom", cb("wheel"), 20)
    n.create_subscription(Odometry, "/odometry/filtered", cb("ekf"), 20)
    pub = n.create_publisher(Twist, "/cmd_vel_teleop", 10)
    samples = []
    t_end = time.time() + 3.0
    while time.time() < t_end:
        rclpy.spin_once(n, timeout_sec=0.05)
    idle = dict(count)
    t0 = time.time()
    while time.time() - t0 < args.spin_s:
        cmd = Twist()
        cmd.angular.z = args.wz
        pub.publish(cmd)
        rclpy.spin_once(n, timeout_sec=0.05)
        if time.time() - t0 > 2.0 and None not in last.values():
            samples.append(dict(last))
    for _ in range(10):
        pub.publish(Twist())
        rclpy.spin_once(n, timeout_sec=0.05)
    mean = {k: (sum(s[k] for s in samples) / len(samples) if samples else None) for k in last}
    track = (abs(mean["ekf"] - mean["imu"]) if samples else None)
    res = {"messages_in_3s_idle": idle, "frames": frames, "spin_cmd_wz": args.wz,
           "mean_wz_after_2s": mean, "ekf_minus_gyro": track, "samples": len(samples),
           "gyro_used": bool(samples) and track is not None and track < 0.05 and abs(mean["imu"]) > 0.1}
    (args.out / "gyro_check.json").write_text(json.dumps(res, indent=2))
    print(json.dumps(res, indent=2))
    n.destroy_node()
    rclpy.shutdown()
    return 0 if res["gyro_used"] else 1


if __name__ == "__main__":
    sys.exit(main())
