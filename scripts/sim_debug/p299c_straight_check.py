#!/usr/bin/env python3
"""P2-99c: straight-line drive, EKF odometry vs Gazebo ground truth.

Drives --v m/s for --t s on /cmd_vel_teleop and compares the EKF
(/odometry/filtered) displacement with the model pose reported by
`gz model -m tennis_robot -p` before and after (run inside the sim container).
"""
import argparse, json, math, re, subprocess, sys, time
from pathlib import Path
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def gz_pose():
    out = subprocess.run(["gz", "model", "-m", "tennis_robot", "-p"], capture_output=True, text=True, timeout=20).stdout
    nums = [float(v) for v in re.findall(r"[-+]?\d*\.\d+(?:[eE][-+]?\d+)?|[-+]?\d+(?:[eE][-+]?\d+)?", out.split("Pose")[-1])]
    return nums[:3], nums[3:6], out


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--v", type=float, default=0.3); ap.add_argument("--t", type=float, default=5.0)
    a = ap.parse_args(); a.out.mkdir(parents=True, exist_ok=True)
    rclpy.init(); n = rclpy.create_node("p299c_straight_check")
    n.set_parameters([rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)])
    odom = {}
    n.create_subscription(Odometry, "/odometry/filtered", lambda m: odom.update(x=m.pose.pose.position.x, y=m.pose.pose.position.y), 20)
    pub = n.create_publisher(Twist, "/cmd_vel_teleop", 10)
    end = time.time() + 3
    while time.time() < end: rclpy.spin_once(n, timeout_sec=0.05)
    p0, r0, raw0 = gz_pose(); o0 = dict(odom)
    t0 = time.time()
    while time.time() - t0 < a.t:
        m = Twist(); m.linear.x = a.v; pub.publish(m); rclpy.spin_once(n, timeout_sec=0.05)
    for _ in range(20): pub.publish(Twist()); rclpy.spin_once(n, timeout_sec=0.05)
    end = time.time() + 2
    while time.time() < end: rclpy.spin_once(n, timeout_sec=0.05)
    p1, r1, raw1 = gz_pose(); o1 = dict(odom)
    gt = math.hypot(p1[0] - p0[0], p1[1] - p0[1]); ek = math.hypot(o1["x"] - o0["x"], o1["y"] - o0["y"])
    res = {"cmd_v": a.v, "cmd_t": a.t, "ground_truth_m": round(gt, 4), "ekf_m": round(ek, 4),
           "ekf_error_pct": round(100 * (ek - gt) / gt, 2) if gt > 0 else None,
           "gt_yaw_change_rad": round(r1[2] - r0[2], 4), "gt_lateral_m": None,
           "raw_gz_before": raw0[-300:], "raw_gz_after": raw1[-300:]}
    (a.out / "straight.json").write_text(json.dumps(res, indent=2)); print(json.dumps({k: v for k, v in res.items() if not k.startswith("raw")}, indent=2))


if __name__ == "__main__":
    sys.exit(main())
