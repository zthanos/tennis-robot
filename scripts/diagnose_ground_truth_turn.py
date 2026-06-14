#!/usr/bin/env python3
"""Compare wheel odom yaw with Gazebo ground-truth model yaw."""

from __future__ import annotations

import math
import re
import subprocess
import time
import argparse

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


GZ_POSE_TOPIC = "/world/tennis_court/pose/info"
CMD_TOPIC = "/cmd_vel_teleop"
ODOM_TOPIC = "/diff_drive_controller/odom"


def _yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _angle_delta(a: float, b: float) -> float:
    return (a - b + math.pi) % (2.0 * math.pi) - math.pi


def _gazebo_model_pose(model_name: str = "tennis_robot") -> tuple[list[float], list[float]]:
    out = subprocess.check_output(
        ["gz", "topic", "-e", "-t", GZ_POSE_TOPIC, "-n", "1"],
        text=True,
        timeout=8,
    )
    current: list[str] = []
    depth = 0
    in_pose = False
    for line in out.splitlines():
        stripped = line.strip()
        if stripped == "pose {":
            in_pose = True
            current = [line]
            depth = 1
            continue
        if not in_pose:
            continue
        current.append(line)
        depth += line.count("{") - line.count("}")
        if depth != 0:
            continue
        block = "\n".join(current)
        if f'name: "{model_name}"' in block:
            vals = [
                float(v)
                for v in re.findall(
                    r"\n\s*(?:x|y|z|w):\s*(-?\d+(?:\.\d+)?(?:e[+-]?\d+)?)",
                    block,
                )
            ]
            return vals[:3], vals[3:7]
        in_pose = False
        current = []
    raise RuntimeError(f"Gazebo pose for {model_name!r} not found")


class GroundTruthTurnDiagnostic:
    def __init__(self) -> None:
        self.node = rclpy.create_node("ground_truth_turn_diagnostic")
        self.pub = self.node.create_publisher(Twist, CMD_TOPIC, 10)
        self.odom: Odometry | None = None
        self.node.create_subscription(Odometry, ODOM_TOPIC, self._odom_cb, 10)

    def _odom_cb(self, msg: Odometry) -> None:
        self.odom = msg

    def _wait_for_odom(self) -> None:
        deadline = time.time() + 8.0
        while self.odom is None and time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        if self.odom is None:
            raise RuntimeError(f"no odometry on {ODOM_TOPIC}")

    def _odom_yaw(self) -> float:
        if self.odom is None:
            raise RuntimeError("no odometry")
        q = self.odom.pose.pose.orientation
        return _yaw_from_quat(q.x, q.y, q.z, q.w)

    def _odom_stamp(self) -> float:
        if self.odom is None:
            raise RuntimeError("no odometry")
        stamp = self.odom.header.stamp
        return stamp.sec + stamp.nanosec * 1e-9

    def _stop(self) -> None:
        msg = Twist()
        for _ in range(10):
            self.pub.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def run_turn(self, label: str, angular_z: float, sim_seconds: float = 2.0) -> None:
        self._wait_for_odom()
        gz_pos0, gz_quat0 = _gazebo_model_pose()
        gz_yaw0 = _yaw_from_quat(*gz_quat0)
        odom_yaw0 = self._odom_yaw()
        odom_stamp0 = self._odom_stamp()

        cmd = Twist()
        cmd.angular.z = angular_z
        started = time.time()
        while time.time() - started < 45.0:
            self.pub.publish(cmd)
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if self._odom_stamp() - odom_stamp0 >= sim_seconds:
                break
        self._stop()

        gz_pos1, gz_quat1 = _gazebo_model_pose()
        gz_yaw1 = _yaw_from_quat(*gz_quat1)
        odom_yaw1 = self._odom_yaw()
        print(
            f"{label}: "
            f"odom_delta_deg={math.degrees(_angle_delta(odom_yaw1, odom_yaw0)):+.1f} "
            f"gz_delta_deg={math.degrees(_angle_delta(gz_yaw1, gz_yaw0)):+.1f} "
            f"gz_xy=({gz_pos0[0]:+.2f},{gz_pos0[1]:+.2f})->"
            f"({gz_pos1[0]:+.2f},{gz_pos1[1]:+.2f})"
        )

    def destroy(self) -> None:
        self.node.destroy_node()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--sequence",
        default="left,right",
        help="Comma-separated turn sequence using left/right labels.",
    )
    args = parser.parse_args()

    rclpy.init()
    diag = GroundTruthTurnDiagnostic()
    try:
        for index, item in enumerate(args.sequence.split(",")):
            direction = item.strip().lower()
            if not direction:
                continue
            if index > 0:
                time.sleep(1.0)
            if direction == "left":
                diag.run_turn("left", 0.8)
            elif direction == "right":
                diag.run_turn("right", -0.8)
            else:
                raise ValueError(f"unknown sequence item: {item!r}")
    finally:
        diag._stop()
        diag.destroy()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
