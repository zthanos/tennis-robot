#!/usr/bin/env python3
"""Compare wheel odom yaw with Gazebo ground-truth model yaw."""

from __future__ import annotations

import math
import re
import subprocess
import time
import argparse

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry


CMD_TOPIC = "/cmd_vel_teleop"
ODOM_TOPIC = "/diff_drive_controller/odom"


def _yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _angle_delta(a: float, b: float) -> float:
    return (a - b + math.pi) % (2.0 * math.pi) - math.pi


def _gazebo_model_pose(model_name: str = "tennis_robot") -> tuple[list[float], list[float]]:
    out = subprocess.check_output(
        ["gz", "model", "-m", model_name, "-p"],
        text=True,
        timeout=8,
    )
    triples = re.findall(
        r"\[\s*(-?\d+(?:\.\d+)?(?:e[+-]?\d+)?)\s+"
        r"(-?\d+(?:\.\d+)?(?:e[+-]?\d+)?)\s+"
        r"(-?\d+(?:\.\d+)?(?:e[+-]?\d+)?)\s*\]",
        out,
        flags=re.IGNORECASE,
    )
    if len(triples) >= 2:
        position = [float(value) for value in triples[-2]]
        roll, pitch, yaw = (float(value) for value in triples[-1])
        cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
        cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
        cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
        quaternion = [
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        ]
        return position, quaternion
    raise RuntimeError(f"Gazebo pose for {model_name!r} not found")


class GroundTruthTurnDiagnostic:
    def __init__(self, cmd_topic: str, stamped_cmd: bool) -> None:
        self.node = rclpy.create_node("ground_truth_turn_diagnostic")
        self.stamped_cmd = stamped_cmd
        command_type = TwistStamped if stamped_cmd else Twist
        self.pub = self.node.create_publisher(command_type, cmd_topic, 10)
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
            self.pub.publish(self._command(msg))
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _command(self, twist: Twist):
        if not self.stamped_cmd:
            return twist
        msg = TwistStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.twist = twist
        return msg

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
            self.pub.publish(self._command(cmd))
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
    parser.add_argument("--cmd-topic", default=CMD_TOPIC)
    parser.add_argument("--stamped-cmd", action="store_true")
    args = parser.parse_args()

    rclpy.init()
    diag = GroundTruthTurnDiagnostic(args.cmd_topic, args.stamped_cmd)
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
