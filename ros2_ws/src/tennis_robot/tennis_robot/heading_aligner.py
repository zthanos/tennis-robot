"""Rotate in place to an absolute map-frame heading, closed-loop and bounded.

Throwing Mode needs the robot to face the net, and nothing in the navigation
path delivers a final heading:

  * the shared ``general_goal_checker`` sets ``yaw_goal_tolerance: 3.14`` on
    purpose, so the collection lanes do not spin at every lane end;
  * Regulated Pure Pursuit performs no final rotation to the goal orientation;
  * Nav2's ``Spin`` behaviour aborts with COLLISION_AHEAD (error 703) even with
    a local costmap measured completely clear around the robot, and this Jazzy
    build's Spin goal has no ``disable_collision_checks`` field to opt out of
    that check.

So the subsystem that needs the heading closes it itself, through the highest
priority twist_mux input, in the same shape as basket_lift_mover: a short-lived
rclpy process, measured feedback, hard bounds, and a guaranteed stop. Nothing
here disables a safety check — the Throwing Mode readiness gate still decides
whether the resulting pose is good enough to arm.

Usage:
    python3 -m tennis_robot.heading_aligner --target-yaw 0.0027
"""

from __future__ import annotations

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from tf2_ros import Buffer, TransformListener

CMD_TOPIC = "/cmd_vel_teleop"          # twist_mux priority 100, timeout 0.5 s
MAP_FRAME = "map"
BASE_FRAME = "base_link"
# Keep at or below behavior_server.max_rotational_vel so the rotation stays in
# the regime the drivetrain and SLAM were tuned for.
MAX_ANGULAR_VEL_RAD_S = 0.25
MIN_ANGULAR_VEL_RAD_S = 0.06
GAIN = 1.2
PUBLISH_HZ = 20.0
SETTLE_S = 0.8
FEEDBACK_STALE_S = 1.0


def _yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class HeadingAligner:
    def __init__(self, node) -> None:
        self._node = node
        self._buffer = Buffer()
        self._listener = TransformListener(self._buffer, node)
        self._publisher = node.create_publisher(Twist, CMD_TOPIC, 10)

    def measured_yaw(self) -> float | None:
        try:
            tf = self._buffer.lookup_transform(
                MAP_FRAME, BASE_FRAME, rclpy.time.Time(),
                timeout=Duration(seconds=0.1))
        except Exception:
            return None
        return _yaw_from_quaternion(tf.transform.rotation)

    def command(self, angular_z: float) -> None:
        msg = Twist()
        msg.angular.z = angular_z
        self._publisher.publish(msg)

    def align(self, target_yaw: float, tolerance: float,
              timeout_s: float) -> tuple[bool, str]:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline and self.measured_yaw() is None:
            rclpy.spin_once(self._node, timeout_sec=0.05)
        start = self.measured_yaw()
        if start is None:
            return False, f"no {MAP_FRAME}->{BASE_FRAME} transform within {timeout_s:g}s"

        error = _wrap(target_yaw - start)
        if abs(error) <= tolerance:
            return True, f"already at {math.degrees(start):.2f} deg"
        # Bound the rotation by the angle actually requested, so a stalled or
        # silent feedback stream can never turn into an open-ended spin.
        bound = abs(error) / MIN_ANGULAR_VEL_RAD_S + 5.0
        deadline = time.monotonic() + min(timeout_s, bound)
        period = 1.0 / PUBLISH_HZ
        next_publish = 0.0
        last_yaw, last_change = start, time.monotonic()
        measured = start
        try:
            while time.monotonic() < deadline:
                now = time.monotonic()
                sample = self.measured_yaw()
                if sample is not None:
                    if abs(_wrap(sample - last_yaw)) > 1e-4:
                        last_yaw, last_change = sample, now
                    measured = sample
                elif now - last_change > FEEDBACK_STALE_S:
                    return False, "pose feedback went stale; stopped rotating"
                error = _wrap(target_yaw - measured)
                if abs(error) <= tolerance:
                    break
                if now >= next_publish:
                    speed = max(MIN_ANGULAR_VEL_RAD_S,
                                min(MAX_ANGULAR_VEL_RAD_S, abs(error) * GAIN))
                    self.command(math.copysign(speed, error))
                    next_publish = now + period
                rclpy.spin_once(self._node, timeout_sec=0.02)
        finally:
            # Never leave the drivetrain turning, on any exit path.
            settle = time.monotonic() + SETTLE_S
            while time.monotonic() < settle:
                self.command(0.0)
                rclpy.spin_once(self._node, timeout_sec=0.02)

        final = self.measured_yaw()
        final = measured if final is None else final
        residual = _wrap(target_yaw - final)
        if abs(residual) > tolerance:
            return False, (
                f"heading {math.degrees(final):.2f} deg leaves "
                f"{math.degrees(residual):.2f} deg of error, tolerance is "
                f"{math.degrees(tolerance):.2f} deg"
            )
        return True, (f"heading {math.degrees(final):.2f} deg, residual "
                      f"{math.degrees(residual):.2f} deg")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--target-yaw", type=float, required=True)
    parser.add_argument("--tolerance-rad", type=float, default=math.radians(6.0))
    parser.add_argument("--timeout-s", type=float, default=45.0)
    args = parser.parse_args(argv)

    rclpy.init()
    node = rclpy.create_node("heading_aligner")
    aligner = HeadingAligner(node)
    try:
        ok, detail = aligner.align(args.target_yaw, args.tolerance_rad, args.timeout_s)
    finally:
        aligner.command(0.0)
        rclpy.spin_once(node, timeout_sec=0.2)
        node.destroy_node()
        rclpy.shutdown()
    print(detail if ok else f"heading alignment failed: {detail}",
          file=sys.stdout if ok else sys.stderr)
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
