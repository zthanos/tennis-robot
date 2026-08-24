"""One bounded, closed-loop move of the basket lift carriage.

Why this is a separate process rather than more logic in RosService: the console
talks to ROS by shelling out, and its feedback samples cost a `ros2 topic echo`
subprocess each (~1.5 s). At the carriage's 0.12 m/s rated speed that is 180 mm
of travel per sample, so a supervisor built on those samples cannot stop within
the 5 mm endpoint tolerance — it overshoots and parks the joint on a hard stop.
Parking on a hard stop is exactly what freezes the joint (a permanently active
DART limit constraint that gz-sim cannot drive out of), so the slow loop does
not merely overshoot: it disables the actuator.

Running the whole move inside one short-lived rclpy process keeps the console's
subprocess boundary intact while giving the loop real feedback at joint-state
rate. RosService invokes it as:

    python3 -m tennis_robot.basket_lift_mover --target 0.100

Exit code 0 means the endpoint was reached, tracked and settled. There is no
retry here and none should be added above it: a failure means the actuator did
not do what it was told, and that must stay visible.
"""

from __future__ import annotations

import argparse
import sys
import time

import rclpy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

COMMAND_TOPIC = "/basket_velocity_controller/commands"
JOINT_NAME = "basket_joint"
# Keep in step with RosService.BASKET_* and the basket_joint limits in
# urdf/components/basket.urdf.xacro.
SPEED_MPS = 0.12
POSITION_TOLERANCE_M = 0.005
SETTLED_VELOCITY_MPS = 0.03
MIN_TRACKING_VELOCITY_MPS = 0.02
PUBLISH_HZ = 50.0
SETTLE_TIMEOUT_S = 3.0
# Slack over the distance-derived drive time (accel, publish jitter).
DRIVE_MARGIN_S = 1.5
# Longest gap in /joint_states tolerated while the carriage is moving.
FEEDBACK_STALE_S = 0.5


class BasketLiftDriver:
    """Sustained velocity command plus measured-state feedback for one axis."""

    def __init__(self, node) -> None:
        self._node = node
        self.position: float | None = None
        self.velocity: float | None = None
        node.create_subscription(JointState, "/joint_states", self._on_joint_states, 10)
        self._publisher = node.create_publisher(Float64MultiArray, COMMAND_TOPIC, 10)

    def _on_joint_states(self, message: JointState) -> None:
        if JOINT_NAME in message.name:
            index = message.name.index(JOINT_NAME)
            self.position = float(message.position[index])
            self.velocity = float(message.velocity[index])

    def command(self, value: float) -> None:
        self._publisher.publish(Float64MultiArray(data=[value]))

    def wait_for_feedback(self, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self._node, timeout_sec=0.05)
            if self.position is not None:
                return True
        return False

    def move_to(self, target_m: float, timeout_s: float) -> tuple[bool, str]:
        """Drive to target_m and stop. Returns (ok, human-readable detail).

        Two hard bounds keep the carriage off its mechanical stops, because
        REACHING a stop under command latches the DART joint exactly like the
        original parked-on-the-limit defect — the over-travel margin only stops
        the carriage being born on the limit, it does not survive an overshoot:

          * the drive is bounded by the distance actually requested, so a quiet
            feedback stream cannot turn into open-ended driving;
          * a stale feedback stream aborts the move instead of driving blind.

        Neither is a retry. Both fail loudly.
        """
        start = self.position if self.position is not None else 0.0
        direction = 1.0 if target_m > start else -1.0
        speed = direction * SPEED_MPS
        # Distance-derived bound: the travel plus one tolerance of slop, never
        # the caller's generous wall-clock timeout.
        travel_s = (abs(target_m - start) + POSITION_TOLERANCE_M) / SPEED_MPS
        deadline = time.monotonic() + min(timeout_s, travel_s + DRIVE_MARGIN_S)
        period = 1.0 / PUBLISH_HZ
        next_publish = 0.0
        peak_tracking = 0.0
        reached = abs(start - target_m) <= POSITION_TOLERANCE_M
        already_there = reached
        stale = False
        last_feedback_at = time.monotonic()
        last_position = self.position
        try:
            while not reached and time.monotonic() < deadline:
                now = time.monotonic()
                if now >= next_publish:
                    self.command(speed)
                    next_publish = now + period
                rclpy.spin_once(self._node, timeout_sec=0.02)
                if self.position != last_position:
                    last_position = self.position
                    last_feedback_at = now
                elif now - last_feedback_at > FEEDBACK_STALE_S:
                    stale = True
                    break
                peak_tracking = max(peak_tracking, (self.velocity or 0.0) * direction)
                measured = self.position if self.position is not None else start
                if abs(measured - target_m) <= POSITION_TOLERANCE_M:
                    reached = True
        finally:
            # No exit path may leave the carriage driving.
            self.command(0.0)

        settle_deadline = time.monotonic() + SETTLE_TIMEOUT_S
        while time.monotonic() < settle_deadline:
            rclpy.spin_once(self._node, timeout_sec=0.02)
            if abs(self.velocity or 0.0) <= SETTLED_VELOCITY_MPS:
                break

        # `or` would turn the lowered endpoint (exactly 0.0) back into start.
        final = self.position if self.position is not None else start
        if stale:
            return False, (
                f"joint feedback went stale for more than {FEEDBACK_STALE_S:g}s at "
                f"{final * 1000:.2f} mm; stopped rather than drive the carriage blind "
                "into a hard stop"
            )
        if already_there:
            return True, f"already at {final * 1000:.2f} mm"
        if peak_tracking < MIN_TRACKING_VELOCITY_MPS:
            return False, (
                f"actuator never tracked the command (peak {peak_tracking * 1000:.3f} "
                f"mm/s in the commanded direction); the joint is frozen, not slow"
            )
        if not reached:
            return False, (
                f"endpoint {target_m * 1000:.1f} mm not reached within {timeout_s:g}s "
                f"(stopped at {final * 1000:.2f} mm)"
            )
        if abs(final - target_m) > POSITION_TOLERANCE_M:
            return False, f"settled at {final * 1000:.2f} mm, outside tolerance"
        if abs(self.velocity or 0.0) > SETTLED_VELOCITY_MPS:
            return False, f"did not settle (velocity {(self.velocity or 0) * 1000:.2f} mm/s)"
        return True, f"{final * 1000:.2f} mm, peak tracking {peak_tracking * 1000:.2f} mm/s"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Move the basket lift to a target.")
    parser.add_argument("--target", type=float, required=True,
                        help="Target position in metres along the lift axis.")
    parser.add_argument("--timeout-s", type=float, default=12.0)
    args = parser.parse_args(argv)

    rclpy.init()
    node = rclpy.create_node("basket_lift_mover")
    driver = BasketLiftDriver(node)
    try:
        if not driver.wait_for_feedback(10.0):
            print("no /joint_states feedback for basket_joint", file=sys.stderr)
            return 1
        ok, detail = driver.move_to(args.target, args.timeout_s)
    finally:
        driver.command(0.0)
        rclpy.spin_once(node, timeout_sec=0.2)
        node.destroy_node()
        rclpy.shutdown()
    print(detail if ok else f"basket move failed: {detail}",
          file=sys.stdout if ok else sys.stderr)
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
