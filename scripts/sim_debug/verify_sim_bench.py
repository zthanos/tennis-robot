#!/usr/bin/env python3
"""Fail-loud readiness gate for a running simulation bench.

A bench is NOT ready because its processes started. `ros2 control` can report
every controller "active" while the ROS simulation clock is missing and every
`/joint_states` stamp is frozen at 0 — controller_manager falls back to the
time argument gz_ros2_control passes it, logs "No clock received, using time
argument instead!", and keeps running. Physics advances, so ground-truth pose
logging still looks healthy, while every timestamped ROS consumer (TF from
robot_state_publisher, the EKF, SLAM, replay fixtures) silently sees t=0.

This script asserts the three independent properties a bench must have before
any sweep or Throwing Mode validation is allowed to trust it:

  1. /clock advances                    — the ROS simulation clock exists.
  2. controller_manager is on that clock — /joint_states stamps track /clock
                                           (joint_state_broadcaster stamps with
                                           the controller_manager node clock).
  3. a joint actually actuates          — a commanded joint's MEASURED state
                                           moves within a bounded timeout.

Every check is bounded; the script always leaves the test joint commanded to
zero. Exit code 0 means all checks passed.

Usage:
    python3 scripts/sim_debug/verify_sim_bench.py [--timeout-s 20]
"""

from __future__ import annotations

import argparse
import sys
import time

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

# The intake wheels are present in every packaging variant, spin freely, and
# integrate position monotonically, so any real actuation is unambiguous and
# nothing moves the robot or disturbs a staged bench scene.
DEFAULT_CONTROLLER_TOPIC = "/intake_wheel_velocity_controller/commands"
DEFAULT_TEST_JOINT = "intake_wheel_left_joint"
DEFAULT_COMMAND = [-8.0, 8.0]

CLOCK_MIN_SIM_TIME_S = 1.0
STAMP_SKEW_TOLERANCE_S = 0.5
# Default suits a freely spinning wheel; a bounded prismatic axis needs a
# smaller threshold, so it is exposed as --min-motion.
MIN_JOINT_MOTION_RAD = 0.5
JOINT_SETTLED_TOLERANCE = 0.05
COMMAND_PUBLISH_HZ = 20.0


def _stamp_s(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class BenchProbe:
    def __init__(self, node, controller_topic: str, test_joint: str) -> None:
        self._node = node
        self._test_joint = test_joint
        # /clock is published RELIABLE depth 1 by sim_clock_relay_node; a
        # BEST_EFFORT reader matches that and cannot apply backpressure.
        clock_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.clock_samples_ns: list[int] = []
        self.latest_clock_s: float | None = None
        self.joint_stamps_s: list[float] = []
        self.joint_message_count = 0
        self.joint_position: float | None = None
        self.joint_velocity: float | None = None
        self.joint_names: list[str] = []
        node.create_subscription(Clock, "/clock", self._on_clock, clock_qos)
        node.create_subscription(JointState, "/joint_states", self._on_joint_states, 10)
        self._command_publisher = node.create_publisher(
            Float64MultiArray, controller_topic, 10
        )

    def _on_clock(self, message: Clock) -> None:
        stamp_ns = int(message.clock.sec) * 1_000_000_000 + int(message.clock.nanosec)
        if not self.clock_samples_ns or stamp_ns != self.clock_samples_ns[-1]:
            self.clock_samples_ns.append(stamp_ns)
        self.latest_clock_s = stamp_ns * 1e-9

    def _on_joint_states(self, message: JointState) -> None:
        self.joint_message_count += 1
        self.joint_names = list(message.name)
        stamp = _stamp_s(message.header.stamp)
        if not self.joint_stamps_s or stamp != self.joint_stamps_s[-1]:
            self.joint_stamps_s.append(stamp)
        if self._test_joint in message.name:
            index = message.name.index(self._test_joint)
            if index < len(message.position):
                self.joint_position = float(message.position[index])
            if index < len(message.velocity):
                self.joint_velocity = float(message.velocity[index])

    def publish_command(self, values: list[float]) -> None:
        self._command_publisher.publish(Float64MultiArray(data=values))

    def spin_until(self, predicate, deadline: float) -> bool:
        while time.monotonic() < deadline:
            rclpy.spin_once(self._node, timeout_sec=0.05)
            if predicate():
                return True
        return False


def check_clock_advancing(probe: BenchProbe, timeout_s: float) -> str | None:
    deadline = time.monotonic() + timeout_s
    if not probe.spin_until(lambda: len(probe.clock_samples_ns) >= 2, deadline):
        publishers = probe._node.count_publishers("/clock")
        return (
            f"/clock did not advance within {timeout_s:g}s "
            f"({len(probe.clock_samples_ns)} distinct stamps, "
            f"{publishers} publisher(s)). The only /clock publisher is "
            "sim_clock_relay_node (gz /clock -> bridge -> /clock_raw -> relay)."
        )
    first, last = probe.clock_samples_ns[0], probe.clock_samples_ns[-1]
    if last <= first:
        return f"/clock is not monotonic: {first} -> {last}"
    print(f"  clock_progress_ns={first}->{last}")
    return None


def check_controller_manager_clock(probe: BenchProbe, timeout_s: float) -> str | None:
    deadline = time.monotonic() + timeout_s
    if not probe.spin_until(lambda: len(probe.joint_stamps_s) >= 2, deadline):
        if probe.joint_message_count == 0:
            return (
                f"no /joint_states message arrived within {timeout_s:g}s; "
                "joint_state_broadcaster is not publishing at all."
            )
        # Messages ARE flowing, but every one carries the same stamp: the
        # controller_manager node clock is not advancing even though its
        # control loop is (gz_ros2_control passes it a time argument directly).
        frozen = probe.joint_stamps_s[-1] if probe.joint_stamps_s else float("nan")
        return (
            f"/joint_states is publishing ({probe.joint_message_count} messages in "
            f"{timeout_s:g}s) but every stamp is frozen at {frozen:g}s: "
            "controller_manager has use_sim_time=true and no clock source. Joint "
            "VALUES are still real; every ROS timestamp derived from them (/tf via "
            "robot_state_publisher, EKF, SLAM, recorded fixtures) is not."
        )
    # Compare only once simulated time is unambiguously past the tolerance, so
    # a frozen t=0 stamp cannot pass by being close to a just-started clock.
    if not probe.spin_until(
        lambda: (probe.latest_clock_s or 0.0) >= CLOCK_MIN_SIM_TIME_S, deadline
    ):
        return (
            f"simulated time stayed below {CLOCK_MIN_SIM_TIME_S:g}s within "
            f"{timeout_s:g}s; cannot compare controller_manager's clock."
        )
    joint_stamp = probe.joint_stamps_s[-1]
    clock_s = probe.latest_clock_s or 0.0
    skew = abs(clock_s - joint_stamp)
    if joint_stamp <= 0.0:
        return (
            f"/joint_states stamps are frozen at {joint_stamp:g} while /clock is at "
            f"{clock_s:.3f}s: controller_manager has use_sim_time=true but no clock "
            "source, so it timestamps with an unset simulated clock. Physics still "
            "advances, which is why process-startup checks miss this."
        )
    if skew > STAMP_SKEW_TOLERANCE_S:
        return (
            f"controller_manager clock skew {skew:.3f}s exceeds "
            f"{STAMP_SKEW_TOLERANCE_S:g}s (/joint_states stamp {joint_stamp:.3f}s vs "
            f"/clock {clock_s:.3f}s)."
        )
    print(f"  joint_states_stamp_s={joint_stamp:.3f} clock_s={clock_s:.3f} skew_s={skew:.3f}")
    return None


def check_joint_actuates(probe: BenchProbe, command: list[float],
                         timeout_s: float,
                         min_motion: float = MIN_JOINT_MOTION_RAD) -> str | None:
    deadline = time.monotonic() + timeout_s
    if not probe.spin_until(lambda: probe.joint_position is not None, deadline):
        return (
            f"test joint {probe._test_joint!r} never appeared in /joint_states "
            f"(saw {probe.joint_names})."
        )
    baseline = probe.joint_position
    assert baseline is not None
    # Hold the command as a stream: a single publish races DDS discovery on a
    # publisher that is only just matched, and a dropped command would be
    # indistinguishable from a dead joint.
    period = 1.0 / COMMAND_PUBLISH_HZ
    next_publish = 0.0

    def drive() -> bool:
        nonlocal next_publish
        now = time.monotonic()
        if now >= next_publish:
            probe.publish_command(command)
            next_publish = now + period
        return abs((probe.joint_position or baseline) - baseline) >= min_motion

    moved = probe.spin_until(drive, deadline)
    delta = abs((probe.joint_position or baseline) - baseline)
    probe.publish_command([0.0] * len(command))
    if not moved:
        return (
            f"test joint {probe._test_joint!r} moved {delta:.4f} under command "
            f"{command} — below the {min_motion:g} threshold within "
            f"{timeout_s:g}s. The controller is active but the joint is not "
            "actuating."
        )
    settle_deadline = time.monotonic() + timeout_s
    settled = probe.spin_until(
        lambda: abs(probe.joint_velocity or 0.0) <= JOINT_SETTLED_TOLERANCE,
        settle_deadline,
    )
    print(f"  test_joint={probe._test_joint} delta_rad={delta:.4f} "
          f"stopped={'yes' if settled else 'no'}")
    if not settled:
        return (
            f"test joint {probe._test_joint!r} did not stop after a zero command "
            f"(velocity {probe.joint_velocity})."
        )
    return None


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--timeout-s", type=float, default=20.0,
                        help="Per-check budget in wall-clock seconds.")
    parser.add_argument("--controller-topic", default=DEFAULT_CONTROLLER_TOPIC)
    parser.add_argument("--test-joint", default=DEFAULT_TEST_JOINT)
    parser.add_argument("--command", type=float, nargs="+", default=DEFAULT_COMMAND,
                        help="Command payload for --controller-topic.")
    parser.add_argument("--min-motion", type=float, default=MIN_JOINT_MOTION_RAD,
                        help="Minimum measured joint travel that counts as actuation "
                             "(rad for revolute, m for prismatic).")
    args = parser.parse_args()

    rclpy.init()
    # Deliberately NOT use_sim_time: this process measures the simulation clock
    # as data, so it must never block on the very thing it is validating.
    node = rclpy.create_node("sim_bench_validator")
    probe = BenchProbe(node, args.controller_topic, args.test_joint)
    checks = (
        ("clock advancing", lambda: check_clock_advancing(probe, args.timeout_s)),
        ("controller_manager on simulation clock",
         lambda: check_controller_manager_clock(probe, args.timeout_s)),
        ("commanded joint actuates",
         lambda: check_joint_actuates(probe, list(args.command), args.timeout_s,
                                      args.min_motion)),
    )
    # Every check runs even after one fails: the checks are independently
    # measurable, and "clock missing BUT the joint still actuates" is exactly
    # the diagnosis needed to judge whether existing bench results are salvage-
    # able. Stopping at the first failure would hide that.
    failures: list[str] = []
    try:
        for label, check in checks:
            print(f"[check] {label}")
            failure = check()
            if failure is None:
                print(f"[ok] {label}")
                continue
            failures.append(f"{label}: {failure}")
            print(f"[FAIL] {label}: {failure}", file=sys.stderr)
    finally:
        try:
            probe.publish_command([0.0] * len(args.command))
            rclpy.spin_once(node, timeout_sec=0.2)
        finally:
            node.destroy_node()
            rclpy.shutdown()
    if failures:
        print(f"BENCH NOT READY ({len(failures)}/{len(checks)} checks failed)",
              file=sys.stderr)
        return 1
    print("BENCH READY")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
