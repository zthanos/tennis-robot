#!/usr/bin/env python3
"""Characterize wheel-derived odometry against Gazebo truth.

Phase 18A2 diagnostic.  Each leg starts from a verified reset inside a bounded
clear workspace, so no trajectory can touch the net or a fence and no leg
inherits pose, wheel state or accumulated error from the one before it.  The
first attempt at this measurement drove the robot into the net after 29 s and
everything past that point was fiction (debug log #80).

Recorded per sample, time-aligned:

    commanded v / omega          what was asked for
    per-wheel joint velocities   what the wheels did
    /diff_drive_controller/odom  what the wheels were interpreted to mean
    /imu/data                    measured yaw rate
    /odometry/filtered           what the EKF concluded
    /sim/robot_true_pose         what actually happened

Longitudinal and lateral motion are compared in the robot's body frame, so a
heading error cannot masquerade as a translation error.  Commands are published
through the production teleop inlet; nothing here changes robot behaviour.

Requires a sim-only stack (TENNIS_LAUNCH_BRAIN=false): with no SLAM running,
resetting the model pose between legs disturbs no localization.

    python3 scripts/sim_debug/characterize_wheel_odometry.py --out runtime/char.jsonl
"""

from __future__ import annotations

import argparse
import json
import math
import subprocess

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import String

DRIVE_TOPIC = "/cmd_vel_teleop"      # production arbiter inlet, highest priority
WHEELS = ("front_left_wheel_joint", "rear_left_wheel_joint",
          "front_right_wheel_joint", "rear_right_wheel_joint")
WHEEL_RADIUS_M = 0.085
REPETITIONS = 3
WORLD, MODEL = "tennis_court", "tennis_robot"
# Clear box in the west half: the net is at x=0, fences at x=±16.5 and y=±8.5.
# Every leg starts here heading +y and stays well inside.
START = (-8.0, -3.0, math.pi / 2.0)
RESET_TOLERANCE_M = 0.05
SETTLE_WHEEL_RAD_S = 0.05        # all four wheels must be this quiet before a leg
SETTLE_HOLD_S = 0.6              # ... and stay quiet this long
# At 0.5 m/s a truth update (~0.06 s) advances ~0.03 m.  Anything past this is a
# teleport or a contact discontinuity, and the leg is rejected rather than
# measured: the invalid connector replay of debug log #81 entered its own reset.
DISCONTINUITY_M = 0.25


def yaw_of(orientation) -> float:
    return math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
    )


def set_model_pose(x: float, y: float, yaw: float) -> None:
    request = (f'name: "{MODEL}", position: {{x: {x}, y: {y}, z: 0.02}}, '
               f'orientation: {{x: 0, y: 0, z: {math.sin(yaw / 2.0)}, w: {math.cos(yaw / 2.0)}}}')
    subprocess.run(
        ["gz", "service", "-s", f"/world/{WORLD}/set_pose", "--reqtype", "gz.msgs.Pose",
         "--reptype", "gz.msgs.Boolean", "--timeout", "3000", "--req", request],
        capture_output=True, text=True,
    )


def connector_profile():
    """Curvature versus arc length of a real connector-0, from a route audit."""
    import glob
    for pattern in ("runtime/route_audit/phase17/*/collection-scan-*.json",
                    "runtime/route_audit/phase16/*/collection-scan-*.json"):
        for path in sorted(glob.glob(pattern)):
            try:
                plan = json.load(open(path, encoding="utf-8"))["plan"]
            except Exception:  # noqa: BLE001 - skip unreadable audits
                continue
            for segment in plan["segments"]:
                if segment["id"] != "connector-0" or len(segment["path"]["points"]) < 8:
                    continue
                poses = [q["pose"] for q in segment["path"]["points"]]
                samples, arc = [], 0.0
                for first, second in zip(poses, poses[1:]):
                    step = math.hypot(second["x_m"] - first["x_m"], second["y_m"] - first["y_m"])
                    if step < 1e-6:
                        continue
                    turn = ((second["yaw_rad"] - first["yaw_rad"] + math.pi)
                            % (2 * math.pi) - math.pi)
                    samples.append((arc, turn / step))
                    arc += step
                if samples:
                    return samples, arc
    return None, 0.0


def legs(mode: str) -> list[dict]:
    """One dict per independent leg; each is reset to START before it runs."""
    plan: list[dict] = []

    def leg(name, phases):
        plan.append({"name": name, "phases": phases})

    if mode in ("all", "straight"):
        for speed in (0.20, 0.35, 0.50):
            for rep in range(1, REPETITIONS + 1):
                leg(f"straight_v{speed:.2f}#{rep}", [("straight", speed, 0.0, 8.0)])

    if mode in ("all", "curvature"):
        for kappa in (0.25, 0.50, 1.00, 1.50, 1.80):
            for sign, tag in ((+1.0, "L"), (-1.0, "R")):
                for rep in range(1, REPETITIONS + 1):
                    leg(f"turn_k{kappa:.2f}_{tag}#{rep}", [
                        ("approach", 0.35, 0.0, 2.0),
                        ("turn", 0.35, sign * kappa, 6.0),
                        ("recovery", 0.35, 0.0, 2.0),
                    ])

    if mode in ("all", "speed"):
        for speed in (0.20, 0.50):
            for rep in range(1, REPETITIONS + 1):
                leg(f"turn_k1.00_v{speed:.2f}#{rep}", [
                    ("approach", speed, 0.0, 2.0),
                    ("turn", speed, 1.00, 6.0),
                    ("recovery", speed, 0.0, 2.0),
                ])

    if mode in ("all", "holdout"):
        leg("holdout_straight_v0.28", [("straight", 0.28, 0.0, 8.0)])
        for kappa in (0.75, 1.25):
            leg(f"holdout_turn_k{kappa:.2f}", [
                ("approach", 0.35, 0.0, 2.0),
                ("turn", 0.35, kappa, 6.0),
                ("recovery", 0.35, 0.0, 2.0),
            ])

    if mode in ("all", "connector"):
        samples, arc = connector_profile()
        if samples:
            for rep in range(1, REPETITIONS + 1):
                plan.append({"name": f"connector_0_replay#{rep}", "phases": [],
                             "profile": samples, "profile_length_m": arc, "speed": 0.35})
    return plan


class Characterization(Node):
    def __init__(self, path: str, mode: str) -> None:
        super().__init__("wheel_odometry_characterization")
        self._handle = open(path, "w", encoding="utf-8")
        self._drive = self.create_publisher(Twist, DRIVE_TOPIC, 10)
        self._joints: dict[str, float] = {}
        self._odom = self._filtered = None
        self._gyro_z = None
        self._truth = None
        self.create_subscription(JointState, "/joint_states", self._on_joints, 10)
        self.create_subscription(Odometry, "/diff_drive_controller/odom", self._on_odom, 10)
        self.create_subscription(Odometry, "/odometry/filtered", self._on_filtered, 10)
        self.create_subscription(Imu, "/imu/data", self._on_imu, 10)
        self.create_subscription(String, "/sim/robot_true_pose", self._on_truth, 1)
        self._legs = legs(mode)
        self._index = -1
        self._state = "reset"
        self._state_started: float | None = None
        self._phase = 0
        self._arc = 0.0
        self._rows = 0
        self._reset_ok = False
        self._baseline = None
        self._quiet_since: float | None = None
        self._last_truth_xy: tuple[float, float] | None = None
        self._max_truth_step = 0.0
        self._leg_rejected = False
        self.finished = False
        self.create_timer(0.02, self._tick)
        self.get_logger().info(f"{len(self._legs)} independent legs -> {path}")

    def _on_joints(self, message: JointState) -> None:
        for name, velocity in zip(message.name, message.velocity):
            if name in WHEELS:
                self._joints[name] = velocity

    def _on_odom(self, message): self._odom = message
    def _on_filtered(self, message): self._filtered = message
    def _on_imu(self, message): self._gyro_z = message.angular_velocity.z

    def _on_truth(self, message: String) -> None:
        try:
            data = json.loads(message.data)
            self._truth = (float(data["x"]), float(data["y"]), float(data["yaw"]))
        except (json.JSONDecodeError, KeyError, TypeError, ValueError):
            pass

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _command(self, v: float, omega: float) -> None:
        message = Twist()
        message.linear.x, message.angular.z = v, omega
        self._drive.publish(message)

    def _tick(self) -> None:
        if self.finished:
            return
        now = self._now()
        if self._state_started is None:
            self._state_started = now
        elapsed = now - self._state_started

        if self._state == "reset":
            self._command(0.0, 0.0)
            if elapsed < 0.5:
                return
            if elapsed < 0.6:
                self._index += 1
                if self._index >= len(self._legs):
                    self.finished = True
                    self.get_logger().info(f"done: {self._rows} samples")
                    return
                set_model_pose(*START)
                return
            # Settled means: placed where it was put, chassis still, wheels still,
            # and staying that way -- checked continuously rather than assumed
            # after a fixed wait.
            placed = self._truth is not None and math.dist(
                self._truth[:2], START[:2]) <= RESET_TOLERANCE_M
            still = self._last_truth_xy is not None and self._truth is not None and math.dist(
                self._truth[:2], self._last_truth_xy) < 0.002
            wheels_quiet = len(self._joints) >= 4 and all(
                abs(value) < SETTLE_WHEEL_RAD_S for value in self._joints.values())
            if placed and still and wheels_quiet:
                if self._quiet_since is None:
                    self._quiet_since = now
            else:
                self._quiet_since = None
            if self._truth is not None:
                self._last_truth_xy = self._truth[:2]
            if self._quiet_since is None or now - self._quiet_since < SETTLE_HOLD_S:
                if elapsed > 12.0:
                    self.get_logger().warn(
                        f"leg {self._legs[self._index]['name']}: never settled; leg rejected")
                    self._reset_ok = False
                    self._state, self._state_started = "run", now
                    self._leg_rejected = True
                    self._phase, self._arc = 0, 0.0
                    self._max_truth_step = 0.0
                    self._baseline = None
                return
            self._reset_ok = True
            self._leg_rejected = False
            self._max_truth_step = 0.0
            # Baselines captured AFTER settling, so nothing before this instant
            # can enter the measurement.
            self._baseline = {
                "truth": self._truth,
                "odom": (self._odom.pose.pose.position.x, self._odom.pose.pose.position.y)
                if self._odom else None,
            }
            self._state, self._state_started, self._phase, self._arc = "run", now, 0, 0.0
            return

        leg = self._legs[self._index]
        if leg.get("profile"):
            speed = leg["speed"]
            self._arc += speed * 0.02
            if self._arc >= leg["profile_length_m"]:
                self._finish_leg()
                return
            kappa = min(leg["profile"], key=lambda item: abs(item[0] - self._arc))[1]
            phase_name, v, omega = "connector", speed, speed * kappa
        else:
            total = 0.0
            phase_name = v = omega = None
            for name, speed, kappa, seconds in leg["phases"]:
                total += seconds
                if elapsed < total:
                    phase_name, v, omega = name, speed, speed * kappa
                    break
            if phase_name is None:
                self._finish_leg()
                return
            kappa = omega / v if v else 0.0

        self._command(v, omega)
        if self._truth is None or self._odom is None or len(self._joints) < 4:
            return
        if self._last_truth_xy is not None:
            step = math.dist(self._truth[:2], self._last_truth_xy)
            if step > DISCONTINUITY_M:
                self._leg_rejected = True
                self.get_logger().warn(
                    f"leg {leg['name']}: {step:.3f} m discontinuity inside the measurement "
                    f"window; leg rejected")
            self._max_truth_step = max(self._max_truth_step, step)
        self._last_truth_xy = self._truth[:2]
        row = {
            "t_s": now, "leg": leg["name"], "phase": phase_name, "reset_ok": self._reset_ok,
            "leg_valid": not self._leg_rejected,
            "baseline_truth": self._baseline["truth"] if self._baseline else None,
            "baseline_odom": self._baseline["odom"] if self._baseline else None,
            "commanded_v_mps": v, "commanded_omega_rad_s": omega,
            "commanded_kappa_per_m": kappa,
            "wheels_rad_s": {name: self._joints.get(name) for name in WHEELS},
            "wheel_radius_m": WHEEL_RADIUS_M,
            "odom_vx_mps": self._odom.twist.twist.linear.x,
            "odom_wz_rad_s": self._odom.twist.twist.angular.z,
            "odom_x_m": self._odom.pose.pose.position.x,
            "odom_y_m": self._odom.pose.pose.position.y,
            "odom_yaw_rad": yaw_of(self._odom.pose.pose.orientation),
            "gyro_wz_rad_s": self._gyro_z,
            "truth_x_m": self._truth[0], "truth_y_m": self._truth[1],
            "truth_yaw_rad": self._truth[2],
        }
        if self._filtered is not None:
            row.update({
                "ekf_x_m": self._filtered.pose.pose.position.x,
                "ekf_y_m": self._filtered.pose.pose.position.y,
                "ekf_yaw_rad": yaw_of(self._filtered.pose.pose.orientation),
            })
        self._handle.write(json.dumps(row) + "\n")
        self._rows += 1

    def _finish_leg(self) -> None:
        self._command(0.0, 0.0)
        self._handle.flush()
        self.get_logger().info(
            f"leg {self._legs[self._index]['name']} complete ({self._rows} samples so far)")
        self._quiet_since = None
        self._state, self._state_started = "reset", self._now()

    def destroy_node(self) -> bool:
        try:
            self._command(0.0, 0.0)
        except Exception:  # noqa: BLE001 - shutting down anyway
            pass
        self._handle.flush()
        self._handle.close()
        return super().destroy_node()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", default="runtime/wheel_odometry_characterization.jsonl")
    parser.add_argument("--mode", default="all",
                        choices=["all", "straight", "curvature", "speed", "holdout", "connector"])
    arguments = parser.parse_args()
    rclpy.init()
    node = Characterization(arguments.out, arguments.mode)
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
