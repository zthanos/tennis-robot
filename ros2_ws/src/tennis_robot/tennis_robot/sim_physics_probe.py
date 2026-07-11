"""Live Gazebo physics probe for intake / roller tuning."""

from __future__ import annotations

import argparse
import json
import math
import os
import time
from dataclasses import dataclass
from pathlib import Path
from typing import TextIO

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from ros_gz_interfaces.msg import Contacts
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from tf2_msgs.msg import TFMessage


BALL_PREFIX = "ball_"
BASE_LINK_HEIGHT_M = 0.045
BALL_RADIUS_M = 0.033
BALL_MASS_KG = 0.058
INTAKE_WHEEL_JOINTS = ("intake_wheel_left_joint", "intake_wheel_right_joint")


def _vec_mag(vec) -> float:
    return math.sqrt(vec.x * vec.x + vec.y * vec.y + vec.z * vec.z)


def _tuple_mag(vec: tuple[float, float, float]) -> float:
    return math.sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2])


def _yaw_from_quat(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def _world_to_base(
    point: tuple[float, float, float],
    robot_pose: tuple[float, float, float, float],
) -> tuple[float, float, float]:
    rx, ry, rz, ryaw = robot_pose
    x, y, z = point
    dx, dy = x - rx, y - ry
    cos_y, sin_y = math.cos(-ryaw), math.sin(-ryaw)
    return (cos_y * dx - sin_y * dy, sin_y * dx + cos_y * dy, z - rz)


@dataclass
class ContactStats:
    samples: int = 0
    active_samples: int = 0
    max_points: int = 0
    max_depth_m: float = 0.0
    max_force_n: float = 0.0
    last_points_base: list[tuple[float, float, float]] | None = None


class SimPhysicsProbe(Node):
    def __init__(self, duration_s: float, print_period_s: float, jsonl_path: str | None) -> None:
        super().__init__("sim_physics_probe")
        self._duration_s = duration_s
        self._print_period_s = print_period_s
        self._started_ns = self.get_clock().now().nanoseconds
        self._stats = ContactStats()
        self._wheel_contact_samples: dict[str, int] = {"left": 0, "right": 0}
        self.done = False
        self._static_balls: dict[str, tuple[float, float, float]] = {}
        self._balls: dict[str, tuple[float, float, float]] = {}
        self._robot_pose: tuple[float, float, float, float] | None = None
        self._bench_robot_initial: tuple[float, float, float, float] | None = None
        if os.getenv("INTAKE_BENCH_ROBOT_X") is not None:
            self._bench_robot_initial = (
                float(os.getenv("INTAKE_BENCH_ROBOT_X", "0.0")),
                float(os.getenv("INTAKE_BENCH_ROBOT_Y", "0.0")),
                float(os.getenv("INTAKE_BENCH_BASE_LINK_Z", "0.045")),
                float(os.getenv("INTAKE_BENCH_ROBOT_YAW", "0.0")),
            )
        self._bench_drive_speed = float(os.getenv("INTAKE_BENCH_DRIVE_SPEED", "0.0"))
        self._closest_ball: dict[str, object] | None = None
        self._roller_contact = False
        self._intake_beam = False
        self._joint_velocity: float | None = None
        self._joint_effort: float | None = None
        self._wheel_velocities: dict[str, float] = {}
        self._ball_history: dict[str, tuple[float, tuple[float, float, float], tuple[float, float, float] | None]] = {}
        self._jsonl: TextIO | None = None
        bench_ball_x = os.getenv("INTAKE_BENCH_BALL_X")
        bench_ball_y = os.getenv("INTAKE_BENCH_BALL_Y")
        if bench_ball_x is not None and bench_ball_y is not None:
            self._static_balls["bench_ball"] = (
                float(bench_ball_x),
                float(bench_ball_y),
                float(os.getenv("INTAKE_BENCH_BALL_Z", "0.033")),
            )
            self._balls.update(self._static_balls)
        if jsonl_path:
            path = Path(jsonl_path)
            path.parent.mkdir(parents=True, exist_ok=True)
            self._jsonl = path.open("w", encoding="utf-8")

        # Dual-wheel side-pinch geometry (docs/dual-wheel-intake-design-el.md).
        self._nip_x = float(os.getenv("INTAKE_NIP_X_M", "0.590"))
        self._wheel_radius = float(os.getenv("INTAKE_WHEEL_RADIUS_M", "0.060"))
        self._wheel_gap = float(os.getenv("INTAKE_WHEEL_GAP_M", "0.060"))
        self._wheel_y = self._wheel_gap / 2.0 + self._wheel_radius
        # Interference per side on a centred 66 mm ball (positive = squeeze).
        self._squeeze_m = (2.0 * BALL_RADIUS_M - self._wheel_gap) / 2.0
        # Horizontal dx (ahead of the nip plane) where a centred ball first
        # touches the wheels: |ball - wheel_centre| = Rw + Rball.
        reach = self._wheel_radius + BALL_RADIUS_M
        self._nominal_bite_dx_m = math.sqrt(max(0.0, reach**2 - self._wheel_y**2))
        self._lip_x = float(
            os.getenv("INTAKE_RAMP_ENTRY_X_M", str(self._nip_x + 0.020))
        )
        self._ramp_clear_run_m = float(os.getenv("INTAKE_RAMP_CLEAR_RUN_M", "0.030"))
        self._front_lip_zone_m = float(os.getenv("INTAKE_FRONT_LIP_ZONE_M", "0.008"))
        self._front_lip_min_x = self._lip_x - min(self._front_lip_zone_m, self._ramp_clear_run_m)

        self.create_subscription(String, "/sim/balls", self._on_balls, 10)
        self.create_subscription(TFMessage, "/gz/pose_info", self._on_pose_info, 10)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(Odometry, "/diff_drive_controller/odom", self._on_odom, 10)
        self.create_subscription(Bool, "/sim/roller_contact", self._on_roller_contact, 10)
        self.create_subscription(
            Bool, "/collector/intake_beam_broken", self._on_intake_beam, 10
        )
        self.create_subscription(JointState, "/joint_states", self._on_joint_states, 10)
        self.create_subscription(
            Contacts, "/gz/roller_contact_0",
            lambda msg: self._on_contacts(msg, "left"), 10,
        )
        self.create_subscription(
            Contacts, "/gz/roller_contact_1",
            lambda msg: self._on_contacts(msg, "right"), 10,
        )
        self.create_subscription(Contacts, "/gz/lip_contact_0", self._on_lip_contacts, 10)

        self.create_timer(print_period_s, self._print_summary)
        if duration_s > 0.0:
            self.create_timer(duration_s, self._finish)

        self.get_logger().info(
            "probe started (dual-wheel): nip_x=%.0f mm, wheel r=%.0f mm, "
            "gap=%.0f mm (squeeze %.1f mm/side), ramp entry=%.0f mm, "
            "nominal bite dx=%.1f mm"
            % (
                self._nip_x * 1000.0,
                self._wheel_radius * 1000.0,
                self._wheel_gap * 1000.0,
                self._squeeze_m * 1000.0,
                self._lip_x * 1000.0,
                self._nominal_bite_dx_m * 1000.0,
            )
        )

    def _on_balls(self, msg: String) -> None:
        try:
            balls = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        parsed = {}
        for ball in balls:
            name = str(ball.get("def", ""))
            if name.startswith(BALL_PREFIX):
                point = (
                    float(ball.get("x", 0.0)),
                    float(ball.get("y", 0.0)),
                    float(ball.get("z", 0.0)),
                )
                parsed[name] = point
                self._update_ball_velocity(name, point)
        self._balls = {**self._static_balls, **parsed}

    def _on_pose_info(self, msg: TFMessage) -> None:
        for transform in msg.transforms:
            name = transform.child_frame_id.split("::")[-1]
            t = transform.transform.translation
            if name.startswith(BALL_PREFIX):
                point = (t.x, t.y, t.z)
                self._balls[name] = point
                self._update_ball_velocity(name, point)
                continue
            if name != "tennis_robot":
                continue
            q = transform.transform.rotation
            self._robot_pose = (t.x, t.y, t.z, _yaw_from_quat(q))

    def _on_odom(self, msg: Odometry) -> None:
        if self._bench_robot_initial is not None:
            return
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self._robot_pose = (p.x, p.y, p.z, _yaw_from_quat(q))

    def _on_roller_contact(self, msg: Bool) -> None:
        self._roller_contact = msg.data

    def _on_intake_beam(self, msg: Bool) -> None:
        self._intake_beam = msg.data

    def _on_joint_states(self, msg: JointState) -> None:
        names = list(msg.name)
        for joint in INTAKE_WHEEL_JOINTS:
            try:
                index = names.index(joint)
            except ValueError:
                continue
            if index < len(msg.velocity):
                self._wheel_velocities[joint] = float(msg.velocity[index])
            if index < len(msg.effort):
                self._joint_effort = msg.effort[index]
        if self._wheel_velocities:
            # Representative magnitude for surface-speed math (the two wheels
            # counter-rotate; their magnitudes should match).
            self._joint_velocity = max(
                (abs(v) for v in self._wheel_velocities.values()), default=None
            )

    def _update_ball_velocity(self, name: str, point: tuple[float, float, float]) -> None:
        now = self.get_clock().now().nanoseconds / 1e9
        prev = self._ball_history.get(name)
        velocity = prev[2] if prev else None
        if prev is not None:
            prev_t, prev_point, _ = prev
            dt = now - prev_t
            if dt > 1e-4:
                velocity = tuple((point[i] - prev_point[i]) / dt for i in range(3))
        self._ball_history[name] = (now, point, velocity)

    def _ball_velocity(self, name: str | None) -> tuple[float, float, float] | None:
        if not name:
            return None
        rec = self._ball_history.get(name)
        return rec[2] if rec else None

    def _current_robot_pose(self) -> tuple[float, float, float, float] | None:
        if self._robot_pose is not None:
            return self._robot_pose
        if self._bench_robot_initial is None:
            return None
        elapsed = (self.get_clock().now().nanoseconds - self._started_ns) / 1e9
        x0, y0, z0, yaw = self._bench_robot_initial
        return (
            x0 + math.cos(yaw) * self._bench_drive_speed * elapsed,
            y0 + math.sin(yaw) * self._bench_drive_speed * elapsed,
            z0,
            yaw,
        )

    def _contact_ball_name(self, names: str) -> str | None:
        for part in names.replace("::", " ").replace("/", " ").split():
            if part.startswith(BALL_PREFIX):
                return part
        index = names.find(BALL_PREFIX)
        if index < 0:
            return None
        tail = names[index:]
        return tail.split()[0].split("_collision")[0]

    def _write_contact_log(
        self,
        sample_type: str,
        ball_name: str | None,
        contact_names: list[str],
        points_world: list[tuple[float, float, float]],
        points_base: list[tuple[float, float, float]] | None,
        max_depth: float,
        max_force: float,
        wheel: str | None = None,
    ) -> None:
        if self._jsonl is None:
            return
        vel = self._ball_velocity(ball_name)
        speed = _tuple_mag(vel) if vel else None
        roller_surface_speed = (
            abs(self._joint_velocity) * self._wheel_radius
            if self._joint_velocity is not None
            else None
        )
        loss_mps = (
            max(0.0, roller_surface_speed - speed)
            if roller_surface_speed is not None and speed is not None
            else None
        )
        kinetic_j = 0.5 * BALL_MASS_KG * speed * speed if speed is not None else None
        rec = {
            "type": sample_type,
            "wheel": wheel,
            "t_wall": round(time.time(), 6),
            "t_s": round((self.get_clock().now().nanoseconds - self._started_ns) / 1e9, 4),
            "ball": ball_name,
            "collisions": contact_names,
            "points_world": [
                [round(x, 5), round(y, 5), round(z, 5)] for x, y, z in points_world
            ],
            "points_base": [
                [round(x, 5), round(y, 5), round(z, 5)] for x, y, z in points_base
            ] if points_base else None,
            "max_depth_m": round(max_depth, 6),
            "max_force_n": round(max_force, 4),
            "joint_velocity_rad_s": round(self._joint_velocity, 4)
            if self._joint_velocity is not None else None,
            "roller_surface_speed_m_s": round(roller_surface_speed, 4)
            if roller_surface_speed is not None else None,
            "ball_velocity_m_s": [round(v, 4) for v in vel] if vel else None,
            "ball_speed_m_s": round(speed, 4) if speed is not None else None,
            "ball_kinetic_j": round(kinetic_j, 5) if kinetic_j is not None else None,
            "surface_to_ball_speed_loss_m_s": round(loss_mps, 4) if loss_mps is not None else None,
            "wheel_joint_velocities_rad_s": {
                j: round(v, 4) for j, v in self._wheel_velocities.items()
            } or None,
            "geometry": {
                "nip_x_m": round(self._nip_x, 5),
                "wheel_radius_m": round(self._wheel_radius, 5),
                "wheel_gap_m": round(self._wheel_gap, 5),
                "wheel_y_m": round(self._wheel_y, 5),
                "squeeze_per_side_m": round(self._squeeze_m, 5),
                "ramp_entry_x_m": round(self._lip_x, 5),
                "front_lip_min_x_m": round(self._front_lip_min_x, 5),
                "ramp_clear_run_m": round(self._ramp_clear_run_m, 5),
                "nominal_bite_dx_m": round(self._nominal_bite_dx_m, 5),
            },
        }
        self._jsonl.write(json.dumps(rec) + "\n")
        self._jsonl.flush()

    def _extract_ball_contacts(
        self, msg: Contacts
    ) -> tuple[str | None, list[str], list[tuple[float, float, float]], float, float]:
        points_world: list[tuple[float, float, float]] = []
        max_depth = 0.0
        max_force = 0.0
        contact_names: list[str] = []
        ball_name: str | None = None
        for contact in msg.contacts:
            names = f"{contact.collision1.name} {contact.collision2.name}"
            if BALL_PREFIX not in names:
                continue
            contact_names.append(names)
            ball_name = ball_name or self._contact_ball_name(names)
            points_world.extend((p.x, p.y, p.z) for p in contact.positions)
            if contact.depths:
                max_depth = max(max_depth, max(contact.depths))
            for wrench in contact.wrenches:
                max_force = max(
                    max_force,
                    _vec_mag(wrench.body_1_wrench.force),
                    _vec_mag(wrench.body_2_wrench.force),
                )
        return ball_name, contact_names, points_world, max_depth, max_force

    def _on_contacts(self, msg: Contacts, wheel: str = "left") -> None:
        self._stats.samples += 1
        ball_name, contact_names, points_world, max_depth, max_force = (
            self._extract_ball_contacts(msg)
        )

        if not points_world:
            return

        self._stats.active_samples += 1
        self._wheel_contact_samples[wheel] = self._wheel_contact_samples.get(wheel, 0) + 1
        self._stats.max_points = max(self._stats.max_points, len(points_world))
        self._stats.max_depth_m = max(self._stats.max_depth_m, max_depth)
        self._stats.max_force_n = max(self._stats.max_force_n, max_force)
        points_base = None
        robot_pose = self._current_robot_pose()
        if robot_pose is not None:
            points_base = [
                _world_to_base(point, robot_pose) for point in points_world
            ]
            self._stats.last_points_base = points_base
        self._write_contact_log(
            "roller_contact_sample",
            ball_name,
            contact_names,
            points_world,
            points_base,
            max_depth,
            max_force,
            wheel=wheel,
        )

    def _on_lip_contacts(self, msg: Contacts) -> None:
        ball_name, contact_names, points_world, max_depth, max_force = (
            self._extract_ball_contacts(msg)
        )
        if not points_world:
            return
        points_base = None
        robot_pose = self._current_robot_pose()
        if robot_pose is not None:
            points_base = [
                _world_to_base(point, robot_pose) for point in points_world
            ]
        self._write_contact_log(
            "lip_contact_sample",
            ball_name,
            contact_names,
            points_world,
            points_base,
            max_depth,
            max_force,
        )
        if points_base is None:
            return

        front_pairs = [
            (world, base)
            for world, base in zip(points_world, points_base)
            if base[0] >= self._front_lip_min_x
        ]
        ramp_pairs = [
            (world, base)
            for world, base in zip(points_world, points_base)
            if base[0] < self._front_lip_min_x
        ]
        if front_pairs:
            self._write_contact_log(
                "front_lip_contact_sample",
                ball_name,
                contact_names,
                [world for world, _base in front_pairs],
                [base for _world, base in front_pairs],
                max_depth,
                max_force,
            )
        if ramp_pairs:
            self._write_contact_log(
                "ramp_guide_contact_sample",
                ball_name,
                contact_names,
                [world for world, _base in ramp_pairs],
                [base for _world, base in ramp_pairs],
                max_depth,
                max_force,
            )

    def _nearest_ball_base(self) -> tuple[str, tuple[float, float, float]] | None:
        robot_pose = self._current_robot_pose()
        if robot_pose is None:
            return None
        best: tuple[str, tuple[float, float, float], float] | None = None
        for name, point in self._balls.items():
            base = _world_to_base(point, robot_pose)
            score = abs(base[0] - self._nip_x) + abs(base[1])
            if best is None or score < best[2]:
                best = (name, base, score)
        if best is None:
            return None
        return best[0], best[1]

    def _print_summary(self) -> None:
        elapsed = (self.get_clock().now().nanoseconds - self._started_ns) / 1e9
        nearest = self._nearest_ball_base()
        ball_text = "ball=n/a"
        if nearest is not None:
            name, (x, y, z) = nearest
            dx = x - self._nip_x
            candidate = {
                "name": name,
                "t_s": round(elapsed, 4),
                "base_xyz_m": [round(x, 5), round(y, 5), round(z, 5)],
                "dx_to_nip_m": round(dx, 5),
                "lateral_y_m": round(y, 5),
            }
            if self._closest_ball is None or abs(dx) < abs(
                float(self._closest_ball["dx_to_nip_m"])
            ):
                self._closest_ball = candidate
            ball_text = (
                f"ball={name} base=({x:+.3f},{y:+.3f},{z:+.3f}) "
                f"to_nip dx={dx * 1000:+.0f}mm lat={y * 1000:+.0f}mm"
            )

        contact_text = "last_contact_base=n/a"
        if self._stats.last_points_base:
            xs = [p[0] for p in self._stats.last_points_base]
            ys = [p[1] for p in self._stats.last_points_base]
            zs = [p[2] for p in self._stats.last_points_base]
            contact_text = (
                "last_contact_base="
                f"x[{min(xs):+.3f},{max(xs):+.3f}] "
                f"y[{min(ys):+.3f},{max(ys):+.3f}] "
                f"z[{min(zs):+.3f},{max(zs):+.3f}]"
            )

        joint = "wheels=n/a"
        if self._wheel_velocities:
            left = self._wheel_velocities.get(INTAKE_WHEEL_JOINTS[0])
            right = self._wheel_velocities.get(INTAKE_WHEEL_JOINTS[1])
            joint = (
                f"wheels L={left:+.2f} R={right:+.2f} rad/s"
                if left is not None and right is not None
                else f"wheels partial={self._wheel_velocities}"
            )

        self.get_logger().info(
            f"t={elapsed:5.1f}s contact={int(self._roller_contact)} "
            f"beam={int(self._intake_beam)} max_depth={self._stats.max_depth_m * 1000:.2f}mm "
            f"max_force={self._stats.max_force_n:.2f}N points_max={self._stats.max_points} "
            f"{joint} contacts L={self._wheel_contact_samples.get('left', 0)} "
            f"R={self._wheel_contact_samples.get('right', 0)} "
            f"bite_dx={self._nominal_bite_dx_m * 1000:.0f}mm "
            f"{ball_text} {contact_text}"
        )

    def _finish(self) -> None:
        self._print_summary()
        self.get_logger().info(
            "final: contact_samples=%d/%d max_depth=%.2fmm max_force=%.2fN"
            % (
                self._stats.active_samples,
                self._stats.samples,
                self._stats.max_depth_m * 1000.0,
                self._stats.max_force_n,
            )
        )
        if self._jsonl is not None:
            nearest = self._nearest_ball_base()
            nearest_ball = None
            if nearest is not None:
                name, (x, y, z) = nearest
                nearest_ball = {
                    "name": name,
                    "base_xyz_m": [round(x, 5), round(y, 5), round(z, 5)],
                    "dx_to_nip_m": round(x - self._nip_x, 5),
                    "lateral_y_m": round(y, 5),
                }
            self._jsonl.write(json.dumps({
                "type": "summary",
                "t_wall": round(time.time(), 6),
                "contact_samples": self._stats.active_samples,
                "total_contact_msgs": self._stats.samples,
                "max_depth_m": round(self._stats.max_depth_m, 6),
                "max_force_n": round(self._stats.max_force_n, 4),
                "joint_velocity_rad_s": round(self._joint_velocity, 4)
                if self._joint_velocity is not None else None,
                "joint_effort": round(self._joint_effort, 4)
                if self._joint_effort is not None and math.isfinite(self._joint_effort) else None,
                "wheel_contact_samples": dict(self._wheel_contact_samples),
                "wheel_joint_velocities_rad_s": {
                    j: round(v, 4) for j, v in self._wheel_velocities.items()
                } or None,
                "nip_x_m": round(self._nip_x, 5),
                "wheel_gap_m": round(self._wheel_gap, 5),
                "wheel_radius_m": round(self._wheel_radius, 5),
                "ramp_entry_x_m": round(self._lip_x, 5),
                "nominal_bite_dx_m": round(self._nominal_bite_dx_m, 5),
                "nearest_ball": nearest_ball,
                "closest_ball": self._closest_ball,
            }) + "\n")
            self._jsonl.close()
            self._jsonl = None
        self.done = True


def main(args=None) -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=0.0)
    parser.add_argument("--period", type=float, default=0.5)
    parser.add_argument("--jsonl", default=None, help="Optional per-contact JSONL log path.")
    parsed, remaining = parser.parse_known_args(args)

    rclpy.init(args=remaining)
    node = SimPhysicsProbe(parsed.duration, parsed.period, parsed.jsonl)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
