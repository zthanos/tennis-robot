"""Live Gazebo physics probe for intake / roller tuning."""

from __future__ import annotations

import argparse
import json
import math
import os
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from ros_gz_interfaces.msg import Contacts
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from tf2_msgs.msg import TFMessage


BALL_PREFIX = "ball_"
ROLLER_BASE_X_M = 0.60
ROLLER_BASE_Z_M = 0.067
BASE_LINK_HEIGHT_M = 0.045
ROLLER_RADIUS_M = 0.045
BALL_RADIUS_M = 0.033


def _vec_mag(vec) -> float:
    return math.sqrt(vec.x * vec.x + vec.y * vec.y + vec.z * vec.z)


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
    def __init__(self, duration_s: float, print_period_s: float) -> None:
        super().__init__("sim_physics_probe")
        self._duration_s = duration_s
        self._print_period_s = print_period_s
        self._started_ns = self.get_clock().now().nanoseconds
        self._stats = ContactStats()
        self.done = False
        self._balls: dict[str, tuple[float, float, float]] = {}
        self._robot_pose: tuple[float, float, float, float] | None = None
        self._roller_contact = False
        self._intake_beam = False
        self._joint_velocity: float | None = None
        self._joint_effort: float | None = None

        self._roller_x = ROLLER_BASE_X_M + float(
            os.getenv("INTAKE_ROLLER_X_OFFSET_M", "0.0")
        )
        self._roller_z = ROLLER_BASE_Z_M + float(
            os.getenv("INTAKE_ROLLER_Z_OFFSET_M", "0.0")
        )
        self._roller_center_ground_z = BASE_LINK_HEIGHT_M + self._roller_z
        self._roller_bottom_ground_z = self._roller_center_ground_z - ROLLER_RADIUS_M
        self._ball_clearance_m = self._roller_bottom_ground_z - (2.0 * BALL_RADIUS_M)

        self.create_subscription(String, "/sim/balls", self._on_balls, 10)
        self.create_subscription(TFMessage, "/gz/pose_info", self._on_pose_info, 10)
        self.create_subscription(Bool, "/sim/roller_contact", self._on_roller_contact, 10)
        self.create_subscription(
            Bool, "/collector/intake_beam_broken", self._on_intake_beam, 10
        )
        self.create_subscription(JointState, "/joint_states", self._on_joint_states, 10)
        for index in range(8):
            self.create_subscription(
                Contacts,
                f"/gz/roller_contact_{index}",
                self._on_contacts,
                10,
            )

        self.create_timer(print_period_s, self._print_summary)
        if duration_s > 0.0:
            self.create_timer(duration_s, self._finish)

        self.get_logger().info(
            "probe started: roller center z=%.1f mm, bottom clearance=%.1f mm, "
            "ball fit margin=%.1f mm"
            % (
                self._roller_center_ground_z * 1000.0,
                self._roller_bottom_ground_z * 1000.0,
                self._ball_clearance_m * 1000.0,
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
                parsed[name] = (
                    float(ball.get("x", 0.0)),
                    float(ball.get("y", 0.0)),
                    float(ball.get("z", 0.0)),
                )
        self._balls = parsed

    def _on_pose_info(self, msg: TFMessage) -> None:
        for transform in msg.transforms:
            name = transform.child_frame_id.split("::")[-1]
            t = transform.transform.translation
            if name.startswith(BALL_PREFIX):
                self._balls[name] = (t.x, t.y, t.z)
                continue
            if name != "tennis_robot":
                continue
            q = transform.transform.rotation
            self._robot_pose = (t.x, t.y, t.z, _yaw_from_quat(q))

    def _on_roller_contact(self, msg: Bool) -> None:
        self._roller_contact = msg.data

    def _on_intake_beam(self, msg: Bool) -> None:
        self._intake_beam = msg.data

    def _on_joint_states(self, msg: JointState) -> None:
        try:
            index = list(msg.name).index("lift_wheel_joint")
        except ValueError:
            return
        if index < len(msg.velocity):
            self._joint_velocity = msg.velocity[index]
        if index < len(msg.effort):
            self._joint_effort = msg.effort[index]

    def _on_contacts(self, msg: Contacts) -> None:
        self._stats.samples += 1
        points_world: list[tuple[float, float, float]] = []
        max_depth = 0.0
        max_force = 0.0
        for contact in msg.contacts:
            names = f"{contact.collision1.name} {contact.collision2.name}"
            if BALL_PREFIX not in names:
                continue
            points_world.extend((p.x, p.y, p.z) for p in contact.positions)
            if contact.depths:
                max_depth = max(max_depth, max(contact.depths))
            for wrench in contact.wrenches:
                max_force = max(
                    max_force,
                    _vec_mag(wrench.body_1_wrench.force),
                    _vec_mag(wrench.body_2_wrench.force),
                )

        if not points_world:
            return

        self._stats.active_samples += 1
        self._stats.max_points = max(self._stats.max_points, len(points_world))
        self._stats.max_depth_m = max(self._stats.max_depth_m, max_depth)
        self._stats.max_force_n = max(self._stats.max_force_n, max_force)
        if self._robot_pose is not None:
            self._stats.last_points_base = [
                _world_to_base(point, self._robot_pose) for point in points_world
            ]

    def _nearest_ball_base(self) -> tuple[str, tuple[float, float, float]] | None:
        if self._robot_pose is None:
            return None
        best: tuple[str, tuple[float, float, float], float] | None = None
        for name, point in self._balls.items():
            base = _world_to_base(point, self._robot_pose)
            score = abs(base[0] - self._roller_x) + abs(base[1]) + abs(base[2] - self._roller_z)
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
            dx = x - self._roller_x
            dz = z - self._roller_z
            ball_text = (
                f"ball={name} base=({x:+.3f},{y:+.3f},{z:+.3f}) "
                f"to_roller dx={dx * 1000:+.0f}mm dz={dz * 1000:+.0f}mm"
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

        joint = "joint=n/a"
        if self._joint_velocity is not None:
            joint = f"joint_vel={self._joint_velocity:+.2f}rad/s"
            if self._joint_effort is not None:
                joint += f" effort={self._joint_effort:+.3f}"

        self.get_logger().info(
            f"t={elapsed:5.1f}s contact={int(self._roller_contact)} "
            f"beam={int(self._intake_beam)} max_depth={self._stats.max_depth_m * 1000:.2f}mm "
            f"max_force={self._stats.max_force_n:.2f}N points_max={self._stats.max_points} "
            f"{joint} {ball_text} {contact_text}"
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
        self.done = True


def main(args=None) -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=0.0)
    parser.add_argument("--period", type=float, default=0.5)
    parsed, remaining = parser.parse_known_args(args)

    rclpy.init(args=remaining)
    node = SimPhysicsProbe(parsed.duration, parsed.period)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
