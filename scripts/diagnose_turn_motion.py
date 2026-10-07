#!/usr/bin/env python3
"""Sim-time motion diagnostic for the tennis robot.

Run inside the Gazebo / ROS 2 environment:
  python3 /workspace/scripts/diagnose_turn_motion.py

The test intentionally uses ROS simulation time, not wall-clock time. Gazebo may
run below real time, so wall-clock based tests can make healthy motion look slow.
Start from a fresh spawn, away from the net, before trusting the numbers.
"""

import argparse
import math
import time

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


WHEEL_RADIUS_M = 0.085
WHEEL_SEPARATION_M = 0.70


def yaw_from_quat(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def angle_delta(a, b):
    return (a - b + math.pi) % (2.0 * math.pi) - math.pi


def stamp_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class MotionDiagnostic:
    def __init__(self, cmd_topic, stamped_cmd):
        self.node = rclpy.create_node("diagnose_turn_motion")
        self.stamped_cmd = stamped_cmd
        command_type = TwistStamped if stamped_cmd else Twist
        self.pub = self.node.create_publisher(command_type, cmd_topic, 10)
        self.odom = None
        self.joint_state = None
        self.cmd_out = None
        self.node.create_subscription(
            Odometry, "/diff_drive_controller/odom", self._on_odom, qos_profile_sensor_data
        )
        self.node.create_subscription(
            JointState, "/joint_states", self._on_joint_state, qos_profile_sensor_data
        )
        output_type = TwistStamped if stamped_cmd else Twist
        self.node.create_subscription(output_type, "/diff_drive_controller/cmd_vel_out", self._on_cmd_out, 10)

    def _command(self, twist):
        if not self.stamped_cmd:
            return twist
        msg = TwistStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.twist = twist
        return msg

    def _on_odom(self, msg):
        self.odom = msg

    def _on_joint_state(self, msg):
        self.joint_state = msg

    def _on_cmd_out(self, msg):
        self.cmd_out = msg

    def wait_for_data(self, timeout_s):
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.odom is not None and self.joint_state is not None:
                return True
        return False

    def stop(self):
        msg = Twist()
        for _ in range(10):
            self.pub.publish(self._command(msg))
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def run_command(self, label, twist, sim_duration_s, max_wall_s):
        if not self.wait_for_data(10.0):
            print(f"{label}: missing odom or joint_states")
            return

        start_odom = self.odom
        start_joint_state = self.joint_state
        start_stamp = stamp_sec(start_odom.header.stamp)
        start_wall = time.time()
        samples = []

        while time.time() - start_wall < max_wall_s:
            self.pub.publish(self._command(twist))
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if self.odom is None or self.joint_state is None:
                continue
            now_stamp = stamp_sec(self.odom.header.stamp)
            samples.append((time.time() - start_wall, now_stamp, self.odom, self.joint_state, self.cmd_out))
            if now_stamp - start_stamp >= sim_duration_s:
                break

        self.stop()

        if not samples:
            print(f"{label}: no samples")
            return

        end_odom = samples[-1][2]
        end_joint_state = samples[-1][3]
        sim_dt = stamp_sec(end_odom.header.stamp) - start_stamp
        wall_dt = time.time() - start_wall

        start_x = start_odom.pose.pose.position.x
        end_x = end_odom.pose.pose.position.x
        start_yaw = yaw_from_quat(start_odom.pose.pose.orientation)
        end_yaw = yaw_from_quat(end_odom.pose.pose.orientation)
        dx = end_x - start_x
        dyaw = angle_delta(end_yaw, start_yaw)

        start_positions = dict(zip(start_joint_state.name, start_joint_state.position))
        end_positions = dict(zip(end_joint_state.name, end_joint_state.position))
        end_velocities = dict(zip(end_joint_state.name, end_joint_state.velocity))
        # 4WD skid-steer: sample one wheel per side (front+rear share a command).
        left_delta = end_positions.get("rear_left_wheel_joint", 0.0) - start_positions.get("rear_left_wheel_joint", 0.0)
        right_delta = end_positions.get("rear_right_wheel_joint", 0.0) - start_positions.get("rear_right_wheel_joint", 0.0)
        distance_from_wheels = WHEEL_RADIUS_M * (left_delta + right_delta) / 2.0
        yaw_from_wheels = WHEEL_RADIUS_M * (right_delta - left_delta) / WHEEL_SEPARATION_M

        print(f"\n=== {label} ===")
        print(f"sim_time_delta_s={sim_dt:.3f} wall_time_delta_s={wall_dt:.3f}")
        print(f"odom_delta_x_m={dx:+.3f} odom_yaw_delta_rad={dyaw:+.3f} odom_yaw_delta_deg={math.degrees(dyaw):+.1f}")
        print(f"joint_delta_rad_left={left_delta:+.3f} joint_delta_rad_right={right_delta:+.3f}")
        print(f"distance_from_joint_positions_m={distance_from_wheels:+.3f}")
        print(f"yaw_from_joint_positions_rad={yaw_from_wheels:+.3f} yaw_from_joint_positions_deg={math.degrees(yaw_from_wheels):+.1f}")
        if sim_dt > 0:
            print(f"linear_rate_sim_m_s={dx / sim_dt:+.3f}")
            print(f"yaw_rate_sim_rad_s={dyaw / sim_dt:+.3f}")
        print(
            "last_wheel_velocity_rad_s="
            f"left={end_velocities.get('rear_left_wheel_joint', 0.0):+.3f} "
            f"right={end_velocities.get('rear_right_wheel_joint', 0.0):+.3f}"
        )
        if self.cmd_out is not None:
            cmd_out = self.cmd_out.twist if self.stamped_cmd else self.cmd_out
            print(
                "last_cmd_vel_out="
                f"linear.x={cmd_out.linear.x:+.3f} angular.z={cmd_out.angular.z:+.3f}"
            )

    def destroy(self):
        self.node.destroy_node()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--cmd-topic", default="/cmd_vel_teleop")
    parser.add_argument(
        "--stamped-cmd",
        action="store_true",
        help="Publish geometry_msgs/TwistStamped (required by Jazzy diff_drive_controller).",
    )
    parser.add_argument("--sim-seconds", type=float, default=2.0)
    parser.add_argument("--max-wall-seconds", type=float, default=45.0)
    parser.add_argument("--linear-x", type=float, default=0.3)
    parser.add_argument("--angular-z", type=float, default=0.8)
    args = parser.parse_args()

    rclpy.init()
    diag = MotionDiagnostic(args.cmd_topic, args.stamped_cmd)
    try:
        linear = Twist()
        linear.linear.x = args.linear_x
        diag.run_command("forward", linear, args.sim_seconds, args.max_wall_seconds)
        time.sleep(1.0)

        left = Twist()
        left.angular.z = args.angular_z
        diag.run_command("turn_left", left, args.sim_seconds, args.max_wall_seconds)
        time.sleep(1.0)

        right = Twist()
        right.angular.z = -args.angular_z
        diag.run_command("turn_right", right, args.sim_seconds, args.max_wall_seconds)
    finally:
        diag.stop()
        diag.destroy()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
