"""Thin Webots controller that bridges hardware devices to/from ROS 2 topics.

This is the only file that imports the Webots `controller` module. All behavior
logic lives in the ROS 2 nodes (ros2_ws/src/tennis_robot/).

Run by setting the robot's controller to "webots_bridge" in the world file,
or by keeping "ball_detector" for the legacy path.

Requires rclpy in the Webots Python environment (see Dockerfile.ros2-webots).
"""

from __future__ import annotations

import json
import math
import os
import struct
import time

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time as RosTime
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Header, String

from controller import Camera, DistanceSensor, Lidar, Motor, RangeFinder, Supervisor
from tennis_robot_msgs.msg import CollectorCmd, IrReadings, RobotCommand

TIME_STEP_MS = 32
MAX_SPEED_RAD_S = float(os.getenv("ROBOT_MAX_WHEEL_SPEED_RAD_S", "6.28"))
WHEEL_RADIUS_M = 0.11
TRACK_WIDTH_M = 0.48
COLLECTION_ANIMATION_S = 0.75
COLLECTION_PATH_LOCAL = (
    (0.66, 0.0, -0.165),
    (0.62, 0.0, -0.135),
    (0.54, 0.0, -0.120),
    (0.36, 0.0, -0.045),
    (0.17, 0.0, 0.120),
)


def _ros_time(sec: float) -> RosTime:
    t = RosTime()
    t.sec = int(sec)
    t.nanosec = int((sec - t.sec) * 1e9)
    return t


def _header(frame_id: str) -> Header:
    h = Header()
    h.stamp = _ros_time(time.time())
    h.frame_id = frame_id
    return h


def _rot_to_quaternion(m: list[float]) -> tuple[float, float, float, float]:
    """Convert Webots 3x3 orientation matrix (row-major, 9 floats) to xyzw quaternion."""
    r00, r01, r02 = m[0], m[1], m[2]
    r10, r11, r12 = m[3], m[4], m[5]
    r20, r21, r22 = m[6], m[7], m[8]
    trace = r00 + r11 + r22
    if trace > 0:
        s = 0.5 / math.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (r21 - r12) * s
        y = (r02 - r20) * s
        z = (r10 - r01) * s
    elif r00 > r11 and r00 > r22:
        s = 2.0 * math.sqrt(1.0 + r00 - r11 - r22)
        w = (r21 - r12) / s
        x = 0.25 * s
        y = (r01 + r10) / s
        z = (r02 + r20) / s
    elif r11 > r22:
        s = 2.0 * math.sqrt(1.0 + r11 - r00 - r22)
        w = (r02 - r20) / s
        x = (r01 + r10) / s
        y = 0.25 * s
        z = (r12 + r21) / s
    else:
        s = 2.0 * math.sqrt(1.0 + r22 - r00 - r11)
        w = (r10 - r01) / s
        x = (r02 + r20) / s
        y = (r12 + r21) / s
        z = 0.25 * s
    return x, y, z, w


class WebotsBridgeNode(Node):
    def __init__(self, robot: Supervisor) -> None:
        super().__init__("webots_bridge")
        self._robot = robot
        self._robot_node = robot.getSelf()

        self._camera: Camera = robot.getDevice("front_camera")
        self._depth: RangeFinder | None = robot.getDevice("front_depth")
        self._lidar: Lidar | None = robot.getDevice("front_lidar")
        self._left_motor: Motor = robot.getDevice("left_wheel_motor")
        self._right_motor: Motor = robot.getDevice("right_wheel_motor")
        self._lift_motor: Motor | None = robot.getDevice("lift_wheel_motor")
        self._ir_left: DistanceSensor | None = robot.getDevice("ir_intake_left")
        self._ir_right: DistanceSensor | None = robot.getDevice("ir_intake_right")
        self._anim_ball = robot.getFromDef("COLLECTOR_ANIMATION_BALL")

        self._camera.enable(TIME_STEP_MS)
        if self._depth is not None:
            self._depth.enable(TIME_STEP_MS)
        if self._lidar is not None:
            self._lidar.enable(TIME_STEP_MS)
        if self._ir_left is not None:
            self._ir_left.enable(TIME_STEP_MS)
        if self._ir_right is not None:
            self._ir_right.enable(TIME_STEP_MS)

        self._left_motor.setPosition(math.inf)
        self._right_motor.setPosition(math.inf)
        self._left_motor.setVelocity(0.0)
        self._right_motor.setVelocity(0.0)
        if self._lift_motor is not None:
            self._lift_motor.setPosition(math.inf)
            self._lift_motor.setVelocity(0.0)

        # Publishers
        self._pub_img = self.create_publisher(Image, "/camera/image_raw", 1)
        self._pub_depth = self.create_publisher(Image, "/camera/depth", 1)
        self._pub_scan = self.create_publisher(LaserScan, "/scan", 1)
        self._pub_odom = self.create_publisher(Odometry, "/odom", 1)
        self._pub_ir = self.create_publisher(IrReadings, "/ir/readings", 10)
        self._pub_sim_balls = self.create_publisher(String, "/sim/balls", 1)

        # Subscribers
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 1)
        self.create_subscription(CollectorCmd, "/collector/cmd", self._on_collector_cmd, 1)
        self.create_subscription(String, "/ball/collected", self._on_ball_collected, 10)

        self._collection_animation: dict | None = None
        self.get_logger().info("webots_bridge started")

    # ── actuator callbacks ─────────────────────────────────────────────────────

    def _on_cmd_vel(self, msg: Twist) -> None:
        linear = msg.linear.x
        angular = msg.angular.z
        right_wheel = (linear - angular * TRACK_WIDTH_M / 2) / WHEEL_RADIUS_M
        left_wheel = (linear + angular * TRACK_WIDTH_M / 2) / WHEEL_RADIUS_M
        # Webots device naming is historically swapped: left_wheel_motor is on robot's right.
        self._left_motor.setVelocity(max(-MAX_SPEED_RAD_S, min(MAX_SPEED_RAD_S, right_wheel)))
        self._right_motor.setVelocity(max(-MAX_SPEED_RAD_S, min(MAX_SPEED_RAD_S, left_wheel)))

    def _on_collector_cmd(self, msg: CollectorCmd) -> None:
        if self._lift_motor is not None:
            speed = max(-MAX_SPEED_RAD_S, min(MAX_SPEED_RAD_S, msg.lift_wheel_speed * 4.0))
            self._lift_motor.setVelocity(speed)

    def _on_ball_collected(self, msg: String) -> None:
        ball_def = msg.data
        node = self._robot.getFromDef(ball_def)
        if node is None:
            return
        node.remove()
        self._collection_animation = {"elapsed_s": 0.0}
        if self._anim_ball is not None:
            self._anim_ball.getField("translation").setSFVec3f(list(COLLECTION_PATH_LOCAL[0]))
        self.get_logger().info(f"collected {ball_def}")

    # ── per-step publish ───────────────────────────────────────────────────────

    def step(self, dt_s: float) -> None:
        self._publish_camera()
        self._publish_depth()
        self._publish_lidar()
        self._publish_odom()
        self._publish_ir()
        self._publish_sim_balls()
        self._update_collection_animation(dt_s)
        rclpy.spin_once(self, timeout_sec=0)

    def _publish_camera(self) -> None:
        w = self._camera.getWidth()
        h = self._camera.getHeight()
        raw = self._camera.getImage()
        if raw is None:
            return
        msg = Image()
        msg.header = _header("camera_link")
        msg.width = w
        msg.height = h
        msg.encoding = "bgra8"
        msg.step = w * 4
        msg.data = list(raw)
        self._pub_img.publish(msg)

    def _publish_depth(self) -> None:
        if self._depth is None:
            return
        w = self._depth.getWidth()
        h = self._depth.getHeight()
        raw = self._depth.getRangeImage()
        if raw is None:
            return
        msg = Image()
        msg.header = _header("camera_link")
        msg.width = w
        msg.height = h
        msg.encoding = "32FC1"
        msg.step = w * 4
        msg.data = list(struct.pack(f"{w * h}f", *raw))
        self._pub_depth.publish(msg)

    def _publish_lidar(self) -> None:
        if self._lidar is None:
            return
        ranges = self._lidar.getRangeImage()
        if ranges is None:
            return
        n = len(ranges)
        fov = self._lidar.getFov()
        msg = LaserScan()
        msg.header = _header("lidar_link")
        msg.angle_min = -fov / 2
        msg.angle_max = fov / 2
        msg.angle_increment = fov / max(1, n - 1)
        msg.range_min = float(self._lidar.getMinRange())
        msg.range_max = float(self._lidar.getMaxRange())
        msg.ranges = [float(r) for r in ranges]
        self._pub_scan.publish(msg)

    def _publish_odom(self) -> None:
        pos = self._robot_node.getPosition()
        ori = self._robot_node.getOrientation()
        qx, qy, qz, qw = _rot_to_quaternion(ori)
        msg = Odometry()
        msg.header = _header("odom")
        msg.child_frame_id = "base_link"
        msg.pose.pose.position.x = float(pos[0])
        msg.pose.pose.position.y = float(pos[1])
        msg.pose.pose.position.z = float(pos[2])
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        self._pub_odom.publish(msg)

    def _publish_ir(self) -> None:
        msg = IrReadings()
        msg.left = float(self._ir_left.getValue()) if self._ir_left is not None else 0.0
        msg.right = float(self._ir_right.getValue()) if self._ir_right is not None else 0.0
        self._pub_ir.publish(msg)

    def _publish_sim_balls(self) -> None:
        balls: list[dict] = []
        for i in range(100):
            node = self._robot.getFromDef(f"TENNIS_BALL_{i:02d}")
            if node is None:
                continue
            if self._collection_animation and self._collection_animation.get("index") == i:
                continue
            x, y, _ = node.getPosition()
            balls.append({"def": f"TENNIS_BALL_{i:02d}", "x": float(x), "y": float(y)})
        msg = String()
        msg.data = json.dumps(balls)
        self._pub_sim_balls.publish(msg)

    def _update_collection_animation(self, dt_s: float) -> None:
        if self._collection_animation is None or self._anim_ball is None:
            return
        elapsed = self._collection_animation["elapsed_s"] + dt_s
        self._collection_animation["elapsed_s"] = elapsed
        progress = min(1.0, elapsed / COLLECTION_ANIMATION_S)
        n = len(COLLECTION_PATH_LOCAL) - 1
        scaled = min(n - 1e-9, progress * n)
        idx = int(scaled)
        t = scaled - idx
        a, b = COLLECTION_PATH_LOCAL[idx], COLLECTION_PATH_LOCAL[idx + 1]
        pos = [a[j] + (b[j] - a[j]) * t for j in range(3)]
        self._anim_ball.getField("translation").setSFVec3f(pos)
        if progress >= 1.0:
            self._anim_ball.getField("translation").setSFVec3f([0.0, 0.0, -1.0])
            self._collection_animation = None


def main() -> None:
    rclpy.init()
    robot = Supervisor()
    node = WebotsBridgeNode(robot)

    while robot.step(TIME_STEP_MS) != -1:
        node.step(TIME_STEP_MS / 1000.0)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
