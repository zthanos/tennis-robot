"""Gazebo adapter node: IR LaserScan → IrReadings, pose info → /sim/balls.

Subscribes:
  /gz/ir_left/scan   (sensor_msgs/LaserScan, 1 ray)
  /gz/ir_right/scan  (sensor_msgs/LaserScan, 1 ray)
  /gz/pose_info      (tf2_msgs/TFMessage — world model poses from Gazebo)

Publishes:
  /ir/readings       (tennis_robot_msgs/IrReadings)
  /collector/intake_beam_broken (std_msgs/Bool)
  /sim/balls         (std_msgs/String, JSON list of {name, x, y})
  /sim/ball_markers  (visualization_msgs/MarkerArray, base_link frame —
                      ground-truth balls for RViz, incl. z so a ball riding
                      the scoop ramp is visible in the FrontFollow view)
  /sim/roller_contact_markers (visualization_msgs/MarkerArray, red spheres at
                               real Gazebo roller contact points)
  /sim/roller_contact (std_msgs/Bool, short held contact heartbeat)
"""

from __future__ import annotations

import json
import math
import os
import subprocess

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker, MarkerArray
from ros_gz_interfaces.msg import Contacts

from tennis_robot_msgs.msg import IrReadings

_WORLD_NAME = os.getenv("GZ_WORLD_NAME", "tennis_court")

# IR beam-break: range <= threshold means a ball is present → value 1000
_IR_MAX_RANGE_M = 0.22
_BALL_PREFIX = "ball_"


def _range_to_ir_value(range_m: float) -> float:
    """Convert a proximity range reading to a 0–1000 IR value (1000 = object present)."""
    if not math.isfinite(range_m) or range_m > _IR_MAX_RANGE_M:
        return 0.0
    return 1000.0 * max(0.0, 1.0 - range_m / _IR_MAX_RANGE_M)


class GazeboExtrasNode(Node):
    def __init__(self) -> None:
        super().__init__("gazebo_extras_node")

        self._ir_left = 0.0
        self._ir_right = 0.0
        self._intake_ir_left = 0.0
        self._intake_ir_right = 0.0
        # name -> {def,x,y,z}. /gz/pose_info messages often carry only the
        # entities that MOVED this cycle, so ball poses are merged into this
        # dict instead of rebuilding a list per message (a rebuild made
        # /sim/balls flap to [] whenever only the robot was moving).
        self._balls: dict[str, dict] = {}
        self._collected: set[str] = set()

        # Roller-first intake: collection is confirmed at the BASKET beam pair
        # (hopper entry), matching the Concept-A doc's collection_confirmed
        # signal. The throat beams (/gz/ir_left, /gz/ir_right) now sit at the
        # nip for jam/partial-capture detection and are intentionally NOT the
        # collection signal — a ball at the lip is not a collected ball.
        self.create_subscription(LaserScan, "/gz/basket_ir_left/scan", self._on_ir_left, 10)
        self.create_subscription(LaserScan, "/gz/basket_ir_right/scan", self._on_ir_right, 10)
        self.create_subscription(
            LaserScan, "/gz/ir_left/scan", self._on_intake_ir_left, 10
        )
        self.create_subscription(
            LaserScan, "/gz/ir_right/scan", self._on_intake_ir_right, 10
        )
        self.create_subscription(TFMessage, "/gz/pose_info", self._on_pose_info, 10)
        # Robot pose comes from /odom, not /gz/pose_info: the ros_gz bridge
        # leaves child_frame_id empty on every transform in this Gazebo
        # version, so name-matching "tennis_robot" there never fires and
        # self._robot_pose stayed None forever — which silently starved both
        # marker publishers below (they no-op without a robot pose).
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(String, "/ball/collected", self._on_ball_collected, 10)

        self._pub_ir = self.create_publisher(IrReadings, "/ir/readings", 10)
        self._pub_intake_beam = self.create_publisher(
            Bool, "/collector/intake_beam_broken", 10
        )
        self._pub_balls = self.create_publisher(String, "/sim/balls", 1)
        self._pub_markers = self.create_publisher(MarkerArray, "/sim/ball_markers", 1)
        self._pub_contact_markers = self.create_publisher(
            MarkerArray, "/sim/roller_contact_markers", 10
        )
        self._pub_roller_contact = self.create_publisher(Bool, "/sim/roller_contact", 10)
        self._robot_pose: tuple[float, float, float, float] | None = None  # x, y, z, yaw
        self._contact_points: list[tuple[float, float, float]] = []
        self._last_contact_ns = 0
        for index in range(8):
            self.create_subscription(
                Contacts,
                f"/gz/roller_contact_{index}",
                self._on_roller_contacts,
                10,
            )

        self.create_timer(0.05, self._publish)
        self.get_logger().info("gazebo_extras_node started")

    def _on_ir_left(self, msg: LaserScan) -> None:
        r = msg.ranges[0] if msg.ranges else float("inf")
        self._ir_left = _range_to_ir_value(r)

    def _on_ir_right(self, msg: LaserScan) -> None:
        r = msg.ranges[0] if msg.ranges else float("inf")
        self._ir_right = _range_to_ir_value(r)

    def _on_intake_ir_left(self, msg: LaserScan) -> None:
        r = msg.ranges[0] if msg.ranges else float("inf")
        self._intake_ir_left = _range_to_ir_value(r)

    def _on_intake_ir_right(self, msg: LaserScan) -> None:
        r = msg.ranges[0] if msg.ranges else float("inf")
        self._intake_ir_right = _range_to_ir_value(r)

    def _on_pose_info(self, msg: TFMessage) -> None:
        for transform in msg.transforms:
            name = transform.child_frame_id
            t = transform.transform.translation
            # Accept both plain ("ball_02") and scoped ("tennis_court::ball_02")
            # entity names — pose_info naming differs between gz versions.
            leaf = name.split("::")[-1]
            if not leaf.startswith(_BALL_PREFIX) or leaf in self._collected:
                continue
            self._balls[leaf] = {
                "def": leaf, "x": round(t.x, 4), "y": round(t.y, 4), "z": round(t.z, 4)
            }

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._robot_pose = (p.x, p.y, p.z, yaw)

    def _on_ball_collected(self, msg: String) -> None:
        """Remove a collected ball: drop it from /sim/balls and delete the
        model from the Gazebo world (nothing else consumed /ball/collected in
        the Gazebo port — it was a Webots-era animation hook)."""
        name = msg.data.strip()
        if not name or name in self._collected:
            return
        self._collected.add(name)
        self._balls.pop(name, None)
        req = f'name: "{name}" type: MODEL'
        try:
            subprocess.Popen(
                [
                    "gz", "service", "-s", f"/world/{_WORLD_NAME}/remove",
                    "--reqtype", "gz.msgs.Entity", "--reptype", "gz.msgs.Boolean",
                    "--timeout", "2000", "--req", req,
                ],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            self.get_logger().info(f"ball collected -> removing {name} from world")
        except OSError as exc:
            self.get_logger().warning(f"gz remove failed for {name}: {exc}")

    def _publish(self) -> None:
        ir_msg = IrReadings()
        ir_msg.left = self._ir_left
        ir_msg.right = self._ir_right
        self._pub_ir.publish(ir_msg)
        self._pub_intake_beam.publish(
            Bool(
                data=(
                    self._intake_ir_left > 500.0
                    or self._intake_ir_right > 500.0
                )
            )
        )

        balls_msg = String()
        balls_msg.data = json.dumps(list(self._balls.values()))
        self._pub_balls.publish(balls_msg)
        self._publish_ball_markers()
        self._publish_roller_contacts()

    def _on_roller_contacts(self, msg: Contacts) -> None:
        points: list[tuple[float, float, float]] = []
        for contact in msg.contacts:
            # Ignore roller contact with the robot itself / court; the signal
            # is specifically for verifying paddle-to-tennis-ball engagement.
            names = f"{contact.collision1} {contact.collision2}"
            if _BALL_PREFIX not in names:
                continue
            points.extend((p.x, p.y, p.z) for p in contact.positions)
        if points:
            self._contact_points = points
            self._last_contact_ns = self.get_clock().now().nanoseconds

    def _publish_roller_contacts(self) -> None:
        now = self.get_clock().now()
        active = (
            bool(self._contact_points)
            and now.nanoseconds - self._last_contact_ns <= 300_000_000
        )
        self._pub_roller_contact.publish(Bool(data=active))
        arr = MarkerArray()
        clear = Marker()
        clear.header.frame_id = "base_footprint"
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        if active and self._robot_pose is not None:
            rx, ry, rz, ryaw = self._robot_pose
            cos_y, sin_y = math.cos(-ryaw), math.sin(-ryaw)
            for index, (x, y, z) in enumerate(self._contact_points):
                dx, dy = x - rx, y - ry
                marker = Marker()
                marker.header.frame_id = "base_footprint"
                marker.header.stamp = now.to_msg()
                marker.ns = "roller_contacts"
                marker.id = index
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position.x = cos_y * dx - sin_y * dy
                marker.pose.position.y = sin_y * dx + cos_y * dy
                marker.pose.position.z = z - rz
                marker.pose.orientation.w = 1.0
                marker.scale.x = marker.scale.y = marker.scale.z = 0.018
                marker.color.r, marker.color.g = 1.0, 0.05
                marker.color.b, marker.color.a = 0.02, 1.0
                arr.markers.append(marker)
        self._pub_contact_markers.publish(arr)

    def _publish_ball_markers(self) -> None:
        """Ground-truth ball markers in the base_link frame for RViz. Relative
        to the robot on purpose: no gz-world <-> map alignment is needed, and a
        ball riding the scoop ramp shows up right in the intake."""
        if self._robot_pose is None:
            return
        rx, ry, rz, ryaw = self._robot_pose
        cos_y, sin_y = math.cos(-ryaw), math.sin(-ryaw)
        arr = MarkerArray()
        clear = Marker()
        clear.header.frame_id = "base_footprint"
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        now = self.get_clock().now().to_msg()
        for i, ball in enumerate(self._balls.values()):
            dx, dy = ball["x"] - rx, ball["y"] - ry
            m = Marker()
            m.header.frame_id = "base_footprint"
            m.header.stamp = now
            m.ns = "sim_balls"
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = cos_y * dx - sin_y * dy
            m.pose.position.y = sin_y * dx + cos_y * dy
            m.pose.position.z = ball.get("z", 0.0325) - rz
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.065
            m.color.r, m.color.g, m.color.b, m.color.a = 0.8, 1.0, 0.1, 1.0
            arr.markers.append(m)
        self._pub_markers.publish(arr)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GazeboExtrasNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
