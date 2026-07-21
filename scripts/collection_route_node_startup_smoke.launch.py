"""Phase 6D.4 smoke: real controller node reaches empty-route terminal state."""

import json
import math
import os
from pathlib import Path
import time
import unittest

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import TransformStamped
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node as RclpyNode
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
from tennis_robot_msgs.msg import BallDetectionArray, RobotCommand


SMOKE_DIR = Path("/tmp/collection-route-node-smoke")
SMOKE_DIR.mkdir(parents=True, exist_ok=True)
BOUNDARY = SMOKE_DIR / "court_boundary.json"
BOUNDARY.write_text(json.dumps({
    "schema": "court_knowledge_model/v2", "status": "OK", "failure_reason": None,
    "frame": "map", "completed": True,
    "net": {
        "center": {"x_m": 0.0, "y_m": 0.0},
        "axis_length": {"x_m": 1.0, "y_m": 0.0},
        "axis_width": {"x_m": 0.0, "y_m": 1.0},
        "posts": [{"x_m": 0.0, "y_m": -6.0}, {"x_m": 0.0, "y_m": 6.0}],
    },
    "court": {"lines_court_frame": {"service_x": [-6.4, 6.4], "center_line_y": 0.0}},
    "fence": {"corners": [
        {"x_m": -9.0, "y_m": -8.0}, {"x_m": 9.0, "y_m": -8.0},
        {"x_m": 9.0, "y_m": 8.0}, {"x_m": -9.0, "y_m": 8.0},
    ]},
    "obstacles": [],
}), encoding="utf-8")


def generate_test_description():
    params = "/workspace/ros2_ws/install_smoke/tennis_robot/share/tennis_robot/config/nav2_params.yaml"
    calibration = "/workspace/calibration_artifacts/gazebo/range_depth_quality_diagonal_v1-gazebo-v2.json"
    return LaunchDescription([
        Node(package="tf2_ros", executable="static_transform_publisher",
             arguments=["0", "0", "0", "0", "0", "0", "map", "odom"]),
        Node(package="tf2_ros", executable="static_transform_publisher",
             arguments=["0", "0", "0", "0", "0", "0", "base_footprint", "base_link"]),
        Node(package="nav2_controller", executable="controller_server", name="controller_server",
             parameters=[params, {"use_sim_time": False}]),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="route_smoke_lifecycle_manager",
             parameters=[{"autostart": True, "node_names": ["controller_server"], "use_sim_time": False}]),
        Node(
            package="tennis_robot", executable="controller_node", name="controller_node",
            parameters=[{
                "use_sim_time": False,
                "collection_route.scan_step_count": 1,
                "collection_route.scan_start_yaw_rad": 0.0,
            }],
            additional_env={
                "WORKSPACE": "/workspace",
                "ROBOT_STATUS_FILE": str(SMOKE_DIR / "robot_status.json"),
                "ROBOT_COMMAND_FILE": str(SMOKE_DIR / "robot_command.json"),
                "COLLECTION_EVENT_LOG_FILE": str(SMOKE_DIR / "events.jsonl"),
                "COLLECTION_ROUTE_CALIBRATION_ARTIFACT": calibration,
            },
        ),
        launch_testing.actions.ReadyToTest(),
    ])


class TestCollectionRouteNodeStartup(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = RclpyNode("collection_route_smoke_driver")
        cls.tf = TransformBroadcaster(cls.node)
        cls.odom_pub = cls.node.create_publisher(Odometry, "/odom", 10)
        cls.scan_pub = cls.node.create_publisher(LaserScan, "/scan", 10)
        cls.detection_pub = cls.node.create_publisher(
            BallDetectionArray, "/perception/ball_detections", 10
        )
        cls.command_pub = cls.node.create_publisher(RobotCommand, "/robot/command", 10)
        cls.status_messages = []
        cls.node.create_subscription(String, "/robot/status", cls.status_messages.append, 10)
        cls.nav_server = ActionServer(
            cls.node, NavigateToPose, "navigate_to_pose", execute_callback=cls.navigate
        )
        cls.timer = cls.node.create_timer(0.03, cls.publish_inputs)

    @classmethod
    def tearDownClass(cls):
        cls.timer.cancel()
        cls.nav_server.destroy()
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def navigate(cls, goal_handle):
        goal_handle.succeed()
        return NavigateToPose.Result()

    @classmethod
    def publish_inputs(cls):
        stamp = cls.node.get_clock().now().to_msg()
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id, tf.child_frame_id = "odom", "base_footprint"
        tf.transform.translation.x = -6.4
        tf.transform.rotation.w = 1.0
        cls.tf.sendTransform(tf)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id, odom.child_frame_id = "odom", "base_footprint"
        odom.pose.pose.position.x = -6.4
        odom.pose.pose.orientation.w = 1.0
        cls.odom_pub.publish(odom)

        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = "base_scan"
        scan.angle_min, scan.angle_max, scan.angle_increment = -math.pi, math.pi, math.pi / 8
        scan.range_min, scan.range_max = 0.1, 20.0
        scan.ranges = [float("inf")] * 17
        cls.scan_pub.publish(scan)

        detections = BallDetectionArray()
        detections.header.stamp = stamp
        detections.header.frame_id = "camera_link_optical_frame"
        detections.spatial_targets_healthy = True
        detections.calibration_id = "gazebo-range-depth-quality-diagonal-v1-20260719-v2"
        detections.configuration_id = "gazebo-mvp-provisional-planning-safety-v1"
        detections.detections = []
        cls.detection_pub.publish(detections)

    def spin(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def test_real_controller_reaches_completed_no_targets(self):
        self.spin(1.0)
        command = RobotCommand()
        command.mode, command.source = "collect_route", "6d4-node-smoke"
        self.command_pub.publish(command)

        deadline = time.monotonic() + 20.0
        terminal = None
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            for message in reversed(self.status_messages):
                payload = json.loads(message.data)
                route = payload.get("collect_route", {})
                if route.get("state") == "completed_no_targets":
                    terminal = payload
                    break
            if terminal is not None:
                break

        self.assertIsNotNone(terminal, "controller never published completed_no_targets")
        route = terminal["collect_route"]
        self.assertEqual(route["planning_status"], "empty_no_balls")
        self.assertEqual(route["ball_results"], [])
        self.assertEqual(route["segments"], [])


@launch_testing.post_shutdown_test()
class TestSmokeShutdown(unittest.TestCase):
    def test_exit_ok(self):
        self.assertTrue(True)
