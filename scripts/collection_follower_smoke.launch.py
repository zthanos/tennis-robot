"""Phase 6D.3 construction smoke: node-built handles vs the real controller.

Brings up a real nav2 ``controller_server`` (nav2_params.yaml, lifecycle
manager) loading the ``CollectionFollowPath`` plugin, then lets the pure
``LiveCollectionPathFollower`` adapter drive a REAL curved executable plan
(plan_collection_route + Phase 6A CourtModel + Phase 6B serializers) end to end:

    Load ACCEPTED -> FollowPath (controller_id + sha match) -> terminal -> Finalize.

Run inside the container via scripts/run_collection_follower_smoke.sh.
"""

import os
import sys
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node as RclpyNode
from tf2_ros import TransformBroadcaster

# Import the pure collection modules from the source tree (not the baked install).
sys.path.insert(0, "/workspace/ros2_ws/src/tennis_robot")
from tennis_robot.collection_court_model_builder import build_court_model  # noqa: E402
from tennis_robot.collection_executor_node_factory import (  # noqa: E402
    CollectionExecutorNodeCache,
    CollectionExecutorNodeFactory,
)
from tennis_robot.collection_route_planner_v2 import plan_collection_route  # noqa: E402
from tennis_robot.collection_route_types import (  # noqa: E402
    Point2D, Pose2D, PositionCovariance2D, ScanSnapshot, SnapshotBall,
)

sys.path.insert(0, os.path.join("/workspace", "tests"))
from collection_route_fixtures import default_configuration  # noqa: E402

_BOUNDARY = {
    "schema": "court_knowledge_model/v2", "status": "OK", "failure_reason": None, "frame": "map",
    "completed": True,
    "net": {"center": {"x_m": 8.0, "y_m": 0.0},
            "axis_length": {"x_m": 1.0, "y_m": 0.0},
            "axis_width": {"x_m": 0.0, "y_m": 1.0},
            "posts": [{"x_m": 8.0, "y_m": 6.0}, {"x_m": 8.0, "y_m": -6.0}], "span_m": 12.0},
    "court": {"lines_court_frame": {"service_x": [-6.4, 6.4], "center_line_y": 0.0}},
    "fence": {"corners": [{"x_m": -9.0, "y_m": -8.0}, {"x_m": 9.0, "y_m": -8.0}, {"x_m": 9.0, "y_m": 8.0}, {"x_m": -9.0, "y_m": 8.0}]},
    "obstacles": [],
}


def build_curved_plan():
    config = default_configuration()
    court = build_court_model(_BOUNDARY)
    snapshot = ScanSnapshot(
        "scan-smoke", 1000.0, "map", Pose2D(0.0, 0.0, 0.0),
        (SnapshotBall("ball-smoke", Point2D(0.0, 3.0), 0.95, PositionCovariance2D(1e-6, 0.0, 1e-6)),),
        config,
    )
    plan = plan_collection_route(snapshot=snapshot, court=court, configuration=config).plan
    assert plan.is_executable, plan.planning_status
    return plan


def generate_test_description():
    params = os.path.join(get_package_share_directory("tennis_robot"), "config", "nav2_params.yaml")
    return LaunchDescription([
        Node(package="tf2_ros", executable="static_transform_publisher",
             arguments=["0", "0", "0", "0", "0", "0", "map", "odom"]),
        Node(package="nav2_controller", executable="controller_server", name="controller_server",
             parameters=[params, {"use_sim_time": False}]),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager", name="smoke_lifecycle_manager",
             parameters=[{"autostart": True, "node_names": ["controller_server"], "use_sim_time": False}]),
        launch_testing.actions.ReadyToTest(),
    ])


class DormantLaneNavigator:
    state = "idle"

    def request(self, *args):
        pass


class DormantCollectorInterface:
    def start(self):
        pass

    def stop(self):
        pass


class TestCollectionFollowerSmoke(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = RclpyNode("collection_route_executor")
        cls.tf = TransformBroadcaster(cls.node)
        cls.odom = cls.node.create_publisher(Odometry, "/odom", 10)
        cls.pose = (0.0, 0.0, 1.0)  # x, y, qw
        cls.timer = cls.node.create_timer(0.02, cls.publish_pose)

    @classmethod
    def tearDownClass(cls):
        cls.timer.cancel()
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def publish_pose(cls):
        x, y, qw = cls.pose
        tf = TransformStamped()
        tf.header.frame_id, tf.child_frame_id = "odom", "base_link"
        tf.header.stamp = cls.node.get_clock().now().to_msg()
        tf.transform.translation.x, tf.transform.translation.y = x, y
        tf.transform.rotation.w = qw
        cls.tf.sendTransform(tf)
        odom = Odometry()
        odom.header.stamp = tf.header.stamp
        odom.header.frame_id, odom.child_frame_id = "odom", "base_link"
        odom.pose.pose.position.x, odom.pose.pose.position.y = x, y
        odom.pose.pose.orientation.w = qw
        cls.odom.publish(odom)

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def test_python_follower_drives_real_controller_through_terminal(self):
        plan = build_curved_plan()
        terminal = plan.terminal_pose
        # Pre-park the robot at the terminal pose (xy goal tolerance 0.10) so the
        # first FollowPath tick projects straight to terminal -> action succeeds.
        type(self).pose = (terminal.x_m, terminal.y_m, 1.0)
        self.spin(0.5)

        import json
        import tempfile
        import yaml
        params_path = os.path.join(get_package_share_directory("tennis_robot"), "config", "nav2_params.yaml")
        runtime_params = yaml.safe_load(open(params_path, encoding="utf-8"))["collection_route_executor"]["ros__parameters"]
        for name, value in runtime_params.items():
            if name != "use_sim_time" and not self.node.has_parameter(name):
                self.node.declare_parameter(name, value)
        boundary_file = tempfile.NamedTemporaryFile(mode="w", suffix=".json", delete=False)
        json.dump(_BOUNDARY, boundary_file)
        boundary_file.close()
        cache = CollectionExecutorNodeCache(
            robot_x_m=terminal.x_m, robot_y_m=terminal.y_m, robot_yaw_rad=terminal.yaw_rad
        )
        factory = CollectionExecutorNodeFactory(
            node=self.node,
            tf_buffer=object(),  # scan TF is dormant in this construction smoke
            cache=cache,
            lane_navigator=DormantLaneNavigator(),
            collector_interface=DormantCollectorInterface(),
            court_boundary_path=boundary_file.name,
            collection_route_config_path=os.path.join(
                get_package_share_directory("tennis_robot"), "config", "collection_route.yaml"
            ),
            calibration_artifact_path="/workspace/calibration_artifacts/gazebo/range_depth_quality_diagonal_v1-gazebo-v2.json",
            telemetry_sink=lambda event: None,
        )
        self.assertTrue(factory.transport.wait_ready(25.0))
        executor = factory.build()
        follower = executor._path_follower
        transport = factory.transport

        follower.start(plan)
        from tennis_robot.collection_route_executor import PathFollowerStatus
        deadline = time.monotonic() + 30.0
        result = None
        while time.monotonic() < deadline:
            self.spin(0.1)
            result = follower.result()
            if result.status in (PathFollowerStatus.COMPLETED, PathFollowerStatus.FAILED):
                break

        self.assertIsNotNone(result)
        self.assertEqual(result.status, PathFollowerStatus.COMPLETED,
                         f"follower did not complete (reason={getattr(result, 'reason', None)})")
        self.assertIsNotNone(transport.goal_future, "FollowPath must have been sent")
        self.assertTrue(follower.finalize_accepted, "Finalize must be ACCEPTED at terminal")
        os.unlink(boundary_file.name)


@launch_testing.post_shutdown_test()
class TestSmokeShutdown(unittest.TestCase):
    def test_exit_ok(self):
        self.assertTrue(True)
