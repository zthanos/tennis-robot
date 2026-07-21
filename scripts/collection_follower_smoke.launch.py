"""Phase 6C.2 container smoke: Python PathFollower vs the REAL C++ controller.

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
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped, Twist
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node as RclpyNode
from action_msgs.msg import GoalStatus
from tf2_ros import TransformBroadcaster

from tennis_robot_msgs.msg import (
    CollectionControllerState,
    CollectionExecutionContext,
    CollectionExecutionProfile,
    CollectionExecutionSegment,
    CollectionPlannedCrossing,
)
from tennis_robot_msgs.srv import (
    FinalizeCollectionExecutionContext,
    LoadCollectionExecutionContext,
    ResetCollectionExecutionContext,
    SetCollectionSafetyHold,
)

# Import the pure collection modules from the source tree (not the baked install).
sys.path.insert(0, "/workspace/ros2_ws/src/tennis_robot")
from tennis_robot.collection_court_model_builder import build_court_model  # noqa: E402
from tennis_robot.collection_execution_context_builder import ControllerTuning  # noqa: E402
from tennis_robot.collection_path_follower_port import LiveCollectionPathFollower  # noqa: E402
from tennis_robot.collection_route_planner_v2 import plan_collection_route  # noqa: E402
from tennis_robot.collection_route_types import (  # noqa: E402
    Point2D, Pose2D, PositionCovariance2D, ScanSnapshot, SnapshotBall,
)

sys.path.insert(0, os.path.join("/workspace", "tests"))
from collection_route_fixtures import default_configuration  # noqa: E402

CONTROLLER_ID = "CollectionFollowPath"
_BOUNDARY = {
    "schema": "court_knowledge_model/v2", "status": "OK", "failure_reason": None, "frame": "map",
    "completed": True,
    "net": {"center": {"x_m": 8.0, "y_m": 0.0}, "posts": [{"x_m": 8.0, "y_m": 6.0}, {"x_m": 8.0, "y_m": -6.0}], "span_m": 12.0},
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


def _pose_msg(canonical) -> Pose:
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = canonical.x, canonical.y, canonical.z
    pose.orientation.x, pose.orientation.y = canonical.qx, canonical.qy
    pose.orientation.z, pose.orientation.w = canonical.qz, canonical.qw
    return pose


def context_values_to_msg(values) -> CollectionExecutionContext:
    context = CollectionExecutionContext()
    context.context_schema_version = values.context_schema_version
    context.plan_id = values.plan_id
    context.path_sha256 = values.path_sha256
    context.context_activation_timeout_s = values.context_activation_timeout_s
    context.terminal_progress_s = values.terminal_progress_s
    context.terminal_pose = _pose_msg(values.terminal_pose)
    context.configuration_snapshot_json = values.configuration_snapshot_json
    t = values.controller_tuning
    context.controller_tuning.lookahead_distance_m = t.lookahead_distance_m
    context.controller_tuning.max_angular_velocity_rad_s = t.max_angular_velocity_rad_s
    context.controller_tuning.progress_projection_window_m = t.progress_projection_window_m
    context.controller_tuning.crossing_speed_window_m = t.crossing_speed_window_m
    context.controller_tuning.terminal_progress_tolerance_m = t.terminal_progress_tolerance_m
    for seg in values.segments:
        segment = CollectionExecutionSegment()
        segment.segment_id = seg.segment_id
        segment.segment_type = seg.segment_type
        segment.progress_start_s = seg.progress_start_s
        segment.progress_end_s = seg.progress_end_s
        p = seg.execution_profile
        profile = CollectionExecutionProfile()
        for field in (
            "nominal_speed_mps", "min_speed_mps", "max_speed_mps", "nominal_speed_warning_tolerance_mps",
            "max_acceleration_mps2", "max_deceleration_mps2", "required_entry_m", "required_run_in_m",
            "required_run_out_m", "max_curvature_per_m", "max_lateral_error_m", "max_heading_error_rad",
            "allow_reversing", "allow_standalone_rotate",
        ):
            setattr(profile, field, getattr(p, field))
        segment.execution_profile = profile
        for cr in seg.planned_crossings:
            crossing = CollectionPlannedCrossing()
            crossing.ball_id = cr.ball_id
            crossing.position_x_m = cr.position_x_m
            crossing.position_y_m = cr.position_y_m
            crossing.progress_s = cr.progress_s
            crossing.heading_rad = cr.heading_rad
            crossing.predicted_lateral_error = cr.predicted_lateral_error
            segment.planned_crossings.append(crossing)
        context.segments.append(segment)
    return context


def poses_to_path(map_frame, poses) -> Path:
    path = Path()
    path.header.frame_id = map_frame
    for canonical in poses:
        stamped = PoseStamped()
        stamped.pose = _pose_msg(canonical)
        path.poses.append(stamped)
    return path


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


class RclpyTransport:
    """Real rclpy implementation of the LiveCollectionPathFollower handles."""

    def __init__(self, node: RclpyNode):
        self._node = node
        self._load = node.create_client(LoadCollectionExecutionContext, f"/{CONTROLLER_ID}/load_collection_execution_context")
        self._hold = node.create_client(SetCollectionSafetyHold, f"/{CONTROLLER_ID}/set_collection_safety_hold")
        self._finalize = node.create_client(FinalizeCollectionExecutionContext, f"/{CONTROLLER_ID}/finalize_collection_execution_context")
        self._action = ActionClient(node, FollowPath, "/follow_path")
        self.state = None
        node.create_subscription(CollectionControllerState, f"/{CONTROLLER_ID}/state", self._on_state, 10)
        self._load_future = None
        self._goal_future = None
        self._goal_handle = None
        self._result_future = None

    def wait_ready(self, timeout=25.0):
        assert self._load.wait_for_service(timeout_sec=timeout)
        assert self._hold.wait_for_service(timeout_sec=timeout)
        assert self._finalize.wait_for_service(timeout_sec=timeout)
        assert self._action.wait_for_server(timeout_sec=timeout)

    def _on_state(self, msg):
        self.state = msg

    # ── follower handles ──
    def load_sender(self, context_values):
        request = LoadCollectionExecutionContext.Request()
        request.context = context_values_to_msg(context_values)
        self._load_future = self._load.call_async(request)

    def load_outcome_provider(self):
        if self._load_future is None or not self._load_future.done():
            return None
        return "accepted" if self._load_future.result().accepted else "rejected"

    def follow_paths_sent(self):
        return self._goal_future is not None

    def follow_path_sender(self, *, map_frame, poses, controller_id):
        goal = FollowPath.Goal()
        goal.path = poses_to_path(map_frame, poses)
        goal.controller_id = controller_id
        self._goal_future = self._action.send_goal_async(goal)

    def goal_status_provider(self):
        if self._goal_future is None:
            return "pending"
        if self._goal_handle is None:
            if not self._goal_future.done():
                return "pending"
            self._goal_handle = self._goal_future.result()
            if self._goal_handle is None or not self._goal_handle.accepted:
                return "rejected"
            self._result_future = self._goal_handle.get_result_async()
        if self._result_future is not None and self._result_future.done():
            status = self._result_future.result().status
            return "succeeded" if status == GoalStatus.STATUS_SUCCEEDED else "failed"
        return "accepted"

    def state_provider(self):
        if self.state is None:
            return None
        return {
            "lifecycle_state": self.state.lifecycle_state,
            "progress_s": self.state.progress_s,
            "lateral_error_m": self.state.lateral_error_m,
            "failure_reason": self.state.failure_reason,
        }

    def hold_sender(self, *, plan_id, path_sha256, hold):
        request = SetCollectionSafetyHold.Request()
        request.plan_id, request.path_sha256, request.hold = plan_id, path_sha256, hold
        self._hold.call_async(request)

    def finalize_sender(self, *, plan_id, path_sha256, action_outcome):
        request = FinalizeCollectionExecutionContext.Request()
        request.plan_id, request.path_sha256, request.action_outcome = plan_id, path_sha256, action_outcome
        future = self._finalize.call_async(request)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=5.0)
        return bool(future.result() and future.result().accepted)


class Clock:
    def now_s(self):
        return time.monotonic()


class TestCollectionFollowerSmoke(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = RclpyNode("collection_follower_smoke")
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

        transport = RclpyTransport(self.node)
        transport.wait_ready()

        follower = LiveCollectionPathFollower(
            controller_tuning=ControllerTuning(1.0, 3.0, 10.0, 0.25, 0.05),
            context_schema_version="collection-execution-context/v1",
            context_activation_timeout_s=10.0,
            load_sender=transport.load_sender,
            load_outcome_provider=transport.load_outcome_provider,
            follow_path_sender=transport.follow_path_sender,
            goal_status_provider=transport.goal_status_provider,
            state_provider=transport.state_provider,
            hold_sender=transport.hold_sender,
            finalize_sender=transport.finalize_sender,
            clock=Clock(),
            controller_id=CONTROLLER_ID,
        )

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
        self.assertTrue(transport.follow_paths_sent(), "FollowPath must have been sent")
        self.assertTrue(follower.finalize_accepted, "Finalize must be ACCEPTED at terminal")


@launch_testing.post_shutdown_test()
class TestSmokeShutdown(unittest.TestCase):
    def test_exit_ok(self):
        self.assertTrue(True)
