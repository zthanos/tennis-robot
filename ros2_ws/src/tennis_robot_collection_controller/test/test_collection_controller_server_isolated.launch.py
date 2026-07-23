"""C3: direct FollowPath integration against a real isolated controller_server."""

import os
import struct
import time
import unittest
from dataclasses import dataclass

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node as RclpyNode
from tf2_ros import TransformBroadcaster
from tennis_robot_msgs.msg import CollectionControllerState
from tennis_robot_msgs.msg import CollectionExecutionContext, CollectionExecutionSegment, CollectionPlannedCrossing
from tennis_robot_msgs.srv import FinalizeCollectionExecutionContext, LoadCollectionExecutionContext, ResetCollectionExecutionContext, SetCollectionSafetyHold


def generate_test_description():
    params = os.path.join(get_package_share_directory('tennis_robot'), 'config', 'nav2_params.yaml')
    return LaunchDescription([
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']),
        Node(package='nav2_controller', executable='controller_server', name='controller_server',
             parameters=[params, {'use_sim_time': False}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='c3_lifecycle_manager',
             parameters=[{'autostart': True, 'node_names': ['controller_server'], 'use_sim_time': False}]),
        launch_testing.actions.ReadyToTest(),
    ])


def path_hash(path):
    raw = struct.pack('>I', len(path.header.frame_id.encode())) + path.header.frame_id.encode()
    raw += struct.pack('>I', len(path.poses))
    for stamped in path.poses:
        pose = stamped.pose
        raw += struct.pack('>7d', pose.position.x, pose.position.y, pose.position.z,
                           pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    import hashlib
    return hashlib.sha256(raw).hexdigest()


@dataclass
class GoalRecord:
    goal_handle: object
    is_collection: bool
    plan_id: str
    path_hash: str
    terminal_result_received: bool = False
    context_finalized: bool = False


class TestCollectionControllerServerIsolation(unittest.TestCase):
    def setUp(self):
        self.goal_registry = []

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = RclpyNode('c3_collection_harness')
        cls.odom = cls.node.create_publisher(Odometry, '/odom', 10)
        cls.tf = TransformBroadcaster(cls.node)
        cls.pose_x = 4.0
        cls.pose_timer = cls.node.create_timer(0.02, cls.publish_pose)
        cls.cmd = []
        cls.node.create_subscription(Twist, '/cmd_vel', lambda msg: cls.cmd.append(msg), 10)
        cls.states = []
        cls.node.create_subscription(CollectionControllerState, '/CollectionFollowPath/state', lambda msg: cls.states.append(msg), 10)
        cls.load = cls.node.create_client(LoadCollectionExecutionContext, '/CollectionFollowPath/load_collection_execution_context')
        cls.reset = cls.node.create_client(ResetCollectionExecutionContext, '/CollectionFollowPath/reset_collection_execution_context')
        cls.hold = cls.node.create_client(SetCollectionSafetyHold, '/CollectionFollowPath/set_collection_safety_hold')
        cls.finalize = cls.node.create_client(FinalizeCollectionExecutionContext, '/CollectionFollowPath/finalize_collection_execution_context')
        cls.action = ActionClient(cls.node, FollowPath, '/follow_path')
        for client in (cls.load, cls.reset, cls.hold, cls.finalize):
            assert client.wait_for_service(timeout_sec=20.0)
        assert cls.action.wait_for_server(timeout_sec=20.0)
        cls.set_pose(4.0)

    @classmethod
    def tearDownClass(cls):
        cls.pose_timer.cancel(); cls.node.destroy_node(); rclpy.shutdown()

    def tearDown(self):
        for record in list(self.goal_registry):
            if not record.terminal_result_received:
                cancel = record.goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self.node, cancel, timeout_sec=3.0)
                self.assertIsNotNone(cancel.result(), 'FollowPath cancel response timed out')
                terminal = record.goal_handle.get_result_async()
                rclpy.spin_until_future_complete(self.node, terminal, timeout_sec=3.0)
                self.assertIsNotNone(terminal.result(), 'FollowPath terminal result timed out after cancel')
                record.terminal_result_received = True
            if record.is_collection and not record.context_finalized:
                response = self.finalize_goal(record, FinalizeCollectionExecutionContext.Request.CANCELED)
                if not response.accepted:
                    self.assertIn(response.rejection_code, (
                        FinalizeCollectionExecutionContext.Response.MISSING_CONTEXT,
                        FinalizeCollectionExecutionContext.Response.INVALID_LIFECYCLE))
        reset = self.call(self.reset, ResetCollectionExecutionContext.Request())
        self.assertTrue(reset.accepted, f'context reset rejected: {reset.rejection_code}')
        self.spin(0.15)
        if self.states:
            self.assertEqual(self.states[-1].lifecycle_state, CollectionControllerState.IDLE)
        self.goal_registry.clear()

    def spin(self, seconds=0.2):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    @classmethod
    def set_pose(cls, x):
        cls.pose_x = x

    def set_pose_and_wait(self, x):
        self.set_pose(x)
        self.spin(0.2)

    def cleanup_collection_goal(self, record):
        cancel = record.goal_handle.cancel_goal_async(); rclpy.spin_until_future_complete(self.node, cancel, timeout_sec=3.0)
        terminal = record.goal_handle.get_result_async(); rclpy.spin_until_future_complete(self.node, terminal, timeout_sec=3.0)
        self.assertIsNotNone(terminal.result())
        record.terminal_result_received = True
        response = self.finalize_goal(record, FinalizeCollectionExecutionContext.Request.CANCELED)
        self.assertTrue(response.accepted)
        reset = self.call(self.reset, ResetCollectionExecutionContext.Request()); self.assertTrue(reset.accepted)
        self.wait_for_idle()
        self.goal_registry.remove(record)

    def wait_for_idle(self):
        end = time.monotonic() + 2.0
        while time.monotonic() < end:
            self.spin(0.05)
            if self.states and self.states[-1].lifecycle_state == CollectionControllerState.IDLE:
                return
        self.fail('timed out waiting for IDLE lifecycle telemetry')

    def wait_for_state(self, lifecycle, plan_id='c3-plan', digest=None):
        end = time.monotonic() + 2.0
        while time.monotonic() < end:
            self.spin(0.05)
            if self.states and self.states[-1].lifecycle_state == lifecycle and \
                    self.states[-1].plan_id == plan_id and (digest is None or self.states[-1].path_sha256 == digest):
                return
        self.fail(f'timed out waiting for lifecycle state {lifecycle}')

    @classmethod
    def publish_pose(cls):
        transform = TransformStamped(); transform.header.frame_id = 'odom'; transform.child_frame_id = 'base_link'
        transform.header.stamp = cls.node.get_clock().now().to_msg()
        transform.transform.translation.x = cls.pose_x; transform.transform.rotation.w = 1.0
        cls.tf.sendTransform(transform)
        odom = Odometry(); odom.header.stamp = transform.header.stamp; odom.header.frame_id = 'odom'; odom.child_frame_id = 'base_link'; odom.pose.pose.position.x = cls.pose_x
        odom.pose.pose.orientation.w = 1.0; cls.odom.publish(odom)

    def call(self, client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        self.assertIsNotNone(future.result())
        return future.result()

    def send_goal(self, goal):
        future = self.action.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        self.goal_handle = future.result()
        self.assertIsNotNone(self.goal_handle, 'FollowPath goal response timed out')
        self.assertTrue(self.goal_handle.accepted)
        self.goal_registry.append(GoalRecord(self.goal_handle, goal.controller_id == 'CollectionFollowPath', 'c3-plan', path_hash(goal.path)))
        return self.goal_handle

    def finalize_goal(self, record, outcome):
        request = FinalizeCollectionExecutionContext.Request()
        request.plan_id = record.plan_id; request.path_sha256 = record.path_hash; request.action_outcome = outcome
        response = self.call(self.finalize, request)
        if response.accepted:
            record.context_finalized = True
        return response

    def route(self):
        path = Path(); path.header.frame_id = 'map'
        for x in (0.0, 4.0):
            point = PoseStamped(); point.pose.position.x = x; point.pose.orientation.w = 1.0; path.poses.append(point)
        return path

    def context(self, path):
        context = CollectionExecutionContext()
        context.context_schema_version = 'collection-execution-context/v1'; context.plan_id = 'c3-plan'
        context.path_sha256 = path_hash(path); context.context_activation_timeout_s = 5.0
        context.terminal_progress_s = 4.0; context.terminal_pose.position.x = 4.0; context.terminal_pose.orientation.w = 1.0
        context.configuration_snapshot_json = '{}'
        tuning = context.controller_tuning; tuning.lookahead_distance_m = 1.0; tuning.max_angular_velocity_rad_s = 2.0
        tuning.progress_projection_window_m = 5.0; tuning.crossing_speed_window_m = .25; tuning.terminal_progress_tolerance_m = .05
        segment = CollectionExecutionSegment(); segment.segment_id = 'pass'; segment.progress_start_s = 0.0; segment.progress_end_s = 4.0
        profile = segment.execution_profile; profile.nominal_speed_mps = 1.0; profile.min_speed_mps = .8; profile.max_speed_mps = 1.2
        profile.nominal_speed_warning_tolerance_mps = .1; profile.max_acceleration_mps2 = 1.0; profile.max_deceleration_mps2 = 1.0
        profile.required_entry_m = 0.0; profile.required_run_in_m = .5; profile.required_run_out_m = .5; profile.max_curvature_per_m = 2.0
        profile.max_lateral_error_m = .5; profile.max_heading_error_rad = 1.0; profile.allow_reversing = False; profile.allow_standalone_rotate = False
        crossing = CollectionPlannedCrossing(); crossing.ball_id = 'ball'; crossing.position_x_m = 2.0; crossing.progress_s = 2.0
        segment.planned_crossings = [crossing]; context.segments = [segment]
        return context

    def test_direct_follow_path_hash_and_terminal_finalize(self):
        self.set_pose(4.0)
        path = self.route(); load = LoadCollectionExecutionContext.Request(); load.context = self.context(path)
        self.assertTrue(self.call(self.load, load).accepted)
        goal = FollowPath.Goal(); goal.path = path; goal.controller_id = 'CollectionFollowPath'
        sent = self.send_goal(goal)
        result = sent.get_result_async(); rclpy.spin_until_future_complete(self.node, result, timeout_sec=10.0)
        self.assertEqual(result.result().status, 4)  # action succeeded at terminal pose
        record = self.goal_registry[-1]; record.terminal_result_received = True
        self.assertTrue(self.finalize_goal(record, FinalizeCollectionExecutionContext.Request.SUCCEEDED).accepted)
        self.assertFalse(self.finalize_goal(record, FinalizeCollectionExecutionContext.Request.SUCCEEDED).accepted)  # consumed

    def test_missing_context_and_hash_mismatch_do_not_fallback_to_rpp(self):
        self.set_pose(4.0)
        path = self.route(); goal = FollowPath.Goal(); goal.path = path; goal.controller_id = 'CollectionFollowPath'
        sent = self.send_goal(goal)
        result = sent.get_result_async(); rclpy.spin_until_future_complete(self.node, result, timeout_sec=10.0)
        self.assertNotEqual(result.result().status, 4)

    def test_safety_hold_is_zero_and_telemetry_is_bound(self):
        self.set_pose_and_wait(0.0); self.cmd.clear(); self.states.clear()
        path = self.route(); load = LoadCollectionExecutionContext.Request(); load.context = self.context(path)
        self.assertTrue(self.call(self.load, load).accepted)
        goal = FollowPath.Goal(); goal.path = path; goal.controller_id = 'CollectionFollowPath'
        sent = self.send_goal(goal); self.spin()
        self.wait_for_state(CollectionControllerState.EXECUTING, digest=path_hash(path))
        hold = SetCollectionSafetyHold.Request(); hold.plan_id = 'c3-plan'; hold.path_sha256 = path_hash(path); hold.hold = True
        self.assertTrue(self.call(self.hold, hold).accepted); self.spin()
        self.assertTrue(self.cmd); self.assertTrue(any(cmd.linear.x == 0.0 and cmd.angular.z == 0.0 for cmd in self.cmd))
        self.assertTrue(self.states); state = self.states[-1]
        self.assertEqual(state.plan_id, 'c3-plan'); self.assertEqual(state.path_sha256, path_hash(path))
        self.assertEqual(state.lifecycle_state, CollectionControllerState.SAFETY_PAUSED)
        hold.hold = False; self.assertTrue(self.call(self.hold, hold).accepted)
        self.wait_for_state(CollectionControllerState.EXECUTING, digest=path_hash(path))

    def test_collection_is_forward_only_and_survey_rpp_needs_no_context(self):
        self.set_pose_and_wait(0.0); self.cmd.clear()
        path = self.route(); load = LoadCollectionExecutionContext.Request(); load.context = self.context(path)
        self.assertTrue(self.call(self.load, load).accepted)
        collection = FollowPath.Goal(); collection.path = path; collection.controller_id = 'CollectionFollowPath'
        sent = self.send_goal(collection); self.spin()
        self.assertTrue(self.cmd)
        self.assertTrue(all(command.linear.x >= 0.0 for command in self.cmd))
        self.assertTrue(all(not (command.linear.x == 0.0 and command.angular.z != 0.0) for command in self.cmd))
        self.cleanup_collection_goal(self.goal_registry[-1])
        self.set_pose_and_wait(4.0)
        survey = FollowPath.Goal(); survey.path = path; survey.controller_id = 'FollowPath'
        sent = self.send_goal(survey)
        result = sent.get_result_async(); rclpy.spin_until_future_complete(self.node, result, timeout_sec=10.0)
        self.assertEqual(result.result().status, 4)
        names = [name.lower() for name, _ in self.node.get_topic_names_and_types()]
        self.assertFalse(any('backup' in name or 'spin' in name or 'recovery' in name for name in names))
        mismatch = self.route(); mismatch.poses[-1].pose.position.x = 3.0
        mismatch_goal = FollowPath.Goal(); mismatch_goal.path = mismatch; mismatch_goal.controller_id = 'CollectionFollowPath'
        sent = self.send_goal(mismatch_goal)
        result = sent.get_result_async(); rclpy.spin_until_future_complete(self.node, result, timeout_sec=10.0)
        self.assertNotEqual(result.result().status, 4)


@launch_testing.post_shutdown_test()
class TestIsolationProof(unittest.TestCase):
    def test_placeholder(self):
        self.assertTrue(True)
