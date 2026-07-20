"""ROS 2 integration test for the canonical simulated/physical OAK-D contract."""

from __future__ import annotations

import math
import time

import pytest

rclpy = pytest.importorskip("rclpy")

from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tennis_robot.controller_node import (
    PERCEPTION_FRAME_ID,
    PERCEPTION_OBSERVATION_TIMEOUT_S,
    ControllerNode,
)
from tennis_robot_msgs.msg import BallDetection, BallDetectionArray
from tennis_robot.perception_covariance_calibration import PerceptionSpatialValidationConfig


def _spatial_detection(
    *,
    distance_m: float,
    bearing_rad: float,
    right_m: float,
    down_m: float,
    forward_m: float,
    confidence: float,
) -> BallDetection:
    detection = BallDetection()
    detection.has_spatial = True
    detection.distance_m = distance_m
    detection.bearing_rad = bearing_rad
    detection.position_x = right_m
    detection.position_y = down_m
    detection.position_z = forward_m
    detection.confidence = confidence
    detection.matched_depth_stamp.sec = 10
    detection.position_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.02]
    return detection


def _publish_until_received(
    executor: SingleThreadedExecutor,
    publisher,
    message: BallDetectionArray,
    controller: ControllerNode,
    previous_seq: int,
) -> None:
    deadline = time.monotonic() + 3.0
    next_publish = 0.0
    while time.monotonic() < deadline:
        now = time.monotonic()
        if now >= next_publish:
            publisher.publish(message)
            next_publish = now + 0.1
        executor.spin_once(timeout_sec=0.05)
        if controller._latest_obs_seq > previous_seq:
            return
    raise AssertionError("controller did not receive synthetic BallDetectionArray")


def test_controller_consumes_canonical_detection_array_end_to_end():
    rclpy.init()
    controller = ControllerNode()
    controller._perception_spatial_validation_config = PerceptionSpatialValidationConfig(1.0, 1.0, 1e-9, 0.001, 1.0)
    publisher_node = Node("synthetic_oakd_contract_test")
    publisher = publisher_node.create_publisher(
        BallDetectionArray, "/perception/ball_detections", 10
    )
    executor = SingleThreadedExecutor()
    executor.add_node(controller)
    executor.add_node(publisher_node)

    try:
        # Robot at (1, 2), facing +Y. A ball 2 m forward and 0.5 m left of
        # the camera should project to approximately (0.5, 4.535) in map.
        controller._robot_x = 1.0
        controller._robot_y = 2.0
        controller._robot_yaw = math.pi / 2

        message = BallDetectionArray()
        message.header.frame_id = PERCEPTION_FRAME_ID
        message.header.stamp.sec = 10
        message.spatial_targets_healthy = True
        message.spatial_targets_health_reason = "healthy"
        transform = TransformStamped()
        transform.transform.rotation.w = 1.0
        controller._tf_buffer.lookup_transform = lambda *args: transform
        message.detections = [
            _spatial_detection(
                distance_m=3.0,
                bearing_rad=-0.1,
                right_m=0.3,
                down_m=0.0,
                forward_m=2.9,
                confidence=0.7,
            ),
            _spatial_detection(
                distance_m=math.hypot(2.0, 0.5),
                bearing_rad=math.atan2(0.5, 2.0),
                right_m=-0.5,
                down_m=0.0,
                forward_m=2.0,
                confidence=0.9,
            ),
        ]
        previous_seq = controller._latest_obs_seq
        _publish_until_received(
            executor, publisher, message, controller, previous_seq
        )

        observation = controller._fresh_perception_observation()
        assert observation.visible is True
        assert observation.confidence == pytest.approx(0.9)
        assert observation.bearing_rad > 0.0
        # Identity camera_optical->map TF at the RGB header timestamp means
        # map coordinates are the published optical XYZ, not current robot pose.
        assert observation.world_x_m == pytest.approx(-0.5, abs=1e-3)
        assert observation.world_y_m == pytest.approx(0.0, abs=1e-3)
        assert len(controller._latest_camera_balls) == 2

        empty = BallDetectionArray()
        empty.header.frame_id = PERCEPTION_FRAME_ID
        empty.spatial_targets_healthy = True
        empty.spatial_targets_health_reason = "healthy"
        previous_seq = controller._latest_obs_seq
        _publish_until_received(executor, publisher, empty, controller, previous_seq)
        assert controller._fresh_perception_observation().visible is False
        assert controller._latest_obs.source == "no_detection"
        assert controller._latest_camera_balls == []

        unhealthy = BallDetectionArray()
        unhealthy.header.frame_id = PERCEPTION_FRAME_ID
        unhealthy.spatial_targets_healthy = False
        unhealthy.spatial_targets_health_reason = "calibration_missing"
        previous_seq = controller._latest_obs_seq
        _publish_until_received(executor, publisher, unhealthy, controller, previous_seq)
        assert controller._latest_obs.source == "spatial_targets_unhealthy"

        stale_at = (
            controller._runtime_seconds() - PERCEPTION_OBSERVATION_TIMEOUT_S - 0.1
        )
        controller._latest_obs_received_at = stale_at
        controller._latest_camera_balls_received_at = stale_at
        controller._latest_camera_balls = [{"stale": True}]
        expired = controller._fresh_perception_observation()
        assert expired.visible is False
        assert expired.source == "observation_timeout"
        assert controller._latest_camera_balls == []
    finally:
        executor.remove_node(publisher_node)
        executor.remove_node(controller)
        publisher_node.destroy_node()
        controller.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize("reason, mutate", [
    ("matched_depth_stamp_invalid", lambda d: setattr(d.matched_depth_stamp, "sec", 0)),
    ("rgb_depth_delta_exceeded", lambda d: setattr(d.matched_depth_stamp, "sec", 20)),
    ("covariance_nonfinite", lambda d: setattr(d, "position_covariance", [float("nan")] * 9)),
    ("covariance_nonsymmetric", lambda d: setattr(d, "position_covariance", [0.01, 0.1, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.02])),
    ("covariance_not_psd", lambda d: setattr(d, "position_covariance", [0.01, 0.1, 0.0, 0.1, 0.01, 0.0, 0.0, 0.0, 0.02])),
    ("covariance_trace_out_of_range", lambda d: setattr(d, "position_covariance", [2.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 2.0])),
])
def test_metadata_rejection_precedes_tf_lookup(reason, mutate):
    rclpy.init(); controller = ControllerNode()
    controller._perception_spatial_validation_config = PerceptionSpatialValidationConfig(1.0, 1.0, 1e-9, 0.001, 1.0)
    calls = []; controller._tf_buffer.lookup_transform = lambda *args: calls.append(args)
    try:
        message = BallDetectionArray(); message.header.frame_id = PERCEPTION_FRAME_ID; message.header.stamp.sec = 10; message.spatial_targets_healthy = True; message.spatial_targets_health_reason = "healthy"
        detection = _spatial_detection(distance_m=1., bearing_rad=0., right_m=0., down_m=0., forward_m=1., confidence=1.)
        mutate(detection); message.detections = [detection]; controller._on_ball_detections(message)
        assert controller._latest_obs.source == "perception_metadata_rejected"
        assert controller._last_perception_rejection_reason == reason
        assert not controller._latest_obs.visible and controller._latest_camera_balls == [] and not calls
    finally:
        controller.destroy_node(); rclpy.shutdown()


def test_tf_failure_has_no_current_pose_fallback():
    rclpy.init(); controller = ControllerNode(); controller._perception_spatial_validation_config = PerceptionSpatialValidationConfig(1.0, 1.0, 1e-9, 0.001, 1.0)
    controller._robot_x = 99.0; controller._tf_buffer.lookup_transform = lambda *args: (_ for _ in ()).throw(RuntimeError("missing tf"))
    try:
        message = BallDetectionArray(); message.header.frame_id = PERCEPTION_FRAME_ID; message.header.stamp.sec = 10; message.spatial_targets_healthy = True; message.spatial_targets_health_reason = "healthy"; message.detections = [_spatial_detection(distance_m=1., bearing_rad=0., right_m=0., down_m=0., forward_m=1., confidence=1.)]
        controller._on_ball_detections(message)
        assert controller._latest_obs.source == "perception_tf_rejected" and not controller._latest_obs.visible
    finally:
        controller.destroy_node(); rclpy.shutdown()
