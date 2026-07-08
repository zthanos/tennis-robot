"""ROS 2 integration test for the canonical simulated/physical OAK-D contract."""

from __future__ import annotations

import math
import time

import pytest

rclpy = pytest.importorskip("rclpy")

from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from tennis_robot.controller_node import (
    PERCEPTION_FRAME_ID,
    PERCEPTION_OBSERVATION_TIMEOUT_S,
    ControllerNode,
)
from tennis_robot_msgs.msg import BallDetection, BallDetectionArray


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
        assert observation.world_x_m == pytest.approx(0.5, abs=1e-3)
        assert observation.world_y_m == pytest.approx(4.535, abs=1e-3)
        assert len(controller._latest_camera_balls) == 2

        empty = BallDetectionArray()
        empty.header.frame_id = PERCEPTION_FRAME_ID
        previous_seq = controller._latest_obs_seq
        _publish_until_received(executor, publisher, empty, controller, previous_seq)
        assert controller._fresh_perception_observation().visible is False
        assert controller._latest_obs.source == "no_detection"
        assert controller._latest_camera_balls == []

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
