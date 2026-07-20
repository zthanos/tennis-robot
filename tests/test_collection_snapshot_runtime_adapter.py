import os
import sys
from dataclasses import FrozenInstanceError
from types import SimpleNamespace as N

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "ros2_ws", "src", "tennis_robot"))

from collection_route_fixtures import SCAN_POSE, default_configuration, default_court_half_boundary
from tennis_robot.collection_route_types import (
    AcceptedSpatialObservation,
    CollectionRouteConfiguration,
    DomainValidationError,
    SpatialObservationRejection,
)
from tennis_robot.collection_snapshot_runtime_adapter import (
    CollectionSnapshotRuntimeAdapter,
    CollectionSnapshotRuntimeSession,
)
from tennis_robot.perception_covariance_calibration import PerceptionSpatialValidationConfig
from tennis_robot.perception_spatial_observation_adapter import TimestampedCameraToMapTransform


class BuilderSpy:
    def __init__(self):
        self.items = []

    def add(self, item):
        self.items.append(item)


class TF:
    def __init__(self, fail=False, skew_s=0.0):
        self.fail = fail
        self.skew_s = skew_s
        self.timestamps = []

    def at(self, timestamp_s, frame_id):
        self.timestamps.append((timestamp_s, frame_id))
        if self.fail:
            raise RuntimeError("TF unavailable")
        return TimestampedCameraToMapTransform(
            timestamp_s + self.skew_s, "map", frame_id, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)
        )


def frame(*, healthy=True, spatial=True, calibration_id="gazebo-range-depth-quality-diagonal-v1-20260719-v2", configuration_id="gazebo-v2"):
    stamp = N(sec=1, nanosec=0)
    detection = N(
        has_spatial=spatial,
        matched_depth_stamp=stamp,
        position_covariance=[0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1],
        position_x=1.0,
        position_y=2.0,
        position_z=3.0,
        confidence=0.9,
    )
    return N(
        header=N(stamp=stamp, frame_id="camera_link_optical_frame"),
        spatial_targets_healthy=healthy,
        calibration_id=calibration_id,
        configuration_id=configuration_id,
        detections=[detection],
    )


def validation_config():
    return PerceptionSpatialValidationConfig(1.0, 1.0, 1e-9, 0.01, 1.0)


_UNSET = object()


def runtime_adapter(tf=None, localization_xy_covariance=_UNSET):
    profile = default_configuration().gazebo_snapshot
    return CollectionSnapshotRuntimeAdapter(
        tf_provider=tf or TF(),
        validation_config=validation_config(),
        localization_xy_covariance=(
            profile.localization_xy_covariance
            if localization_xy_covariance is _UNSET
            else localization_xy_covariance
        ),
    )


def test_explicit_provisional_gazebo_profile_is_nonzero_and_serialized():
    configuration = default_configuration()
    covariance = configuration.gazebo_snapshot.localization_xy_covariance.covariance
    assert covariance.xx == 0.01
    assert covariance.xy == 0.0
    assert covariance.yy == 0.01
    assert configuration.to_dict()["gazebo_snapshot"]["localization_xy_covariance"] == {
        "covariance": {"xx": 0.01, "xy": 0.0, "yy": 0.01}
    }
    with pytest.raises(FrozenInstanceError):
        configuration.gazebo_snapshot.localization_xy_covariance = None


def test_missing_gazebo_profile_is_rejected_without_default():
    encoded = default_configuration().to_dict()
    del encoded["gazebo_snapshot"]
    with pytest.raises(DomainValidationError):
        CollectionRouteConfiguration.from_dict(encoded)


def test_valid_frame_forwards_accepted_observation_with_supplied_scan_step():
    builder = BuilderSpy()
    tf = TF()
    runtime_adapter(tf).forward(
        scan_id="scan", frame=frame(), scan_step_id="step-01", builder=builder
    )
    assert isinstance(builder.items[0], AcceptedSpatialObservation)
    assert builder.items[0].scan_step_id == "step-01"
    assert builder.items[0].calibration_id == "gazebo-range-depth-quality-diagonal-v1-20260719-v2"
    assert builder.items[0].configuration_id == "gazebo-v2"
    assert tf.timestamps == [(1.0, "camera_link_optical_frame")]


def test_rejections_and_missing_configuration_forward_only_rejection():
    cases = [
        (runtime_adapter(), frame(healthy=False), "step-01"),
        (runtime_adapter(), frame(spatial=False), "step-01"),
        (runtime_adapter(TF(fail=True)), frame(), "step-01"),
        (runtime_adapter(localization_xy_covariance=None), frame(), "step-01"),
        (runtime_adapter(), frame(), None),
    ]
    for adapter, detection_frame, scan_step_id in cases:
        builder = BuilderSpy()
        adapter.forward(
            scan_id="scan", frame=detection_frame, scan_step_id=scan_step_id, builder=builder
        )
        assert isinstance(builder.items[0], SpatialObservationRejection)


def test_stale_tf_transform_beyond_max_detection_to_tf_age_is_rejected():
    builder = BuilderSpy()
    runtime_adapter(TF(skew_s=2.0)).forward(
        scan_id="scan", frame=frame(), scan_step_id="step-01", builder=builder
    )
    rejection = builder.items[0]
    assert isinstance(rejection, SpatialObservationRejection)
    assert rejection.detail == "detection_to_tf_age_exceeded"


def test_missing_or_empty_calibration_identity_is_a_typed_rejection():
    for detection_frame in (
        frame(calibration_id=""),
        frame(configuration_id=""),
    ):
        builder = BuilderSpy()
        runtime_adapter().forward(
            scan_id="scan", frame=detection_frame, scan_step_id="step-01", builder=builder
        )
        rejection = builder.items[0]
        assert isinstance(rejection, SpatialObservationRejection)
        assert rejection.detail == "calibration_identity_missing"
    legacy = frame()
    del legacy.calibration_id
    del legacy.configuration_id
    builder = BuilderSpy()
    runtime_adapter().forward(scan_id="scan", frame=legacy, scan_step_id="step-01", builder=builder)
    assert builder.items[0].detail == "calibration_identity_missing"


def test_session_connects_runtime_adapter_to_builder_without_scan_policy():
    session = CollectionSnapshotRuntimeSession(
        scan_id="scan",
        scan_timestamp_s=1.0,
        robot_pose_at_scan=SCAN_POSE,
        configuration_snapshot=default_configuration(),
        expected_scan_step_ids=("step-01", "step-02"),
        court_half_boundary=default_court_half_boundary(),
        tf_provider=TF(),
    )
    session.forward_frame(frame(), scan_step_id="step-01")
    session.forward_frame(frame(), scan_step_id="step-02")
    snapshot = session.finalize(now_s=2.0)
    assert [ball.ball_id for ball in snapshot.balls] == ["scan/target-1"]
    assert snapshot.configuration_snapshot.gazebo_snapshot.configuration_id == (
        "gazebo-mvp-provisional-planning-safety-v1"
    )
    assert session.builder.required_coverage_fraction == (
        default_configuration().scan.required_coverage_fraction
    )
    assert session.adapter.validation_config == (
        default_configuration().perception_spatial_validation
    )
