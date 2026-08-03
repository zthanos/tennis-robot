"""Off-route discovery while driving: viewpoint stepping and snapshot assembly."""

import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "ros2_ws", "src", "tennis_robot"))

from tennis_robot.collection_drive_observation import (  # noqa: E402
    DriveObservationBuffer,
    DriveObservationError,
    DriveViewpointStepper,
    build_drive_snapshot,
)
from tennis_robot.collection_route_types import (  # noqa: E402
    AcceptedSpatialObservation,
    Point2D,
    Pose2D,
    PositionCovariance2D,
)
from tennis_robot.collection_scan_snapshot import SpatialObservationRejection  # noqa: E402
from tennis_robot.collection_scan_snapshot import SpatialObservationRejectionCode  # noqa: E402
from collection_route_fixtures import (  # noqa: E402
    SCAN_POSE,
    default_configuration,
    default_court_half_boundary,
)


def obs(step, x=1.0, y=2.0, c=0.9, scan_id="drive"):
    return AcceptedSpatialObservation(
        scan_id, 0, 1.0, 1.0, Point2D(x, y), PositionCovariance2D(0.1, 0.0, 0.1), c, step,
        "gazebo-v2", "cfg",
    )


def _build(buffer, **kw):
    kw.setdefault("configuration_snapshot", default_configuration())
    kw.setdefault("court_half_boundary", default_court_half_boundary())
    kw.setdefault("robot_pose", Pose2D(SCAN_POSE.x_m, SCAN_POSE.y_m, SCAN_POSE.yaw_rad))
    kw.setdefault("now_s", 10.0)
    return build_drive_snapshot(buffer=buffer, **kw)


# ── viewpoint stepping ───────────────────────────────────────────────────────

def test_viewpoint_changes_only_after_travelling_the_spacing():
    stepper = DriveViewpointStepper(viewpoint_spacing_m=1.0)
    assert stepper.observe_pose(0.0, 0.0) == "drive-vp-0"
    assert stepper.observe_pose(0.5, 0.0) == "drive-vp-0"  # same place
    assert stepper.observe_pose(1.0, 0.0) == "drive-vp-1"  # travelled the spacing
    assert stepper.observe_pose(1.4, 0.0) == "drive-vp-1"
    assert stepper.observe_pose(2.1, 0.0) == "drive-vp-2"
    assert stepper.visited_ids == ("drive-vp-0", "drive-vp-1", "drive-vp-2")


def test_viewpoint_spacing_must_be_positive_and_finite():
    with pytest.raises(DriveObservationError):
        DriveViewpointStepper(viewpoint_spacing_m=0.0)
    with pytest.raises(DriveObservationError):
        DriveViewpointStepper(viewpoint_spacing_m=float("nan"))
    stepper = DriveViewpointStepper(viewpoint_spacing_m=1.0)
    with pytest.raises(DriveObservationError):
        stepper.observe_pose(float("inf"), 0.0)


def test_reset_starts_a_fresh_run():
    stepper = DriveViewpointStepper(viewpoint_spacing_m=1.0)
    stepper.observe_pose(0.0, 0.0)
    stepper.observe_pose(5.0, 0.0)
    stepper.reset()
    assert stepper.visited_ids == ()
    assert stepper.observe_pose(9.0, 9.0) == "drive-vp-0"


# ── buffer ───────────────────────────────────────────────────────────────────

def test_buffer_separates_accepted_from_rejected():
    buffer = DriveObservationBuffer(scan_id="drive")
    buffer.add(obs("drive-vp-0"))
    buffer.add(
        SpatialObservationRejection(
            SpatialObservationRejectionCode.PERCEPTION_METADATA_REJECTED, 0, 1.0, "stale"
        )
    )
    assert buffer.observation_count == 1
    assert len(buffer.rejections) == 1


def test_buffer_requires_a_scan_id():
    with pytest.raises(DriveObservationError):
        DriveObservationBuffer(scan_id="")


# ── snapshot assembly ────────────────────────────────────────────────────────

def test_no_observations_yields_no_snapshot():
    assert _build(DriveObservationBuffer(scan_id="drive")) is None


def test_ball_seen_from_one_viewpoint_only_is_not_trusted():
    """The 360's two-distinct-steps rule still applies to viewpoints."""
    buffer = DriveObservationBuffer(scan_id="drive")
    buffer.add(obs("drive-vp-0", x=1.0, y=2.0))
    buffer.add(obs("drive-vp-0", x=1.0, y=2.0))  # same place twice is not two viewpoints
    assert _build(buffer) is None


def test_ball_seen_from_two_viewpoints_becomes_a_target():
    buffer = DriveObservationBuffer(scan_id="drive")
    buffer.add(obs("drive-vp-0", x=1.0, y=2.0))
    buffer.add(obs("drive-vp-1", x=1.02, y=2.01))
    snapshot = _build(buffer)
    assert snapshot is not None
    assert len(snapshot.balls) == 1
    assert snapshot.balls[0].ball_id == "drive/target-1"


def test_coverage_gate_is_an_identity_for_a_drive():
    """Viewpoints that saw nothing must not read as an uncovered sweep sector.

    The 360 requires a fraction of its headings to be covered; a drive has no
    such notion, and the accumulation would otherwise fail whenever the robot
    passed through empty ground.
    """
    buffer = DriveObservationBuffer(scan_id="drive")
    # Two contributing viewpoints out of a nominally long drive.
    buffer.add(obs("drive-vp-3", x=1.0, y=2.0))
    buffer.add(obs("drive-vp-9", x=1.02, y=2.01))
    assert default_configuration().scan.required_coverage_fraction > 0.0
    snapshot = _build(buffer)
    assert snapshot is not None and len(snapshot.balls) == 1


def test_balls_the_finished_route_already_knew_are_dropped():
    buffer = DriveObservationBuffer(scan_id="drive")
    buffer.add(obs("drive-vp-0", x=1.0, y=2.0))
    buffer.add(obs("drive-vp-1", x=1.02, y=2.01))
    assert _build(buffer, known_positions=((1.0, 2.0),), merge_radius_m=0.5) is None
    # Far enough away, it is a genuinely new target.
    assert _build(buffer, known_positions=((5.0, 5.0),), merge_radius_m=0.5) is not None


def test_target_ids_are_renumbered_contiguously_after_dedup():
    buffer = DriveObservationBuffer(scan_id="drive")
    for x in (1.0, 3.0):
        buffer.add(obs("drive-vp-0", x=x, y=2.0))
        buffer.add(obs("drive-vp-1", x=x + 0.02, y=2.01))
    snapshot = _build(buffer, known_positions=((1.0, 2.0),), merge_radius_m=0.5)
    assert snapshot is not None
    assert [ball.ball_id for ball in snapshot.balls] == ["drive/target-1"]


def test_invalid_inputs_fail_loud():
    buffer = DriveObservationBuffer(scan_id="drive")
    buffer.add(obs("drive-vp-0"))
    with pytest.raises(DriveObservationError):
        _build(buffer, merge_radius_m=-1.0)
    with pytest.raises(DriveObservationError):
        _build(buffer, now_s=float("nan"))
    with pytest.raises(DriveObservationError):
        _build(buffer, robot_pose=None)


# ── contract with the real runtime adapter ───────────────────────────────────
# The buffer stands in for a ScanSnapshotBuilder. Exercising it only through
# direct .add() calls missed that the adapter also calls .record_visited_step(),
# which killed controller_node on the first execution tick of a live route.

from types import SimpleNamespace as N  # noqa: E402

from tennis_robot.collection_snapshot_runtime_adapter import (  # noqa: E402
    CollectionSnapshotRuntimeAdapter,
)
from tennis_robot.perception_covariance_calibration import (  # noqa: E402
    PerceptionSpatialValidationConfig,
)
from tennis_robot.perception_spatial_observation_adapter import (  # noqa: E402
    TimestampedCameraToMapTransform,
)


class _TF:
    def at(self, timestamp_s, frame_id):
        return TimestampedCameraToMapTransform(
            timestamp_s, "map", frame_id, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)
        )


def _live_frame():
    stamp = N(sec=1, nanosec=0)
    detection = N(
        has_spatial=True,
        matched_depth_stamp=stamp,
        position_covariance=[0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1],
        position_x=1.0,
        position_y=2.0,
        position_z=3.0,
        confidence=0.9,
    )
    return N(
        header=N(stamp=stamp, frame_id="camera_link_optical_frame"),
        spatial_targets_healthy=True,
        calibration_id="gazebo-range-depth-quality-diagonal-v1-20260719-v2",
        configuration_id="gazebo-v2",
        detections=[detection],
    )


def test_buffer_satisfies_everything_the_runtime_adapter_calls():
    adapter = CollectionSnapshotRuntimeAdapter(
        tf_provider=_TF(),
        validation_config=PerceptionSpatialValidationConfig(1.0, 1.0, 1e-9, 0.01, 1.0),
        localization_xy_covariance=default_configuration().gazebo_snapshot.localization_xy_covariance,
    )
    buffer = DriveObservationBuffer(scan_id="drive")
    for step in ("drive-vp-0", "drive-vp-1"):
        adapter.forward(
            scan_id=buffer.scan_id, frame=_live_frame(), scan_step_id=step, builder=buffer
        )
    assert buffer.visited_steps == {"drive-vp-0", "drive-vp-1"}
    assert buffer.observation_count == 2


def test_coverage_heartbeat_alone_never_produces_a_target():
    """A visited step with no accepted observation must not widen coverage."""
    buffer = DriveObservationBuffer(scan_id="drive")
    for step in ("drive-vp-0", "drive-vp-1", "drive-vp-2"):
        buffer.record_visited_step(step)
    assert buffer.visited_steps
    assert _build(buffer) is None
