"""Deterministic execution scenarios for the Phase 9 evaluator.

These synthesise an *executed* trajectory from a real plan by driving the
planned geometry with a stated error model.  That is enough to prove the
evaluator classifies each outcome correctly and to make the classification
reproducible; it is explicitly **not** evidence about the robot, which only a
live run can provide.  Every scenario states the error it injects, so a test
asserting an outcome is asserting the evaluator, not the robot.
"""

from __future__ import annotations

import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "ros2_ws", "src", "tennis_robot"))

from collection_route_fixtures import FAKE_TIME_S, default_configuration  # noqa: E402
from tennis_robot.collection_capture_geometry import (  # noqa: E402
    PlaneProvenance,
    repo_base_footprint_capture_geometry,
)
from tennis_robot.collection_execution_trace import (  # noqa: E402
    TRACE_SCHEMA_VERSION,
    BallObservation,
    CrossingSample,
    ExecutionTraceRecorder,
    TrajectorySample,
)
from tennis_robot.collection_route_planner_v2 import (  # noqa: E402
    CourtModel,
    plan_collection_route,
)
from tennis_robot.collection_route_types import (  # noqa: E402
    Point2D,
    Pose2D,
    PositionCovariance2D,
    RouteSegmentType,
    ScanSnapshot,
    SnapshotBall,
)

# The mouth sits 0.876 m ahead of base_footprint, so a trajectory that puts the
# *mouth* over a ball has its origin 0.876 m short of it.  Scenarios are written
# in terms of the planned path and this offset is handled by the geometry.
MOUTH_OFFSET_M = 0.876


def capture_geometry():
    return repo_base_footprint_capture_geometry(
        required_pre_contact_straight_m=0.3,
        required_pre_contact_provenance=PlaneProvenance.CONFIGURED,
    )


def court(extent: float = 20.0):
    return CourtModel(
        tuple(Point2D(x, y) for x, y in
              ((-extent, -extent), (extent, -extent), (extent, extent), (-extent, extent))),
        (),
    )


def snapshot(configuration, *entries, pose=Pose2D(0.0, 0.0, 0.0), scan_id="scan-exec"):
    return ScanSnapshot(
        scan_id, FAKE_TIME_S, "map", pose,
        tuple(
            SnapshotBall(ball_id, Point2D(x, y), 0.95, PositionCovariance2D(1e-6, 0.0, 1e-6))
            for ball_id, x, y in entries
        ),
        configuration,
    )


def plan_for(configuration, *entries, pose=Pose2D(0.0, 0.0, 0.0)):
    scan = snapshot(configuration, *entries, pose=pose)
    return scan, plan_collection_route(
        snapshot=scan, court=court(), configuration=configuration
    ).plan


def densify(plan, *, spacing_m=0.05):
    """Walk the planned path, yielding (segment_id, pose, arc length so far)."""
    progress = 0.0
    for segment in plan.segments:
        points = [point.pose for point in segment.path.points]
        for first, second in zip(points, points[1:]):
            span = math.hypot(second.x_m - first.x_m, second.y_m - first.y_m)
            steps = max(1, int(span / spacing_m))
            for step in range(steps):
                ratio = step / steps
                yield (
                    segment.id,
                    Pose2D(
                        first.x_m + ratio * (second.x_m - first.x_m),
                        first.y_m + ratio * (second.y_m - first.y_m),
                        _slerp(first.yaw_rad, second.yaw_rad, ratio),
                    ),
                    progress + ratio * span,
                )
            progress += span


def _slerp(first: float, second: float, ratio: float) -> float:
    delta = math.atan2(math.sin(second - first), math.cos(second - first))
    return first + ratio * delta


def execute(
    plan,
    *,
    run_id="run-1",
    lateral_bias_m=0.0,
    speed_mps=0.35,
    segment_bias=None,
    stop_after_segment=None,
):
    """Drive the plan with a stated tracking error and record a trace.

    ``lateral_bias_m`` offsets the whole path perpendicular to travel;
    ``segment_bias`` offsets named segments only.  Nothing else is randomised,
    so a scenario's outcome is a property of the geometry and the stated error.
    """
    recorder = ExecutionTraceRecorder(
        run_id=run_id, plan_id=plan.plan_id, scan_id=plan.scan_id,
        minimum_spacing_m=0.05, minimum_interval_s=0.5, maximum_samples=20000,
    )
    biases = dict(segment_bias or {})
    moment = 0.0
    previous = None
    stopped = False
    for segment_id, pose, progress in densify(plan):
        if stopped:
            break
        if stop_after_segment is not None and segment_id == stop_after_segment:
            stopped = True
        bias = biases.get(segment_id, lateral_bias_m)
        offset = Pose2D(
            pose.x_m - bias * math.sin(pose.yaw_rad),
            pose.y_m + bias * math.cos(pose.yaw_rad),
            pose.yaw_rad,
        )
        if previous is not None:
            moment += math.hypot(
                offset.x_m - previous.x_m, offset.y_m - previous.y_m
            ) / speed_mps
        recorder.record_pose(
            TrajectorySample(
                moment, offset.x_m, offset.y_m, offset.yaw_rad, speed_mps, 0.0,
                segment_id, progress,
            )
        )
        previous = offset
    return recorder, moment


def add_planned_crossing_rows(recorder, plan, *, lateral_bias_m=0.0, speed_mps=0.35):
    """Mimic the controller's crossing telemetry for each planned crossing."""
    moment = 0.0
    for segment in plan.segments:
        for crossing in segment.planned_crossings:
            moment += 1.0
            recorder.record_crossing(
                CrossingSample(
                    moment, crossing.ball_id, segment.id, crossing.progress_s,
                    crossing.progress_s, lateral_bias_m + crossing.predicted_lateral_error,
                    0.0, speed_mps,
                )
            )


def confirm(recorder, *, t_s, ball_id=None, segment_id=None):
    """Record an entry/confirmed beam pair, as the intake would produce."""
    recorder.record_beam(
        t_s=t_s, beam="entry", level=True, segment_id=segment_id, active_ball_id=ball_id
    )
    recorder.record_beam(
        t_s=t_s + 0.4, beam="confirmed", level=True, segment_id=segment_id,
        active_ball_id=ball_id,
    )
    recorder.record_beam(
        t_s=t_s + 0.6, beam="entry", level=False, segment_id=segment_id,
        active_ball_id=ball_id,
    )
    recorder.record_beam(
        t_s=t_s + 0.9, beam="confirmed", level=False, segment_id=segment_id,
        active_ball_id=ball_id,
    )


def observe(recorder, *, t_s, ball_id, x_m, y_m):
    recorder.record_observation(BallObservation(t_s, ball_id, x_m, y_m))


def crossing_times(plan, trace):
    """When the mouth passed each planned crossing, for scripting beams."""
    times = {}
    for segment in plan.segments:
        for crossing in segment.planned_crossings:
            for sample in trace.samples:
                if sample.segment_id != segment.id or sample.progress_s is None:
                    continue
                if sample.progress_s >= crossing.progress_s:
                    times[crossing.ball_id] = sample.t_s
                    break
    return times


# ── the five deterministic scenarios ────────────────────────────────────────

def scenario_straight_sweep(configuration=None):
    """Three balls on one line: one pass, three crossings, three collections."""
    configuration = configuration or default_configuration(maximum_candidate_count=80)
    return plan_for(configuration, ("a", 4.0, 0.0), ("b", 5.0, 0.0), ("c", 6.0, 0.0))


def scenario_two_passes_with_connector(configuration=None):
    """Two groups far enough apart to need a connector between them."""
    configuration = configuration or default_configuration(maximum_candidate_count=80)
    return plan_for(
        configuration, ("a", 3.0, 2.0), ("b", 4.0, 2.0), ("c", 9.0, -2.0), ("d", 10.0, -2.0)
    )


def scenario_connector_collects(configuration=None):
    """A ball sitting on the natural transit line between two groups."""
    configuration = configuration or default_configuration(maximum_candidate_count=80)
    return plan_for(
        configuration, ("a", 3.0, 0.0), ("b", 4.0, 0.0), ("mid", 8.0, 0.0),
        ("c", 12.0, 0.0), ("d", 13.0, 0.0),
    )


def scenario_near_miss(configuration=None):
    """A later ball just outside an earlier corridor: the disturbance case."""
    configuration = configuration or default_configuration(maximum_candidate_count=80)
    return plan_for(
        configuration, ("a", 4.0, 0.0), ("b", 5.0, 0.0), ("bystander", 6.0, 0.55)
    )


SCENARIOS = {
    "straight_sweep": scenario_straight_sweep,
    "two_passes_with_connector": scenario_two_passes_with_connector,
    "connector_collects": scenario_connector_collects,
    "near_miss": scenario_near_miss,
}
