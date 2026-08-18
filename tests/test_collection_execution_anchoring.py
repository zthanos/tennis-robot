"""The executed route must stay anchored to the frame the balls are in.

Phase 12.  The old execution path froze the whole route into ``odom`` once, at
route start.  The tracker then followed that copy accurately while ``map→odom``
kept changing, so the physical corridor slid away from the map-anchored balls by
the accumulated correction -- 0.13 to 0.44 m measured live, against a 0.205 m
funnel half-width (debug log #72).

Every test here is written against that failure: with a static correction the
two architectures are equivalent, and with a changing one only the map-anchored
route keeps its corridor on the ball.
"""

from __future__ import annotations

import math
import os
import sys

import pytest

sys.path.insert(
    0, os.path.join(os.path.dirname(__file__), "..", "ros2_ws", "src", "tennis_robot")
)

from collection_route_fixtures import snapshot_for  # noqa: E402
from tennis_robot.collection_execution_frame import (  # noqa: E402
    RigidTransform2D,
    transform_collection_plan,
)
from tennis_robot.collection_executor_node_factory import (  # noqa: E402
    _ExecutionFrameContract,
)
from tennis_robot.collection_route_planner_v2 import CourtModel, plan_collection_route  # noqa: E402
from tennis_robot.collection_route_types import Point2D  # noqa: E402

FUNNEL_HALF_WIDTH_M = 0.205


def plan_with_ball(x=6.0, y=1.0):
    snapshot = snapshot_for(("ball", x, y), scan_id="anchoring")
    court = CourtModel(
        (Point2D(-20.0, -20.0), Point2D(20.0, -20.0), Point2D(20.0, 20.0), Point2D(-20.0, 20.0)),
        (),
    )
    return plan_collection_route(
        snapshot=snapshot, court=court, configuration=snapshot.configuration_snapshot
    ).plan


class FakeTransform:
    def __init__(self, x, y, yaw, stamp_s=12.5):
        self.transform = type("T", (), {})()
        self.transform.translation = type("V", (), {"x": x, "y": y, "z": 0.0})()
        half = yaw / 2.0
        self.transform.rotation = type(
            "Q", (), {"x": 0.0, "y": 0.0, "z": math.sin(half), "w": math.cos(half)}
        )()
        self.header = type("H", (), {})()
        self.header.stamp = type("S", (), {"sec": int(stamp_s), "nanosec": 0})()


class FakeBuffer:
    """Enough of a TF buffer to answer one lookup, or to fail like a real one."""

    def __init__(self, transform=None, error=None):
        self._transform, self._error = transform, error
        self.lookups = []

    def lookup_transform(self, target, source, when):
        self.lookups.append((target, source, when))
        if self._error is not None:
            raise self._error
        return self._transform


class FakeRos:
    @staticmethod
    def time_from_seconds(value):
        return value


def corridor_point_nearest(plan, point):
    """Closest point of the route polyline to `point`, in the plan's own frame."""
    best = math.inf
    for segment in plan.segments:
        points = [item.pose for item in segment.path.points]
        for first, second in zip(points, points[1:]):
            dx, dy = second.x_m - first.x_m, second.y_m - first.y_m
            denominator = dx * dx + dy * dy
            ratio = 0.0 if denominator == 0 else max(0.0, min(
                1.0, ((point[0] - first.x_m) * dx + (point[1] - first.y_m) * dy) / denominator))
            best = min(best, math.hypot(
                point[0] - (first.x_m + ratio * dx), point[1] - (first.y_m + ratio * dy)))
    return best


def ball_position(plan):
    for segment in plan.segments:
        for crossing in segment.planned_crossings:
            return (crossing.position_xy.x_m, crossing.position_xy.y_m)
    raise AssertionError("plan has no planned crossing")


def crossing_normal(plan):
    """Unit vector across the pass at the crossing.

    Collection is decided by how far the ball sits from the corridor centreline
    sideways, so a drift is applied along this direction: that is the component
    the funnel mouth actually sees.
    """
    for segment in plan.segments:
        for crossing in segment.planned_crossings:
            return (-math.sin(crossing.heading_rad), math.cos(crossing.heading_rad))
    raise AssertionError("plan has no planned crossing")


# ── the route is left in the frame it was planned in ────────────────────────

def test_execution_leaves_the_route_in_the_map_frame():
    plan = plan_with_ball()
    contract = _ExecutionFrameContract(FakeBuffer(FakeTransform(0.3, -0.2, 0.05)), FakeRos())
    executed = contract(plan)
    assert executed.map_frame == "map"
    assert executed is plan, "the plan must be executed as planned, not rebuilt"


def test_route_semantics_are_untouched_by_execution():
    plan = plan_with_ball()
    executed = _ExecutionFrameContract(FakeBuffer(FakeTransform(0.3, -0.2, 0.05)), FakeRos())(plan)
    assert executed.plan_id == plan.plan_id
    assert [segment.id for segment in executed.segments] == [s.id for s in plan.segments]
    assert [segment.type for segment in executed.segments] == [s.type for s in plan.segments]
    assert [
        (segment.progress_start_m, segment.progress_end_m) for segment in executed.segments
    ] == [(s.progress_start_m, s.progress_end_m) for s in plan.segments]
    assert [
        [(crossing.ball_id, crossing.progress_s) for crossing in segment.planned_crossings]
        for segment in executed.segments
    ] == [
        [(crossing.ball_id, crossing.progress_s) for crossing in segment.planned_crossings]
        for segment in plan.segments
    ]
    assert [
        [(point.pose.x_m, point.pose.y_m) for point in segment.path.points]
        for segment in executed.segments
    ] == [
        [(point.pose.x_m, point.pose.y_m) for point in segment.path.points]
        for segment in plan.segments
    ]


def test_the_correction_that_is_no_longer_applied_is_still_reported():
    contract = _ExecutionFrameContract(FakeBuffer(FakeTransform(0.31, -0.19, 0.02)), FakeRos())
    contract(plan_with_ball())
    diagnostics = contract.last_diagnostics
    assert diagnostics["execution_frame_policy"] == "map_anchored"
    assert diagnostics["identity"] is True
    assert diagnostics["source_frame"] == diagnostics["target_frame"] == "map"
    observed = diagnostics["observed_map_to_odom"]
    assert observed["x_m"] == pytest.approx(0.31)
    assert observed["y_m"] == pytest.approx(-0.19)
    assert observed["yaw_rad"] == pytest.approx(0.02)


def test_a_missing_transform_does_not_stop_the_route():
    # The transform is observability now, not a dependency of execution.
    contract = _ExecutionFrameContract(FakeBuffer(error=RuntimeError("no odom yet")), FakeRos())
    executed = contract(plan_with_ball())
    assert executed.map_frame == "map"
    assert contract.last_diagnostics["observed_map_to_odom"] is None


# ── static versus changing map→odom ─────────────────────────────────────────

def test_with_a_static_correction_both_architectures_are_equivalent():
    """No localization correction during the run: nothing to choose between them."""
    plan = plan_with_ball()
    frozen = RigidTransform2D("odom", "map", 0.4, -0.25, 0.03)
    old_route = transform_collection_plan(plan, frozen)

    ball = ball_position(plan)
    # Old architecture: corridor lives in odom; express it back in map through
    # the same (unchanged) correction to see where it physically sits.
    back = RigidTransform2D("map", "odom", *_inverse(frozen))
    old_in_map = transform_collection_plan(old_route, back)

    new_error = corridor_point_nearest(plan, ball)
    old_error = corridor_point_nearest(old_in_map, ball)
    assert old_error == pytest.approx(new_error, abs=1e-9)
    assert new_error == pytest.approx(0.0, abs=1e-9)


@pytest.mark.parametrize("drift_m", [0.13, 0.30, 0.44])
def test_a_changing_correction_displaces_only_the_odom_frozen_corridor(drift_m):
    """The measured live failure, in a unit test.

    ``map→odom`` moves by `drift_m` after the route starts.  The odom-frozen
    corridor moves with it and leaves the ball; the map-anchored route does not
    move at all, because it is expressed in the frame the ball is in.
    """
    plan = plan_with_ball()
    ball = ball_position(plan)
    normal = crossing_normal(plan)
    frozen = RigidTransform2D("odom", "map", 0.03, 0.01, 0.0)
    old_route = transform_collection_plan(plan, frozen)

    # Localization corrects: map→odom moves by `drift_m` across the pass.
    corrected = RigidTransform2D(
        "odom", "map",
        frozen.x_m + drift_m * normal[0], frozen.y_m + drift_m * normal[1], 0.0,
    )
    back = RigidTransform2D("map", "odom", *_inverse(corrected))
    old_in_map = transform_collection_plan(old_route, back)

    old_error = corridor_point_nearest(old_in_map, ball)
    new_error = corridor_point_nearest(plan, ball)

    assert old_error == pytest.approx(drift_m, abs=1e-3), "the old corridor must drift"
    assert new_error == pytest.approx(0.0, abs=1e-9), "the map route must not move"
    if drift_m > FUNNEL_HALF_WIDTH_M:
        assert old_error > FUNNEL_HALF_WIDTH_M, "this drift put the ball outside the funnel"


def test_the_drift_that_missed_target_2_no_longer_moves_the_corridor():
    """The Phase 10 case: ~0.30 m of correction, ball missed in all three runs."""
    plan = plan_with_ball()
    ball = ball_position(plan)
    normal = crossing_normal(plan)
    frozen = RigidTransform2D("odom", "map", 0.0302, 0.0133, -0.0076)
    old_route = transform_collection_plan(plan, frozen)
    # The live values at the target-2 crossing: |map→odom| grew to 0.377 m.
    measured = 0.377
    corrected = RigidTransform2D(
        "odom", "map",
        frozen.x_m + measured * normal[0], frozen.y_m + measured * normal[1], frozen.yaw_rad,
    )
    back = RigidTransform2D("map", "odom", *_inverse(corrected))

    old_error = corridor_point_nearest(transform_collection_plan(old_route, back), ball)
    # 1 mm: the corridor is a polyline, so the nearest-point measure carries a
    # little discretisation error.  The criterion it is compared against is 205 mm.
    assert old_error == pytest.approx(measured, abs=1e-3)
    assert old_error > FUNNEL_HALF_WIDTH_M, "the old corridor left the funnel width"
    assert corridor_point_nearest(plan, ball) == pytest.approx(0.0, abs=1e-9)


def _inverse(transform):
    """(x, y, yaw) of the inverse of a planar rigid transform."""
    cosine, sine = math.cos(-transform.yaw_rad), math.sin(-transform.yaw_rad)
    return (
        -(cosine * transform.x_m - sine * transform.y_m),
        -(sine * transform.x_m + cosine * transform.y_m),
        -transform.yaw_rad,
    )
