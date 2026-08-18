"""Contract tests for freezing a map-frame route into odom."""

from __future__ import annotations

import math
import os
import sys

sys.path.insert(
    0, os.path.join(os.path.dirname(__file__), "..", "ros2_ws", "src", "tennis_robot")
)

from collection_route_fixtures import snapshot_for  # noqa: E402
from tennis_robot.collection_execution_frame import (  # noqa: E402
    RigidTransform2D,
    transform_collection_plan,
)
from tennis_robot.collection_route_planner_v2 import (  # noqa: E402
    CourtModel,
    plan_collection_route,
)
from tennis_robot.collection_route_types import Point2D  # noqa: E402


def default_plan():
    snapshot = snapshot_for(("ball", 6.0, 1.0), scan_id="execution-frame")
    court = CourtModel(
        (
            Point2D(-20.0, -20.0),
            Point2D(20.0, -20.0),
            Point2D(20.0, 20.0),
            Point2D(-20.0, 20.0),
        ),
        (),
    )
    return plan_collection_route(
        snapshot=snapshot,
        court=court,
        configuration=snapshot.configuration_snapshot,
    ).plan


def test_rigid_execution_transform_moves_every_plan_geometry_consistently():
    plan = default_plan()
    transformed = transform_collection_plan(
        plan,
        RigidTransform2D("odom", "map", 2.0, -3.0, math.pi / 2.0),
    )

    assert transformed.map_frame == "odom"
    assert transformed.plan_id == plan.plan_id
    assert transformed.total_length_m == plan.total_length_m
    assert math.isclose(
        transformed.start_pose.x_m, 2.0 - plan.start_pose.y_m
    )
    assert math.isclose(
        transformed.start_pose.y_m, -3.0 + plan.start_pose.x_m
    )

    original = plan.segments[0].path.points[0].pose
    moved = transformed.segments[0].path.points[0].pose
    assert math.isclose(moved.x_m, 2.0 - original.y_m)
    assert math.isclose(moved.y_m, -3.0 + original.x_m)
    assert math.isclose(
        math.remainder(moved.yaw_rad - original.yaw_rad, 2.0 * math.pi),
        math.pi / 2.0,
    )

    original_crossing = next(
        crossing for segment in plan.segments for crossing in segment.planned_crossings
    )
    moved_crossing = next(
        crossing
        for segment in transformed.segments
        for crossing in segment.planned_crossings
    )
    assert math.isclose(
        moved_crossing.position_xy.x_m, 2.0 - original_crossing.position_xy.y_m
    )
    assert math.isclose(
        moved_crossing.position_xy.y_m, -3.0 + original_crossing.position_xy.x_m
    )
    assert moved_crossing.progress_s == original_crossing.progress_s


def test_execution_transform_rejects_wrong_source_frame():
    plan = default_plan()
    try:
        transform_collection_plan(
            plan, RigidTransform2D("odom", "court", 0.0, 0.0, 0.0)
        )
    except ValueError as exc:
        assert "does not match" in str(exc)
    else:
        raise AssertionError("wrong source frame must fail")
