"""Segment construction, and the declare-only rule that makes overlap legal."""

import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "ros2_ws", "src", "tennis_robot"))

from collection_route_fixtures import default_configuration  # noqa: E402
from tennis_robot.collection_route_plan_builder import (  # noqa: E402
    connector_segment,
    pass_segment,
    plan_id_for,
    terminal_segment,
)
from tennis_robot.collection_route_planner_v2 import FunnelPassCandidate  # noqa: E402
from tennis_robot.collection_route_connector_graph import (  # noqa: E402
    ConnectorEdge,
    ConnectorPath,
    ConnectorPrimitive,
)
from tennis_robot.collection_route_types import (  # noqa: E402
    PlannedCrossing,
    Point2D,
    Pose2D,
    RouteSegmentType,
)


def three_ball_pass():
    return FunnelPassCandidate(
        "line:a+b+c", ("a", "b", "c"), 0.0,
        Pose2D(0.0, 0.0, 0.0), Point2D(1.0, 0.0), Pose2D(3.3, 0.0, 0.0), 0.05,
        (Point2D(1.0, 0.0), Point2D(2.0, 0.02), Point2D(3.0, 0.0)),
    )


def straight_edge_over(*ball_ids):
    poses = tuple(Pose2D(float(index), 0.0, 0.0) for index in range(6))
    path = ConnectorPath("LSL", (ConnectorPrimitive("S", 5.0, 0.0),), poses, 5.0, 0.0)
    crossings = tuple(
        PlannedCrossing(ball_id, Point2D(1.0 + index, 0.0), 1.0 + index, 0.0, 0.0)
        for index, ball_id in enumerate(ball_ids)
    )
    return ConnectorEdge("e", "s", "t", path, 1.0, True, None, crossings)


def test_pass_declares_every_ball_when_nothing_is_collected_yet():
    segment = pass_segment("p", three_ball_pass(), 0.0, default_configuration())
    assert segment.covered_ball_ids == ("a", "b", "c")
    assert tuple(item.ball_id for item in segment.planned_crossings) == ("a", "b", "c")


def test_pass_over_an_already_collected_ball_declares_only_the_new_ones():
    # 'b' is in the basket already: the robot still drives the same line, the
    # plan simply does not claim it twice.
    segment = pass_segment(
        "p", three_ball_pass(), 0.0, default_configuration(), declare_only=frozenset({"a", "c"})
    )
    assert segment.covered_ball_ids == ("a", "c")
    assert tuple(item.ball_id for item in segment.planned_crossings) == ("a", "c")
    # Geometry is untouched: the segment still spans the whole pass.
    assert math.isclose(segment.progress_end_m - segment.progress_start_m, 3.3, abs_tol=1e-9)
    assert segment.path.points[0].pose.x_m == 0.0
    assert math.isclose(segment.path.points[-1].pose.x_m, 3.3, abs_tol=1e-9)


def test_declared_crossings_keep_strictly_increasing_progress_after_filtering():
    segment = pass_segment(
        "p", three_ball_pass(), 2.0, default_configuration(), declare_only=frozenset({"c"})
    )
    assert segment.covered_ball_ids == ("c",)
    crossing = segment.planned_crossings[0]
    assert segment.progress_start_m < crossing.progress_s < segment.progress_end_m


def test_connector_declares_only_new_balls_and_keeps_the_capture_gate():
    configuration = default_configuration()
    both = connector_segment("c", straight_edge_over("x", "y"), 0.0, configuration)
    assert both.covered_ball_ids == ("x", "y")
    assert both.execution_profile.max_heading_error_rad == (
        configuration.planning.default_execution_profile.max_heading_error_rad
    )
    one = connector_segment(
        "c", straight_edge_over("x", "y"), 0.0, configuration, declare_only=frozenset({"y"})
    )
    assert one.covered_ball_ids == ("y",)
    assert one.type is RouteSegmentType.CONNECTOR


def test_connector_that_declares_nothing_falls_back_to_the_transit_gate():
    configuration = default_configuration()
    segment = connector_segment(
        "c", straight_edge_over("x"), 0.0, configuration, declare_only=frozenset()
    )
    assert segment.covered_ball_ids == ()
    assert segment.execution_profile.max_heading_error_rad == (
        configuration.planning.connector_max_heading_error_rad
    )


def test_plan_id_separates_routes_that_differ_only_in_geometry():
    configuration = default_configuration()
    first = (
        pass_segment("p", three_ball_pass(), 0.0, configuration),
        terminal_segment("t", Pose2D(3.3, 0.0, 0.0), Pose2D(3.8, 0.0, 0.0), 3.3, 0.5, configuration),
    )
    shifted = FunnelPassCandidate(
        "line:a+b+c", ("a", "b", "c"), 0.0,
        Pose2D(0.0, 0.5, 0.0), Point2D(1.0, 0.5), Pose2D(3.3, 0.5, 0.0), 0.05,
        (Point2D(1.0, 0.5), Point2D(2.0, 0.52), Point2D(3.0, 0.5)),
    )
    second = (
        pass_segment("p", shifted, 0.0, configuration),
        terminal_segment("t", Pose2D(3.3, 0.5, 0.0), Pose2D(3.8, 0.5, 0.0), 3.3, 0.5, configuration),
    )
    assert plan_id_for(first, "scan") != plan_id_for(second, "scan")
    assert plan_id_for(first, "scan") == plan_id_for(first, "scan")
    assert plan_id_for(first, "scan") != plan_id_for(first, "other-scan")
