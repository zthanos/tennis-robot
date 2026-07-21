"""Pure Phase 3B1 CSC connector graph tests."""

from dataclasses import FrozenInstanceError, replace
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "ros2_ws", "src", "tennis_robot"))

from collection_route_fixtures import FAKE_TIME_S, default_configuration
import math

from tennis_robot.collection_route_connector_graph import (
    ConnectorRejectionCode,
    _dubins_parameters,
    _materialize_path,
    _self_intersects,
    build_directed_candidate_graph,
)
from tennis_robot.collection_route_planner_v2 import CourtModel, FunnelPassCandidate, PolygonObstacle
from tennis_robot.collection_route_types import Point2D, Pose2D, ScanSnapshot


def polygon(*coordinates):
    return tuple(Point2D(float(x), float(y)) for x, y in coordinates)


def candidate(ball_id, entry, exit):
    return FunnelPassCandidate(
        ball_id,
        (ball_id,),
        entry.yaw_rad,
        entry,
        Point2D(entry.x_m, entry.y_m),
        exit,
        0.05,
        (Point2D(entry.x_m, entry.y_m),),
    )


def snapshot(configuration, start=Pose2D(0.0, 0.0, 0.0)):
    return ScanSnapshot("scan-b1", FAKE_TIME_S, "map", start, (), configuration)


def court(*obstacles):
    return CourtModel(polygon((-20, -20), (20, -20), (20, 20), (-20, 20)), tuple(obstacles))


def graph(configuration, candidates, model=None, start=Pose2D(0.0, 0.0, 0.0)):
    return build_directed_candidate_graph(
        snapshot=snapshot(configuration, start),
        candidates=tuple(candidates),
        court=model or court(),
        configuration=configuration,
    )


def _chord_sum(poses):
    return sum(
        math.hypot(poses[index + 1].x_m - poses[index].x_m, poses[index + 1].y_m - poses[index].y_m)
        for index in range(len(poses) - 1)
    )


# Start/target pairs chosen so each CSC family resolves with real (non-zero) arcs.
_CURVED_CASES = {
    "LSL": (Pose2D(0.0, 0.0, 0.0), Pose2D(3.0, 2.0, math.radians(80))),
    "RSR": (Pose2D(0.0, 0.0, 0.0), Pose2D(3.0, -2.0, math.radians(-80))),
    "LSR": (Pose2D(0.0, 0.0, 0.0), Pose2D(3.0, 0.5, math.radians(-40))),
    "RSL": (Pose2D(0.0, 0.0, 0.0), Pose2D(3.0, -0.5, math.radians(40))),
}


@pytest.mark.parametrize("mode", ["LSL", "RSR", "LSR", "RSL"])
def test_densified_arc_poses_chord_sum_tracks_arc_length(mode):
    radius = default_configuration().mechanical.minimum_turning_radius_m
    start, target = _CURVED_CASES[mode]
    normalized = _dubins_parameters(start, target, radius, mode)
    assert normalized is not None
    path = _materialize_path(start, radius, mode, normalized)
    # A real curved connector (has at least one non-zero arc primitive).
    assert any(primitive.kind != "S" and primitive.arc_angle_rad > 1e-6 for primitive in path.primitives)
    # The densified chord polyline is within 0.5% of the arc-based length_m; the
    # sparse two-pose-per-arc encoding was off by metres for the same connector.
    assert abs(_chord_sum(path.poses) - path.length_m) <= 0.005 * path.length_m
    # length_m stays arc-based (unchanged by densification) and the simple CSC
    # chord polyline does not self-intersect.
    assert not _self_intersects(path.poses)


def test_densification_keeps_endpoints_and_turn_invariant():
    # Only pose sampling changes: the connector endpoint, primitives and total
    # turn are identical to a single-advance-per-primitive materialization.
    radius = default_configuration().mechanical.minimum_turning_radius_m
    start, target = _CURVED_CASES["LSL"]
    normalized = _dubins_parameters(start, target, radius, "LSL")
    path = _materialize_path(start, radius, "LSL", normalized)
    assert path.poses[0] == start
    # Endpoint reproduced by chaining the full primitives directly.
    from tennis_robot.collection_route_connector_graph import _advance

    current = start
    for primitive in path.primitives:
        current = _advance(current, primitive, radius)
    assert path.poses[-1].x_m == pytest.approx(current.x_m)
    assert path.poses[-1].y_m == pytest.approx(current.y_m)
    assert path.poses[-1].yaw_rad == pytest.approx(current.yaw_rad)
    assert path.total_turn_rad == pytest.approx(sum(p.arc_angle_rad for p in path.primitives))


def test_feasible_forward_csc_connector_and_immutable_edge_data():
    configuration = default_configuration()
    candidate_a = candidate("a", Pose2D(4.0, 0.0, 0.0), Pose2D(5.0, 0.0, 0.0))
    built = graph(configuration, (candidate_a,))
    accepted = [edge for edge in built.edges if edge.collision_free]
    assert accepted
    assert all(edge.path.mode in {"LSL", "RSR", "LSR", "RSL"} for edge in accepted)
    assert all(edge.path.length_m <= configuration.connector.max_connector_length_m for edge in accepted)
    with pytest.raises(FrozenInstanceError):
        accepted[0].edge_id = "mutated"


def test_ccc_loop_reverse_and_standalone_rotate_are_not_generated():
    built = graph(default_configuration(), (candidate("a", Pose2D(4.0, 0.0, 0.0), Pose2D(5.0, 0.0, 0.0)),))
    assert {edge.edge_id.rsplit(":", 1)[1] for edge in built.edges} <= {"LSL", "RSR", "LSR", "RSL"}
    assert all("RLR" not in edge.edge_id and "LRL" not in edge.edge_id for edge in built.edges)


def test_max_length_rejection_is_separate_from_turning_rejection():
    configuration = replace(default_configuration(), connector=replace(default_configuration().connector, max_connector_length_m=1.0))
    built = graph(configuration, (candidate("a", Pose2D(4.0, 0.0, 0.0), Pose2D(5.0, 0.0, 0.0)),))
    assert any(edge.rejection is ConnectorRejectionCode.LENGTH_REJECTED for edge in built.edges)


def test_arc_angle_and_total_turn_limits_are_turning_constraint_rejections():
    base = default_configuration()
    configuration = replace(base, connector=replace(base.connector, max_connector_arc_angle_rad=0.1, max_connector_total_turn_rad=0.1))
    turn = candidate("turn", Pose2D(1.0, 1.0, 1.5707963267948966), Pose2D(2.0, 1.0, 1.5707963267948966))
    built = graph(configuration, (turn,))
    assert any(edge.rejection is ConnectorRejectionCode.TURNING_CONSTRAINT_REJECTED for edge in built.edges)


def test_obstacle_collision_is_reported_separately():
    configuration = default_configuration()
    obstacle = PolygonObstacle("bench", "bench", polygon((1.8, -0.2), (2.2, -0.2), (2.2, 0.2), (1.8, 0.2)))
    built = graph(configuration, (candidate("a", Pose2D(4.0, 0.0, 0.0), Pose2D(5.0, 0.0, 0.0)),), court(obstacle))
    assert any(edge.rejection is ConnectorRejectionCode.COLLISION_REJECTED for edge in built.edges)


def test_continuous_swept_arc_collision_is_detected_between_sparse_endpoints():
    base = default_configuration()
    configuration = replace(
        base,
        connector=replace(base.connector, max_connector_arc_angle_rad=2.0, max_connector_total_turn_rad=4.0),
    )
    obstacle = PolygonObstacle("arc-obstacle", "other", polygon((0.65, 0.24), (0.75, 0.24), (0.75, 0.34), (0.65, 0.34)))
    turn = candidate("turn", Pose2D(1.0, 1.0, 1.5707963267948966), Pose2D(2.0, 1.0, 1.5707963267948966))
    built = graph(configuration, (turn,), court(obstacle))
    lsl = next(edge for edge in built.edges if edge.edge_id.endswith(":LSL"))
    assert lsl.rejection is ConnectorRejectionCode.COLLISION_REJECTED


def test_graph_is_directed_from_start_and_between_distinct_passes_only():
    configuration = default_configuration()
    first = candidate("a", Pose2D(4.0, 0.0, 0.0), Pose2D(5.0, 0.0, 0.0))
    second = candidate("b", Pose2D(8.0, 0.0, 0.0), Pose2D(9.0, 0.0, 0.0))
    built = graph(configuration, (first, second))
    start_targets = {edge.target_node_id for edge in built.edges if edge.source_node_id == "start"}
    assert start_targets == {"pass:a:0", "pass:b:1"}
    assert any(edge.source_node_id == "pass:a:0" and edge.target_node_id == "pass:b:1" for edge in built.edges)
    assert any(edge.source_node_id == "pass:b:1" and edge.target_node_id == "pass:a:0" for edge in built.edges)
    assert not any(edge.source_node_id == edge.target_node_id for edge in built.edges)
