"""Pure Phase 3B1 CSC connector graph tests."""

from dataclasses import FrozenInstanceError, replace
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "ros2_ws", "src", "tennis_robot"))

from collection_route_fixtures import FAKE_TIME_S, default_configuration
import math

from tennis_robot.collection_route_connector_graph import (
    _CSC_MODES,
    _EPSILON,
    ConnectorEdge,
    ConnectorRejectionCode,
    _build_edge,
    _dubins_parameters,
    _materialize_path,
    _path_is_collision_free,
    _path_intervals,
    _self_intersects,
    build_directed_candidate_graph,
)
from tennis_robot.collection_route_planner_v2 import CourtModel, FunnelPassCandidate, PolygonObstacle
from tennis_robot.collection_route_types import Point2D, PositionCovariance2D, Pose2D, ScanSnapshot, SnapshotBall


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


def _reference_build_edge(source_id, source, target_id, target, mode, model, configuration):
    """Pre-gate behaviour: materialize every CSC path, then check limits.

    This mirrors the connector edge logic exactly as it existed before the
    analytic turning/length gate was hoisted ahead of ``_materialize_path``.
    The gated ``_build_edge`` must stay byte-identical to this reference.
    """
    edge_id = f"{source_id}->{target_id}:{mode}"
    radius = configuration.mechanical.minimum_turning_radius_m
    normalized = _dubins_parameters(source, target, radius, mode)
    if normalized is None:
        return ConnectorEdge(edge_id, source_id, target_id, None, None, False, ConnectorRejectionCode.TURNING_CONSTRAINT_REJECTED)
    path = _materialize_path(source, radius, mode, normalized)
    connector = configuration.connector
    arc_angles = tuple(primitive.arc_angle_rad for primitive in path.primitives if primitive.kind != "S")
    if (
        path.length_m > connector.max_connector_length_m + _EPSILON
        or any(angle > connector.max_connector_arc_angle_rad + _EPSILON for angle in arc_angles)
        or path.total_turn_rad > connector.max_connector_total_turn_rad + _EPSILON
        or _self_intersects(path.poses)
    ):
        return ConnectorEdge(edge_id, source_id, target_id, None, None, False, ConnectorRejectionCode.TURNING_CONSTRAINT_REJECTED if path.length_m <= connector.max_connector_length_m + _EPSILON else ConnectorRejectionCode.LENGTH_REJECTED)
    if not _path_is_collision_free(path, model, configuration.feasibility.footprint_clearance_radius_m, radius):
        return ConnectorEdge(edge_id, source_id, target_id, path, 1.0 / radius, False, ConnectorRejectionCode.COLLISION_REJECTED)
    return ConnectorEdge(edge_id, source_id, target_id, path, 1.0 / radius, True, None)


def test_analytic_gate_is_byte_identical_to_materialize_first_reference():
    configuration = default_configuration()
    model = court()
    # A dense grid of source/target poses across every CSC mode exercises all
    # gate branches: turning-rejected, length-rejected, self-intersection,
    # collision-rejected and accepted edges.
    poses = tuple(
        Pose2D(x, y, yaw)
        for x in (-6.0, -1.0, 0.0, 3.5, 12.0)
        for y in (-4.0, 0.0, 2.5)
        for yaw in (0.0, 1.2, math.pi, -2.0)
    )
    compared = 0
    for source in poses:
        for target in poses:
            if source is target:
                continue
            for mode in _CSC_MODES:
                gated = _build_edge("s", source, "t", target, mode, configuration.mechanical.minimum_turning_radius_m, 1.0, model, configuration)
                reference = _reference_build_edge("s", source, "t", target, mode, model, configuration)
                assert gated.rejection is reference.rejection
                assert gated.collision_free == reference.collision_free
                assert (gated.path is None) == (reference.path is None)
                assert gated.maximum_curvature_per_m == reference.maximum_curvature_per_m
                if gated.path is not None:
                    assert gated.path.length_m == reference.path.length_m
                    assert gated.path.total_turn_rad == reference.path.total_turn_rad
                    assert gated.path.poses == reference.path.poses
                compared += 1
    assert compared > 5000


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


def _sweep_snapshot(configuration, *balls, start=Pose2D(0.0, 0.0, 0.0)):
    return ScanSnapshot(
        "scan-sweep", FAKE_TIME_S, "map", start,
        tuple(
            SnapshotBall(ball_id, Point2D(x, y), 0.9, PositionCovariance2D(1e-8, 0.0, 1e-8))
            for ball_id, x, y in balls
        ),
        configuration,
    )


def test_connector_sweeps_a_ball_lying_on_its_straight_run():
    configuration = default_configuration()
    # The start connector runs straight along y=0 towards the pass entry, so a
    # ball parked on that line is taken in transit.
    target = candidate("a", Pose2D(8.0, 0.0, 0.0), Pose2D(9.0, 0.0, 0.0))
    # "free" is feasible in its own right; the point is that the route can take
    # it in transit instead of spending a dedicated pass on it.
    free = candidate("free", Pose2D(2.0, 0.0, 0.0), Pose2D(4.0, 0.0, 0.0))
    built = build_directed_candidate_graph(
        snapshot=_sweep_snapshot(configuration, ("a", 8.5, 0.0), ("free", 3.0, 0.0)),
        candidates=(target, free), court=court(), configuration=configuration,
    )
    start_edge = next(
        edge for edge in built.edges
        if edge.source_node_id == "start" and edge.collision_free and edge.swept_ball_ids
    )
    assert start_edge.swept_ball_ids == ("free",)
    crossing = start_edge.swept_crossings[0]
    assert crossing.ball_id == "free"
    assert crossing.progress_s >= configuration.mechanical.minimum_run_in_m
    assert start_edge.path.length_m - crossing.progress_s >= configuration.mechanical.minimum_run_out_m


def test_ball_beside_the_corridor_is_not_swept():
    configuration = default_configuration()
    target = candidate("a", Pose2D(8.0, 0.0, 0.0), Pose2D(9.0, 0.0, 0.0))
    aside = candidate("aside", Pose2D(2.0, 1.5, 0.0), Pose2D(4.0, 1.5, 0.0))
    built = build_directed_candidate_graph(
        snapshot=_sweep_snapshot(configuration, ("a", 8.5, 0.0), ("aside", 3.0, 1.5)),
        candidates=(target, aside), court=court(), configuration=configuration,
    )
    assert all("aside" not in edge.swept_ball_ids for edge in built.edges)


def test_tight_arc_portion_cannot_host_a_crossing():
    configuration = default_configuration()
    minimum = configuration.mechanical.minimum_turning_radius_m
    turning = candidate("turn", Pose2D(1.0, 1.0, math.pi / 2), Pose2D(1.0, 2.0, math.pi / 2))
    built = build_directed_candidate_graph(
        snapshot=_sweep_snapshot(configuration, ("turn", 1.0, 1.5), ("arc", minimum, 0.0)),
        candidates=(turning,), court=court(), configuration=configuration,
    )
    # Whatever the search does with the geometry, no crossing may be attached on
    # a portion tighter than the capture radius.
    for edge in built.edges:
        if not edge.collision_free:
            continue
        intervals, _ = _path_intervals(edge.path.poses)
        for crossing in edge.swept_crossings:
            hosting = [
                radius for begin, length, radius, _, _ in intervals
                if begin <= crossing.progress_s <= begin + length
            ]
            assert all(radius + 1e-9 >= configuration.connector.capture_minimum_turn_radius_m for radius in hosting)


def test_unreachable_ball_is_never_swept_in_transit():
    configuration = default_configuration()
    target = candidate("a", Pose2D(8.0, 0.0, 0.0), Pose2D(9.0, 0.0, 0.0))
    # "keepout" has no pass candidate, so per-ball feasibility rejected it and a
    # connector must not drive through it.
    built = build_directed_candidate_graph(
        snapshot=_sweep_snapshot(configuration, ("a", 8.5, 0.0), ("keepout", 3.0, 0.0)),
        candidates=(target,), court=court(), configuration=configuration,
    )
    assert all("keepout" not in edge.swept_ball_ids for edge in built.edges)


def test_gentler_radius_multipliers_add_distinct_edges():
    base = default_configuration()
    multi = replace(base, connector=replace(base.connector, sweep_radius_multipliers=(1.0, 4.0)))
    target = candidate("a", Pose2D(8.0, 0.0, 0.0), Pose2D(9.0, 0.0, 0.0))
    single_graph = graph(base, (target,))
    multi_graph = build_directed_candidate_graph(
        snapshot=snapshot(multi), candidates=(target,), court=court(), configuration=multi,
    )
    assert len(multi_graph.edges) == 2 * len(single_graph.edges)
    curvatures = {edge.maximum_curvature_per_m for edge in multi_graph.edges if edge.maximum_curvature_per_m}
    assert len(curvatures) == 2
    # The tight geometry keeps its original identity so single-radius graphs are
    # reproduced byte-for-byte.
    assert {edge.edge_id for edge in single_graph.edges} <= {edge.edge_id for edge in multi_graph.edges}
