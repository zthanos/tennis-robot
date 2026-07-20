"""Pure Phase 3B2 bounded global route search; no runtime dependencies."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import math

from tennis_robot.collection_route_connector_graph import ConnectorEdge, DirectedCandidateGraph
from tennis_robot.collection_route_planner_v2 import CourtModel, FunnelPassCandidate, PerBallFeasibility, _segment_is_collision_free
from tennis_robot.collection_route_types import (
    BallReasonCode, BallResult, BallStatus, CollectionRouteConfiguration,
    CollectionRoutePlan, ObstacleConstraint, ObstacleConstraintKind, Path2D,
    PathPoint, PlanningSearchStatus, PlanningStatus, Point2D, Pose2D,
    PlannedCrossing, RouteSegment, RouteSegmentType, ScanSnapshot,
)


@dataclass(frozen=True)
class _Route:
    node_ids: tuple[str, ...]
    edges: tuple[ConnectorEdge, ...]
    terminal_pose: Pose2D
    length_m: float
    duration_s: float
    curvature_rad: float
    covered_ball_ids: tuple[str, ...]


def solve_global_route(*, snapshot: ScanSnapshot, feasibility: tuple[PerBallFeasibility, ...], graph: DirectedCandidateGraph, court: CourtModel, configuration: CollectionRouteConfiguration) -> CollectionRoutePlan:
    if snapshot.configuration_snapshot != configuration:
        raise ValueError("configuration must exactly match snapshot configuration")
    if not isinstance(feasibility, tuple) or not isinstance(graph, DirectedCandidateGraph) or not isinstance(court, CourtModel):
        raise ValueError("invalid pure solver inputs")
    by_node = dict(graph.pass_nodes)
    all_ball_ids = tuple(ball.ball_id for ball in snapshot.balls)
    if set(ball_id for _, candidate in graph.pass_nodes for ball_id in candidate.covered_ball_ids) - set(all_ball_ids):
        raise ValueError("candidate covers ball outside snapshot")
    if not all_ball_ids:
        return _empty_plan(snapshot, configuration, PlanningStatus.EMPTY_NO_BALLS, PlanningSearchStatus.COMPLETE, ())

    valid = [edge for edge in graph.edges if edge.rejection is None and edge.path is not None]
    outgoing: dict[str, list[ConnectorEdge]] = {}
    for edge in valid:
        outgoing.setdefault(edge.source_node_id, []).append(edge)
    for edges in outgoing.values():
        edges.sort(key=lambda edge: edge.edge_id)

    budget = configuration.global_route_search.max_search_expansions
    expansions = 0
    exhausted = False
    expanded_nodes: set[str] = set()
    best: _Route | None = None

    def consider(route: _Route) -> None:
        nonlocal best
        if best is None or _route_score(route, configuration) < _route_score(best, configuration):
            best = route

    def dfs(current_node_id: str, node_ids: tuple[str, ...], edges: tuple[ConnectorEdge, ...]) -> None:
        nonlocal expansions, exhausted
        if exhausted:
            return
        expansions += 1
        if expansions > budget:
            exhausted = True
            return
        expanded_nodes.add(current_node_id)
        candidate = by_node[current_node_id]
        terminal = _terminal_route(candidate, court, configuration)
        if terminal is not None:
            length, duration, terminal_pose = terminal
            connector_length = sum(edge.path.length_m for edge in edges)
            connector_turn = sum(edge.path.total_turn_rad for edge in edges)
            pass_length = sum(_pass_length(by_node[node_id], configuration) for node_id in node_ids)
            pass_duration = sum(_pass_length(by_node[node_id], configuration) / configuration.global_route_search.crossing_nominal_speed_m_s for node_id in node_ids)
            covered = tuple(sorted({ball_id for node_id in node_ids for ball_id in by_node[node_id].covered_ball_ids}))
            consider(_Route(node_ids, edges, terminal_pose, connector_length + pass_length + length, sum(edge.path.length_m / configuration.global_route_search.connector_nominal_speed_m_s for edge in edges) + pass_duration + duration, connector_turn, covered))
        for edge in outgoing.get(current_node_id, ()):
            if edge.target_node_id in node_ids:
                continue
            current_covered = {ball_id for node_id in node_ids for ball_id in by_node[node_id].covered_ball_ids}
            if current_covered & set(by_node[edge.target_node_id].covered_ball_ids):
                continue
            dfs(edge.target_node_id, node_ids + (edge.target_node_id,), edges + (edge,))

    for edge in outgoing.get("start", ()):
        if exhausted:
            break
        dfs(edge.target_node_id, (edge.target_node_id,), (edge,))

    search_status = PlanningSearchStatus.BUDGET_EXHAUSTED if exhausted else PlanningSearchStatus.COMPLETE
    if best is None:
        status = PlanningStatus.PLANNING_TIMEOUT if outgoing.get("start") else PlanningStatus.EMPTY_NO_FEASIBLE_TARGETS
        return _empty_plan(snapshot, configuration, status, search_status, _ball_results(snapshot, feasibility, (), exhausted, expanded_nodes, by_node, graph, court, configuration))
    status = PlanningStatus.FEASIBLE if set(best.covered_ball_ids) == set(all_ball_ids) and not exhausted else PlanningStatus.PARTIAL
    selected_passes = {
        ball_id: f"pass:{node_id}"
        for node_id in best.node_ids
        for ball_id in by_node[node_id].covered_ball_ids
    }
    return _plan_from_route(snapshot, configuration, best, status, search_status, _ball_results(snapshot, feasibility, best.covered_ball_ids, exhausted, expanded_nodes, by_node, graph, court, configuration, selected_passes), by_node)


def _terminal_route(candidate, court, configuration):
    length = configuration.global_route_search.terminal_run_out_m
    start = candidate.exit_pose
    terminal = Pose2D(start.x_m + length * math.cos(start.yaw_rad), start.y_m + length * math.sin(start.yaw_rad), start.yaw_rad)
    if not _segment_is_collision_free(Point2D(start.x_m, start.y_m), Point2D(terminal.x_m, terminal.y_m), court, configuration.feasibility.footprint_clearance_radius_m):
        return None
    return length, length / configuration.global_route_search.connector_nominal_speed_m_s, terminal


def _pass_length(candidate, configuration):
    return math.hypot(candidate.crossing.x_m - candidate.entry_pose.x_m, candidate.crossing.y_m - candidate.entry_pose.y_m) + math.hypot(candidate.exit_pose.x_m - candidate.crossing.x_m, candidate.exit_pose.y_m - candidate.crossing.y_m)


def _route_score(route, configuration):
    search = configuration.global_route_search
    energy = route.length_m + search.turn_energy_equivalent_m_per_rad * route.curvature_rad
    cost = search.weight_length * route.length_m + search.weight_time * route.duration_s + search.weight_curvature * route.curvature_rad + search.weight_energy * energy + search.weight_pass_count * len(route.node_ids)
    route_id = "/".join(route.node_ids) + "|" + "/".join(edge.edge_id for edge in route.edges)
    return (-len(route.covered_ball_ids), cost, len(route.node_ids), route_id)


def _ball_results(snapshot, feasibility, covered_ids, exhausted, expanded_nodes, by_node, graph, court, configuration, selected_passes=None):
    infeasible = {item.ball_id: item.unreachable_reason for item in feasibility if not item.reachable}
    covered = set(covered_ids)
    expanded_coverage = {ball_id for node_id in expanded_nodes for ball_id in by_node[node_id].covered_ball_ids}
    results = []
    for ball in snapshot.balls:
        if ball.ball_id in infeasible:
            results.append(BallResult(ball.ball_id, BallStatus.UNREACHABLE, infeasible[ball.ball_id]))
        elif ball.ball_id in covered:
            results.append(BallResult(ball.ball_id, BallStatus.COVERED, BallReasonCode.SELECTED, selected_passes[ball.ball_id]))
        elif exhausted and ball.ball_id not in expanded_coverage:
            results.append(BallResult(ball.ball_id, BallStatus.DEFERRED, BallReasonCode.PLANNING_BUDGET))
        elif not exhausted and _turning_only_unreachable(ball.ball_id, graph, by_node, court, configuration):
            results.append(BallResult(ball.ball_id, BallStatus.UNREACHABLE, BallReasonCode.TURN_RADIUS))
        else:
            results.append(BallResult(ball.ball_id, BallStatus.DEFERRED, BallReasonCode.ROUTE_CONFLICT))
    return tuple(results)


def _turning_only_unreachable(ball_id, graph, by_node, court, configuration):
    targets = {node_id for node_id, candidate in by_node.items() if ball_id in candidate.covered_ball_ids}
    if not targets:
        return False
    return not _graph_route_through(targets, graph, by_node, court, configuration, allow_turning_rejections=False) and _graph_route_through(targets, graph, by_node, court, configuration, allow_turning_rejections=True)


def _graph_route_through(targets, graph, by_node, court, configuration, allow_turning_rejections):
    allowed = [edge for edge in graph.edges if edge.rejection is None or (allow_turning_rejections and edge.rejection.value == "turning_constraint_rejected")]
    forward = {"start"}
    changed = True
    while changed:
        changed = False
        for edge in allowed:
            if edge.source_node_id in forward and edge.target_node_id not in forward:
                forward.add(edge.target_node_id); changed = True
    terminal_nodes = {node_id for node_id, candidate in by_node.items() if _terminal_route(candidate, court, configuration) is not None}
    reverse = set(terminal_nodes)
    changed = True
    while changed:
        changed = False
        for edge in allowed:
            if edge.target_node_id in reverse and edge.source_node_id != "start" and edge.source_node_id not in reverse:
                reverse.add(edge.source_node_id); changed = True
    return bool(targets & forward & reverse)


def _plan_from_route(snapshot, configuration, route, status, search_status, results, by_node):
    segments = []
    progress = 0.0
    for edge, node_id in zip(route.edges, route.node_ids):
        segments.append(_connector_segment(f"connector-{len(segments)}", edge, progress, configuration))
        progress = segments[-1].progress_end_m
        candidate = by_node[node_id]
        segments.append(_pass_segment(f"pass:{node_id}", candidate, progress, configuration, snapshot))
        progress = segments[-1].progress_end_m
    last = by_node[route.node_ids[-1]]
    terminal_start = last.exit_pose
    segments.append(RouteSegment(f"terminal-{last.ball_id}", RouteSegmentType.TERMINAL_CONNECTOR, Path2D((PathPoint(terminal_start), PathPoint(route.terminal_pose))), progress, progress + configuration.global_route_search.terminal_run_out_m, configuration.planning.default_execution_profile, (), ObstacleConstraint(ObstacleConstraintKind.NONE, (), 0.0)))
    route_id_seed = "/".join(route.node_ids) + "|" + "/".join(edge.edge_id for edge in route.edges)
    plan_id = "route-" + hashlib.sha256(route_id_seed.encode()).hexdigest()[:16]
    return CollectionRoutePlan(plan_id, snapshot.scan_id, snapshot.map_frame, snapshot.robot_pose_at_scan, route.terminal_pose, route.length_m, route.duration_s, status, search_status, tuple(segments), tuple(ball.ball_id for ball in snapshot.balls), results, configuration)


def _connector_segment(segment_id, edge, progress, configuration):
    path = Path2D(tuple(PathPoint(pose) for pose in edge.path.poses))
    return RouteSegment(segment_id, RouteSegmentType.CONNECTOR, path, progress, progress + edge.path.length_m, configuration.planning.default_execution_profile, (), ObstacleConstraint(ObstacleConstraintKind.NONE, (), 0.0))


def _pass_segment(segment_id, candidate, progress, configuration, snapshot):
    length = _pass_length(candidate, configuration)
    direction = (math.cos(candidate.heading_rad), math.sin(candidate.heading_rad))
    normal = (-direction[1], direction[0])
    crossings = []
    for ball_id, position in zip(candidate.covered_ball_ids, candidate.crossing_positions):
        dx = position.x_m - candidate.entry_pose.x_m
        dy = position.y_m - candidate.entry_pose.y_m
        along = dx * direction[0] + dy * direction[1]
        lateral = dx * normal[0] + dy * normal[1]
        centreline_position = Point2D(
            candidate.entry_pose.x_m + along * direction[0],
            candidate.entry_pose.y_m + along * direction[1],
        )
        crossings.append(PlannedCrossing(ball_id, centreline_position, progress + along, candidate.heading_rad, lateral))
    crossings = tuple(crossings)
    path = Path2D((PathPoint(candidate.entry_pose),) + tuple(PathPoint(Pose2D(item.position_xy.x_m, item.position_xy.y_m, item.heading_rad)) for item in crossings) + (PathPoint(candidate.exit_pose),))
    return RouteSegment(segment_id, RouteSegmentType.FUNNEL_PASS, path, progress, progress + length, configuration.planning.default_execution_profile, candidate.covered_ball_ids, ObstacleConstraint(ObstacleConstraintKind.NONE, (), 0.0), crossings)


def _empty_plan(snapshot, configuration, status, search_status, results):
    return CollectionRoutePlan("route-empty-" + snapshot.scan_id, snapshot.scan_id, snapshot.map_frame, snapshot.robot_pose_at_scan, snapshot.robot_pose_at_scan, 0.0, 0.0, status, search_status, (), tuple(ball.ball_id for ball in snapshot.balls), results, configuration)
