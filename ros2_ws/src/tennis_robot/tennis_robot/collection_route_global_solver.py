"""Pure Phase 3B2 bounded global route search; no runtime dependencies."""

from __future__ import annotations

from dataclasses import dataclass, replace
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


def solve_global_route(*, snapshot: ScanSnapshot, feasibility: tuple[PerBallFeasibility, ...], graph: DirectedCandidateGraph, court: CourtModel, configuration: CollectionRouteConfiguration, candidate_budget_exhausted: bool = False) -> CollectionRoutePlan:
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

    # Static over-approximation of the distinct balls forward-reachable from
    # each node via valid edges (ignoring path/coverage constraints, which only
    # shrink the true set).  Used as an admissible coverage upper bound so the
    # search can prune any branch that provably cannot match the best coverage
    # found so far.  Fixpoint over a graph that may contain cycles.
    reachable_balls: dict[str, set[str]] = {
        node_id: set(candidate.covered_ball_ids) for node_id, candidate in by_node.items()
    }
    changed = True
    while changed:
        changed = False
        for node_id, node_edges in outgoing.items():
            if node_id == "start":
                continue
            accumulated = reachable_balls[node_id]
            before = len(accumulated)
            for edge in node_edges:
                # Balls swept in transit count towards reachable coverage, or the
                # bound would prune branches that collect on the way.
                accumulated |= reachable_balls[edge.target_node_id] | set(edge.swept_ball_ids)
            if len(accumulated) != before:
                changed = True

    budget = configuration.global_route_search.max_search_expansions
    expansions = 0
    search_exhausted = False
    expanded_nodes: set[str] = set()
    best: _Route | None = None

    def consider(route: _Route) -> None:
        nonlocal best
        if best is None or _route_score(route, configuration) < _route_score(best, configuration):
            best = route

    def dfs(current_node_id: str, node_ids: tuple[str, ...], edges: tuple[ConnectorEdge, ...]) -> None:
        nonlocal expansions, search_exhausted
        if search_exhausted:
            return
        expansions += 1
        if expansions > budget:
            search_exhausted = True
            return
        expanded_nodes.add(current_node_id)
        candidate = by_node[current_node_id]
        current_covered = {ball_id for node_id in node_ids for ball_id in by_node[node_id].covered_ball_ids}
        current_covered |= {ball_id for edge in edges for ball_id in edge.swept_ball_ids}
        search = configuration.global_route_search
        connector_length = sum(edge.path.length_m for edge in edges)
        connector_turn = sum(edge.path.total_turn_rad for edge in edges)
        pass_length = sum(_pass_length(by_node[node_id], configuration) for node_id in node_ids)
        terminal = _terminal_route(candidate, court, configuration)
        if terminal is not None:
            length, duration, terminal_pose = terminal
            pass_duration = pass_length / search.crossing_nominal_speed_m_s
            covered = tuple(sorted(current_covered))
            consider(_Route(node_ids, edges, terminal_pose, connector_length + pass_length + length, sum(edge.path.length_m / search.connector_nominal_speed_m_s for edge in edges) + pass_duration + duration, connector_turn, covered))
        # Admissible pruning: descendants of this prefix can neither exceed the
        # forward-reachable coverage nor undercut the prefix's own cost lower
        # bound (every extension only adds length/turn/passes plus the mandatory
        # terminal run-out).  Prune when the incumbent already dominates.
        if best is not None:
            max_reachable = len(current_covered | reachable_balls[current_node_id])
            best_coverage = len(best.covered_ball_ids)
            if max_reachable < best_coverage:
                return
            if max_reachable == best_coverage and _prefix_cost_lower_bound(
                connector_length, connector_turn, pass_length, len(node_ids), configuration
            ) > _route_cost(best, configuration) + _COST_EPSILON:
                return
        # Explore high-new-coverage, then cheaper, then stable-id edges first so a
        # strong incumbent is found early and the coverage bound prunes hard.
        def order_key(edge: ConnectorEdge):
            gained = set(by_node[edge.target_node_id].covered_ball_ids) | set(edge.swept_ball_ids)
            return (-len(gained - current_covered), edge.path.length_m, edge.edge_id)
        for edge in sorted(outgoing.get(current_node_id, ()), key=order_key):
            if edge.target_node_id in node_ids:
                continue
            swept = set(edge.swept_ball_ids)
            # A ball may be covered exactly once in a plan, so an edge that sweeps
            # something already collected, or that duplicates its own target's
            # coverage, is not a usable extension.
            if current_covered & swept:
                continue
            if (current_covered | swept) & set(by_node[edge.target_node_id].covered_ball_ids):
                continue
            if best is not None and len(current_covered | swept | reachable_balls[edge.target_node_id]) < len(best.covered_ball_ids):
                continue
            dfs(edge.target_node_id, node_ids + (edge.target_node_id,), edges + (edge,))

    for edge in sorted(outgoing.get("start", ()), key=lambda edge: (edge.path.length_m, edge.edge_id)):
        if search_exhausted:
            break
        dfs(edge.target_node_id, (edge.target_node_id,), (edge,))

    exhausted = search_exhausted or candidate_budget_exhausted
    search_status = PlanningSearchStatus.BUDGET_EXHAUSTED if exhausted else PlanningSearchStatus.COMPLETE
    if best is None:
        # Derive the status from the actual ball outcomes, not from whether a
        # start edge exists.  EMPTY_NO_FEASIBLE_TARGETS is reserved for the case
        # where every target is deterministically UNREACHABLE (the run ends as
        # completed_no_targets).  A ball that is 3A-feasible but left out of any
        # valid terminal route is DEFERRED, which is a non-executable
        # PLANNING_TIMEOUT, never a "no feasible targets" outcome.
        results = _ball_results(snapshot, feasibility, (), exhausted, expanded_nodes, by_node, graph, court, configuration)
        status = PlanningStatus.EMPTY_NO_FEASIBLE_TARGETS if all(result.status is BallStatus.UNREACHABLE for result in results) else PlanningStatus.PLANNING_TIMEOUT
        return _empty_plan(snapshot, configuration, status, search_status, results)
    # Coverage determines feasible vs partial.  Search/candidate exhaustion is
    # reported independently; a budgeted search may still prove a route that
    # covers every snapshot target.
    status = PlanningStatus.FEASIBLE if set(best.covered_ball_ids) == set(all_ball_ids) else PlanningStatus.PARTIAL
    selected_passes = {
        ball_id: f"pass:{node_id}"
        for node_id in best.node_ids
        for ball_id in by_node[node_id].covered_ball_ids
    }
    # Balls taken in transit are attributed to the connector that swept them.
    for index, edge in enumerate(best.edges):
        for ball_id in edge.swept_ball_ids:
            selected_passes[ball_id] = f"connector-{2 * index}"
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


_COST_EPSILON = 1e-9


def _cost_value(length_m, duration_s, curvature_rad, pass_count, configuration):
    search = configuration.global_route_search
    energy = length_m + search.turn_energy_equivalent_m_per_rad * curvature_rad
    return search.weight_length * length_m + search.weight_time * duration_s + search.weight_curvature * curvature_rad + search.weight_energy * energy + search.weight_pass_count * pass_count


def _route_cost(route, configuration):
    return _cost_value(route.length_m, route.duration_s, route.curvature_rad, len(route.node_ids), configuration)


def _prefix_cost_lower_bound(connector_length, connector_turn, pass_length, pass_count, configuration):
    """Admissible cost lower bound for any route extending this prefix.

    Every extension only adds connector/pass length, curvature and passes, and
    every route pays the same fixed terminal run-out, so terminating the prefix
    here yields a value no greater than any descendant's final cost.
    """
    search = configuration.global_route_search
    run_out = search.terminal_run_out_m
    length = connector_length + pass_length + run_out
    duration = connector_length / search.connector_nominal_speed_m_s + pass_length / search.crossing_nominal_speed_m_s + run_out / search.connector_nominal_speed_m_s
    return _cost_value(length, duration, connector_turn, pass_count, configuration)


def _route_score(route, configuration):
    cost = _cost_value(route.length_m, route.duration_s, route.curvature_rad, len(route.node_ids), configuration)
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


def _connector_execution_profile(configuration):
    """Default profile with the connector-specific (looser) heading hard gate.

    Connectors are transit, not capture: pure-pursuit steering leads the path
    tangent on their curves, so the capture-grade heading gate would self-abort.
    Every other profile bound (speed, curvature, lateral tube) is unchanged.
    """
    return replace(
        configuration.planning.default_execution_profile,
        max_heading_error_rad=configuration.planning.connector_max_heading_error_rad,
    )


def _connector_segment(segment_id, edge, progress, configuration):
    path = Path2D(tuple(PathPoint(pose) for pose in edge.path.poses))
    # Edge-local crossing progress is chord-based while the segment span uses the
    # arc length, and arc >= chord, so rebasing keeps every crossing inside the
    # segment with at least the required run-out behind it.
    crossings = tuple(
        replace(crossing, progress_s=progress + crossing.progress_s)
        for crossing in edge.swept_crossings
    )
    # A connector that collects is capture motion for that stretch, so it has to
    # hold the capture-grade heading gate rather than the loose transit one. The
    # sweep detector only admits crossings on portions gentle enough to pass it.
    profile = (
        configuration.planning.default_execution_profile
        if crossings
        else _connector_execution_profile(configuration)
    )
    return RouteSegment(segment_id, RouteSegmentType.CONNECTOR, path, progress, progress + edge.path.length_m, profile, edge.swept_ball_ids, ObstacleConstraint(ObstacleConstraintKind.NONE, (), 0.0), crossings)


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
