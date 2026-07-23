"""Pure Phase 3A per-ball feasibility for the continuous-route rewrite.

This module deliberately produces independent straight funnel-pass candidates
only.  It does not order targets, construct connectors, score routes, or make
any ROS/runtime decision.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

from tennis_robot.collection_route_types import (
    BallReasonCode,
    CollectionRouteConfiguration,
    CollectionRoutePlan,
    DomainValidationError,
    Point2D,
    Pose2D,
    ScanSnapshot,
)


_OBSTACLE_KINDS = frozenset({"net", "fence", "post", "bench", "other"})
_EPSILON = 1e-9


class PlannerInputError(ValueError):
    """The pure planner was called without the required explicit inputs."""


@dataclass(frozen=True)
class PolygonObstacle:
    obstacle_id: str
    kind: str
    polygon: tuple[Point2D, ...]

    def __post_init__(self) -> None:
        if not self.obstacle_id:
            raise PlannerInputError("obstacle_id must be non-empty")
        if self.kind not in _OBSTACLE_KINDS:
            raise PlannerInputError("invalid obstacle kind")
        _validate_polygon(self.polygon, f"obstacle {self.obstacle_id}")


@dataclass(frozen=True)
class CourtModel:
    navigable_polygon: tuple[Point2D, ...]
    obstacles: tuple[PolygonObstacle, ...]

    def __post_init__(self) -> None:
        _validate_polygon(self.navigable_polygon, "navigable_polygon")
        if not isinstance(self.obstacles, tuple) or any(
            not isinstance(obstacle, PolygonObstacle) for obstacle in self.obstacles
        ):
            raise PlannerInputError("obstacles must be a tuple of PolygonObstacle")
        if len({obstacle.obstacle_id for obstacle in self.obstacles}) != len(self.obstacles):
            raise PlannerInputError("obstacle IDs must be unique")


@dataclass(frozen=True)
class FunnelPassCandidate:
    ball_id: str
    covered_ball_ids: tuple[str, ...]
    heading_rad: float
    entry_pose: Pose2D
    crossing: Point2D
    exit_pose: Pose2D
    effective_capture_half_width_m: float
    crossing_positions: tuple[Point2D, ...]

    def __post_init__(self) -> None:
        if not self.ball_id or not isinstance(self.covered_ball_ids, tuple) or not self.covered_ball_ids:
            raise PlannerInputError("pass candidate requires covered ball IDs")
        if any(not isinstance(ball_id, str) or not ball_id for ball_id in self.covered_ball_ids):
            raise PlannerInputError("covered ball IDs must be non-empty strings")
        if len(set(self.covered_ball_ids)) != len(self.covered_ball_ids):
            raise PlannerInputError("covered ball IDs must be unique")
        if not isinstance(self.crossing_positions, tuple) or len(self.crossing_positions) != len(self.covered_ball_ids) or any(not isinstance(item, Point2D) for item in self.crossing_positions):
            raise PlannerInputError("candidate requires one Point2D crossing position per covered ball")


@dataclass(frozen=True)
class PerBallFeasibility:
    ball_id: str
    candidates: tuple[FunnelPassCandidate, ...]
    unreachable_reason: BallReasonCode | None

    def __post_init__(self) -> None:
        if not self.ball_id:
            raise PlannerInputError("ball_id must be non-empty")
        if self.candidates and self.unreachable_reason is not None:
            raise PlannerInputError("feasible target cannot have unreachable reason")
        if not self.candidates and self.unreachable_reason is None:
            raise PlannerInputError("infeasible target requires deterministic reason")
        if self.unreachable_reason is BallReasonCode.TURN_RADIUS:
            raise PlannerInputError("turn_radius belongs to Phase 3B connector feasibility")

    @property
    def reachable(self) -> bool:
        return bool(self.candidates)


def analyze_snapshot(
    snapshot: ScanSnapshot,
    court: CourtModel,
    configuration: CollectionRouteConfiguration,
) -> tuple[PerBallFeasibility, ...]:
    """Return all deterministic straight-pass candidates for every snapshot ball."""
    if not isinstance(snapshot, ScanSnapshot):
        raise PlannerInputError("snapshot must be ScanSnapshot")
    if not isinstance(court, CourtModel):
        raise PlannerInputError("court must be CourtModel")
    if not isinstance(configuration, CollectionRouteConfiguration):
        raise PlannerInputError("configuration must be CollectionRouteConfiguration")
    if snapshot.configuration_snapshot != configuration:
        raise PlannerInputError("configuration must exactly match snapshot configuration")
    return tuple(_analyze_ball(ball, court, configuration) for ball in snapshot.balls)


@dataclass(frozen=True)
class PlannerResult:
    """Immutable output of one pure planner run.

    ``plan`` is the frozen route artifact; the remaining fields are
    planner-run telemetry that is not a property of the route itself and is
    deliberately kept out of the ``CollectionRoutePlan`` contract.  New
    planner telemetry is added here, not on the plan.
    """
    plan: CollectionRoutePlan
    shared_pass_candidate_budget_exhausted: bool

    def __post_init__(self) -> None:
        if not isinstance(self.plan, CollectionRoutePlan):
            raise PlannerInputError("PlannerResult requires a CollectionRoutePlan")
        if not isinstance(self.shared_pass_candidate_budget_exhausted, bool):
            raise PlannerInputError("shared_pass_candidate_budget_exhausted must be bool")


def plan_collection_route(*, snapshot: ScanSnapshot, court: CourtModel, configuration: CollectionRouteConfiguration) -> PlannerResult:
    """Compose the pure Phase 3A→3C→3B1→3B2 planner pipeline.

    This is intentionally the sole planner orchestration API.  It imports the
    lower-level pure layers lazily to keep their geometry contracts acyclic.
    Returns a ``PlannerResult`` wrapping the frozen plan and planner telemetry.
    """
    from tennis_robot.collection_route_connector_graph import build_directed_candidate_graph
    from tennis_robot.collection_route_global_solver import solve_global_route
    from tennis_robot.collection_route_shared_pass import generate_shared_passes

    individual = analyze_snapshot(snapshot, court, configuration)
    single_candidates = tuple(
        candidate
        for result in individual
        for candidate in result.candidates
        if len(candidate.covered_ball_ids) == 1
    )
    shared = generate_shared_passes(
        single_ball_candidates=single_candidates,
        court=court,
        configuration=configuration,
    )
    candidates = _merge_candidates(single_candidates + shared.candidates)
    candidates, candidate_budget_exhausted = _bounded_candidates(
        snapshot, candidates, configuration.planning.maximum_candidate_count
    )
    graph = build_directed_candidate_graph(
        snapshot=snapshot,
        candidates=candidates,
        court=court,
        configuration=configuration,
    )
    plan = solve_global_route(
        snapshot=snapshot,
        feasibility=individual,
        graph=graph,
        court=court,
        configuration=configuration,
        candidate_budget_exhausted=candidate_budget_exhausted,
    )
    return PlannerResult(plan, shared.candidate_budget_exhausted)


def _bounded_candidates(
    snapshot: ScanSnapshot,
    candidates: tuple[FunnelPassCandidate, ...],
    limit: int,
) -> tuple[tuple[FunnelPassCandidate, ...], bool]:
    """Apply the declared graph-node budget before O(N²) edge generation.

    Greedy set coverage gives every target a deterministic chance to enter the
    graph (and naturally prefers useful shared passes).  Remaining slots add
    the closest alternatives.  This keeps graph construction bounded without
    silently treating an unexamined target as geometrically unreachable.
    """
    if len(candidates) <= limit:
        return candidates, False

    start = snapshot.robot_pose_at_scan

    def stable_key(item: FunnelPassCandidate):
        return (
            math.hypot(item.entry_pose.x_m - start.x_m, item.entry_pose.y_m - start.y_m),
            -len(item.covered_ball_ids), item.covered_ball_ids, item.heading_rad,
            item.entry_pose.x_m, item.entry_pose.y_m, item.exit_pose.x_m,
            item.exit_pose.y_m, item.ball_id,
        )

    remaining = list(candidates)
    selected: list[FunnelPassCandidate] = []
    covered: set[str] = set()
    coverage_counts: dict[str, int] = {ball.ball_id: 0 for ball in snapshot.balls}
    target_ids = {ball.ball_id for ball in snapshot.balls}
    while remaining and len(selected) < limit and covered != target_ids:
        item = min(
            remaining,
            key=lambda candidate: (
                -len(set(candidate.covered_ball_ids) - covered), stable_key(candidate)
            ),
        )
        newly_covered = set(item.covered_ball_ids) - covered
        if not newly_covered:
            break
        selected.append(item)
        remaining.remove(item)
        covered.update(item.covered_ball_ids)
        for ball_id in item.covered_ball_ids:
            coverage_counts[ball_id] += 1

    while remaining and len(selected) < limit:
        # Round-robin alternatives across targets instead of spending every
        # remaining slot on headings for whichever ball is closest to start.
        item = min(
            remaining,
            key=lambda candidate: (
                min(coverage_counts[ball_id] for ball_id in candidate.covered_ball_ids),
                sum(coverage_counts[ball_id] for ball_id in candidate.covered_ball_ids),
                stable_key(candidate),
            ),
        )
        selected.append(item)
        remaining.remove(item)
        for ball_id in item.covered_ball_ids:
            coverage_counts[ball_id] += 1
    return tuple(selected), True


def _merge_candidates(candidates: tuple[FunnelPassCandidate, ...]) -> tuple[FunnelPassCandidate, ...]:
    """Stable exact deduplication; no geometric tolerance or hidden heuristic."""
    ordered = sorted(
        candidates,
        key=lambda item: (
            item.covered_ball_ids,
            item.heading_rad,
            item.entry_pose.x_m,
            item.entry_pose.y_m,
            item.exit_pose.x_m,
            item.exit_pose.y_m,
            item.ball_id,
        ),
    )
    unique: list[FunnelPassCandidate] = []
    seen = set()
    for candidate in ordered:
        key = (
            candidate.covered_ball_ids,
            candidate.heading_rad,
            candidate.entry_pose,
            candidate.crossing,
            candidate.exit_pose,
        )
        if key not in seen:
            seen.add(key)
            unique.append(candidate)
    return tuple(unique)


def _analyze_ball(ball, court: CourtModel, configuration: CollectionRouteConfiguration) -> PerBallFeasibility:
    feasibility = configuration.feasibility
    mechanical = configuration.mechanical
    clearance = feasibility.footprint_clearance_radius_m
    crossing = ball.position

    if not _point_in_eroded_polygon(crossing, court.navigable_polygon, clearance):
        return PerBallFeasibility(ball.ball_id, (), BallReasonCode.KEEPOUT)
    if any(_point_hits_inflated_polygon(crossing, obstacle.polygon, clearance) for obstacle in court.obstacles):
        return PerBallFeasibility(ball.ball_id, (), BallReasonCode.KEEPOUT)

    tangent_headings = _active_tangent_headings(crossing, court, feasibility.tangent_activation_distance_m)
    headings = _candidate_headings(feasibility.heading_sample_count, tangent_headings)
    tangent_valid = [
        heading
        for heading in headings
        if _satisfies_tangent_constraints(
            heading, tangent_headings, feasibility.max_parallel_heading_error_rad
        )
    ]
    if not tangent_valid:
        return PerBallFeasibility(ball.ball_id, (), BallReasonCode.NO_ENTRY)

    candidates: list[FunnelPassCandidate] = []
    corridor_collapsed = False
    entry_failed = False
    exit_failed = False
    for heading in tangent_valid:
        effective_width = _effective_capture_half_width(ball, heading, configuration)
        if effective_width <= 0.0:
            # Uncertainty/margins consume the whole capture corridor for this
            # heading.  This is not an entry-geometry failure, so it is tracked
            # separately and reported as no_candidate_found.
            corridor_collapsed = True
            continue
        direction = (math.cos(heading), math.sin(heading))
        entry = Point2D(
            crossing.x_m - mechanical.minimum_run_in_m * direction[0],
            crossing.y_m - mechanical.minimum_run_in_m * direction[1],
        )
        exit_point = Point2D(
            crossing.x_m + mechanical.minimum_run_out_m * direction[0],
            crossing.y_m + mechanical.minimum_run_out_m * direction[1],
        )
        if not _segment_is_collision_free(entry, crossing, court, clearance):
            entry_failed = True
            continue
        if not _segment_is_collision_free(crossing, exit_point, court, clearance):
            exit_failed = True
            continue
        candidates.append(
            FunnelPassCandidate(
                ball.ball_id,
                (ball.ball_id,),
                heading,
                Pose2D(entry.x_m, entry.y_m, heading),
                crossing,
                Pose2D(exit_point.x_m, exit_point.y_m, heading),
                effective_width,
                (crossing,),
            )
        )
    if candidates:
        return PerBallFeasibility(ball.ball_id, tuple(candidates), None)
    # Report the earliest blocking geometric stage; a pure corridor collapse
    # (no entry/exit geometry failure) is no_candidate_found, not no_entry.
    if entry_failed:
        reason = BallReasonCode.NO_ENTRY
    elif exit_failed:
        reason = BallReasonCode.NO_EXIT
    else:
        reason = BallReasonCode.NO_CANDIDATE_FOUND
    return PerBallFeasibility(ball.ball_id, (), reason)


def _effective_capture_half_width(ball, heading: float, configuration: CollectionRouteConfiguration) -> float:
    normal_x, normal_y = -math.sin(heading), math.cos(heading)
    covariance = ball.position_covariance
    projected_map_variance = (
        normal_x * normal_x * covariance.xx
        + 2.0 * normal_x * normal_y * covariance.xy
        + normal_y * normal_y * covariance.yy
    )
    # SnapshotBall covariance contains the shared map-localization budget once.
    # The robot and ball are expressed in that same map realization, so this
    # common translational error cancels for the relative funnel crossing. Keep
    # it in the published map covariance/keepout semantics, but subtract its
    # projection for the relative capture-width calculation. Tracking error is
    # still charged independently below.
    localization = configuration.gazebo_snapshot.localization_xy_covariance.covariance
    projected_shared_localization_variance = (
        normal_x * normal_x * localization.xx
        + 2.0 * normal_x * normal_y * localization.xy
        + normal_y * normal_y * localization.yy
    )
    projected_variance = max(
        0.0, projected_map_variance - projected_shared_localization_variance
    )
    projected_stddev = math.sqrt(max(0.0, projected_variance))
    feasibility = configuration.feasibility
    mechanical = configuration.mechanical
    return (
        mechanical.capture_half_width_m
        - mechanical.ball_radius_m
        - feasibility.confidence_multiplier * projected_stddev
        - feasibility.tracking_lateral_error_bound_m
        - feasibility.capture_safety_margin_m
    )


def _candidate_headings(sample_count: int, tangent_headings: tuple[float, ...]) -> tuple[float, ...]:
    headings = [2.0 * math.pi * index / sample_count for index in range(sample_count)]
    headings.extend(tangent_headings)
    return tuple(sorted({_normalize_heading(heading) for heading in headings}))


def _active_tangent_headings(
    point: Point2D, court: CourtModel, activation_distance_m: float
) -> tuple[float, ...]:
    headings: list[float] = []
    for obstacle in court.obstacles:
        if obstacle.kind not in {"net", "fence"}:
            continue
        for start, end in _polygon_edges(obstacle.polygon):
            if _point_segment_distance(point, start, end) <= activation_distance_m + _EPSILON:
                tangent = math.atan2(end.y_m - start.y_m, end.x_m - start.x_m)
                headings.extend((tangent, tangent + math.pi))
    return tuple(headings)


def _satisfies_tangent_constraints(
    heading: float, constraints: tuple[float, ...], max_error_rad: float
) -> bool:
    return all(
        min(
            abs(_wrap_angle(heading - tangent)),
            abs(_wrap_angle(heading - (tangent + math.pi))),
        ) <= max_error_rad + _EPSILON
        for tangent in constraints
    )


def _segment_is_collision_free(
    start: Point2D, end: Point2D, court: CourtModel, clearance: float
) -> bool:
    if not _point_in_eroded_polygon(start, court.navigable_polygon, clearance):
        return False
    if not _point_in_eroded_polygon(end, court.navigable_polygon, clearance):
        return False
    if _segment_crosses_polygon_boundary(start, end, court.navigable_polygon):
        return False
    if _segment_polygon_distance(start, end, court.navigable_polygon) <= clearance + _EPSILON:
        return False
    for obstacle in court.obstacles:
        if _segment_hits_inflated_polygon(start, end, obstacle.polygon, clearance):
            return False
    return True


def _point_in_eroded_polygon(point: Point2D, polygon: tuple[Point2D, ...], clearance: float) -> bool:
    return _point_in_polygon(point, polygon) and _point_polygon_distance(point, polygon) > clearance + _EPSILON


def _point_hits_inflated_polygon(point: Point2D, polygon: tuple[Point2D, ...], clearance: float) -> bool:
    return _point_in_polygon(point, polygon) or _point_polygon_distance(point, polygon) <= clearance + _EPSILON


def _segment_hits_inflated_polygon(start: Point2D, end: Point2D, polygon: tuple[Point2D, ...], clearance: float) -> bool:
    if _point_in_polygon(start, polygon) or _point_in_polygon(end, polygon):
        return True
    return _segment_crosses_polygon_boundary(start, end, polygon) or _segment_polygon_distance(start, end, polygon) <= clearance + _EPSILON


def _segment_polygon_distance(start: Point2D, end: Point2D, polygon: tuple[Point2D, ...]) -> float:
    return min(_segment_segment_distance(start, end, edge_start, edge_end) for edge_start, edge_end in _polygon_edges(polygon))


def _point_polygon_distance(point: Point2D, polygon: tuple[Point2D, ...]) -> float:
    return min(_point_segment_distance(point, start, end) for start, end in _polygon_edges(polygon))


def _polygon_edges(polygon: tuple[Point2D, ...]):
    return tuple(zip(polygon, polygon[1:] + polygon[:1]))


def _point_segment_distance(point: Point2D, start: Point2D, end: Point2D) -> float:
    dx, dy = end.x_m - start.x_m, end.y_m - start.y_m
    length_squared = dx * dx + dy * dy
    if length_squared <= _EPSILON:
        return math.hypot(point.x_m - start.x_m, point.y_m - start.y_m)
    ratio = max(0.0, min(1.0, ((point.x_m - start.x_m) * dx + (point.y_m - start.y_m) * dy) / length_squared))
    return math.hypot(point.x_m - (start.x_m + ratio * dx), point.y_m - (start.y_m + ratio * dy))


def _segment_segment_distance(first_start: Point2D, first_end: Point2D, second_start: Point2D, second_end: Point2D) -> float:
    if _segments_intersect(first_start, first_end, second_start, second_end):
        return 0.0
    return min(
        _point_segment_distance(first_start, second_start, second_end),
        _point_segment_distance(first_end, second_start, second_end),
        _point_segment_distance(second_start, first_start, first_end),
        _point_segment_distance(second_end, first_start, first_end),
    )


def _segment_crosses_polygon_boundary(start: Point2D, end: Point2D, polygon: tuple[Point2D, ...]) -> bool:
    return any(_segments_intersect(start, end, edge_start, edge_end) for edge_start, edge_end in _polygon_edges(polygon))


def _segments_intersect(first_start: Point2D, first_end: Point2D, second_start: Point2D, second_end: Point2D) -> bool:
    def orientation(a: Point2D, b: Point2D, c: Point2D) -> float:
        return (b.x_m - a.x_m) * (c.y_m - a.y_m) - (b.y_m - a.y_m) * (c.x_m - a.x_m)

    first_a = orientation(first_start, first_end, second_start)
    first_b = orientation(first_start, first_end, second_end)
    second_a = orientation(second_start, second_end, first_start)
    second_b = orientation(second_start, second_end, first_end)
    if abs(first_a) <= _EPSILON or abs(first_b) <= _EPSILON or abs(second_a) <= _EPSILON or abs(second_b) <= _EPSILON:
        return _point_segment_distance(first_start, second_start, second_end) <= _EPSILON or _point_segment_distance(first_end, second_start, second_end) <= _EPSILON or _point_segment_distance(second_start, first_start, first_end) <= _EPSILON or _point_segment_distance(second_end, first_start, first_end) <= _EPSILON
    return (first_a > 0.0) != (first_b > 0.0) and (second_a > 0.0) != (second_b > 0.0)


def _point_in_polygon(point: Point2D, polygon: tuple[Point2D, ...]) -> bool:
    inside = False
    for start, end in _polygon_edges(polygon):
        if (start.y_m > point.y_m) != (end.y_m > point.y_m):
            x_intersection = start.x_m + (point.y_m - start.y_m) * (end.x_m - start.x_m) / (end.y_m - start.y_m)
            if point.x_m < x_intersection:
                inside = not inside
    return inside


def _validate_polygon(polygon: tuple[Point2D, ...], name: str) -> None:
    if not isinstance(polygon, tuple) or len(polygon) < 3 or any(not isinstance(point, Point2D) for point in polygon):
        raise PlannerInputError(f"{name} must contain at least three Point2D vertices")
    if len({(point.x_m, point.y_m) for point in polygon}) != len(polygon):
        raise PlannerInputError(f"{name} has duplicate vertices")


def _normalize_heading(heading: float) -> float:
    return heading % (2.0 * math.pi)


def _wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi
