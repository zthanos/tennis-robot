"""Pure Phase 3C multi-ball pass generation by line enumeration.

Candidates used to be built per ball from a 16-heading grid and then merged by
*exact* heading equality.  That could only ever combine balls whose sampled
headings matched bit for bit, so three nearly collinear balls never landed in
one group however wide the capture corridor got: the measured cover ceiling for
a real scan was four passes while the planner produced eight.

This module inverts the construction.  It enumerates the lines the balls
themselves define, keeps the maximal ones, fits each group's line, and emits a
directed pass per travel direction.  Choosing *which* lines to use is not done
here -- fewer passes is not the same as a shorter route, so the covering subset
is chosen by the router against the full route cost, connectors included.
"""

from __future__ import annotations

from dataclasses import dataclass
from itertools import combinations
import math

from tennis_robot.collection_route_planner_v2 import (
    CourtModel,
    FunnelPassCandidate,
    _active_tangent_headings,
    _effective_capture_half_width,
    _satisfies_tangent_constraints,
    _segment_is_collision_free,
)
from tennis_robot.collection_route_types import (
    CollectionRouteConfiguration,
    Point2D,
    Pose2D,
    ScanSnapshot,
)

_EPSILON = 1e-9


@dataclass(frozen=True)
class SharedPassGenerationResult:
    candidates: tuple[FunnelPassCandidate, ...]
    candidate_budget_exhausted: bool


def generate_shared_passes(*, snapshot: ScanSnapshot, single_ball_candidates: tuple[FunnelPassCandidate, ...], court: CourtModel, configuration: CollectionRouteConfiguration) -> SharedPassGenerationResult:
    if not isinstance(snapshot, ScanSnapshot):
        raise ValueError("snapshot must be ScanSnapshot")
    if not isinstance(single_ball_candidates, tuple) or any(not isinstance(candidate, FunnelPassCandidate) for candidate in single_ball_candidates):
        raise ValueError("single_ball_candidates must be FunnelPassCandidate tuple")
    if not isinstance(court, CourtModel) or not isinstance(configuration, CollectionRouteConfiguration):
        raise ValueError("court and configuration are required")

    # Boundary-recovery passes deliberately put one ball on the outer funnel
    # cheek.  They stay isolated contact experiments and never join a group.
    eligible = {
        candidate.covered_ball_ids[0]
        for candidate in single_ball_candidates
        if len(candidate.covered_ball_ids) == 1 and not candidate.boundary_recovery
    }
    balls = tuple(ball for ball in snapshot.balls if ball.ball_id in eligible)
    if len(balls) < 2:
        return SharedPassGenerationResult((), False)

    shared = configuration.shared_pass
    generated: list[FunnelPassCandidate] = []
    exhausted = False
    for group in _maximal_groups(balls, configuration):
        for window in _bounded_windows(group, shared.max_shared_pass_balls):
            for candidate in _line_candidates(window, court, configuration):
                if len(generated) >= shared.max_shared_pass_candidates:
                    exhausted = True
                    continue
                generated.append(candidate)
    return SharedPassGenerationResult(tuple(generated), exhausted)


def _maximal_groups(balls, configuration) -> tuple[tuple, ...]:
    """Every maximal set of balls that one straight corridor can hold.

    Each ball pair defines a line; the balls close enough to it form a group.
    Groups contained in a larger one are dropped, so what is left is the set of
    genuinely distinct passes available on this scan.
    """
    groups: set[frozenset[str]] = set()
    by_id = {ball.ball_id: ball for ball in balls}
    for first, second in combinations(balls, 2):
        delta_x = second.position.x_m - first.position.x_m
        delta_y = second.position.y_m - first.position.y_m
        if math.hypot(delta_x, delta_y) <= _EPSILON:
            continue
        heading = math.atan2(delta_y, delta_x)
        members = _members_within_corridor(balls, first.position, heading, configuration)
        if len(members) >= 2:
            groups.add(frozenset(members))
    maximal = [group for group in groups if not any(group < other for other in groups)]
    ordered = sorted(maximal, key=lambda group: (-len(group), sorted(group)))
    return tuple(tuple(by_id[ball_id] for ball_id in sorted(group)) for group in ordered)


def _members_within_corridor(balls, origin: Point2D, heading: float, configuration):
    normal_x, normal_y = -math.sin(heading), math.cos(heading)
    members = []
    for ball in balls:
        offset = (ball.position.x_m - origin.x_m) * normal_x + (ball.position.y_m - origin.y_m) * normal_y
        if abs(offset) <= _effective_capture_half_width(ball, heading, configuration):
            members.append(ball.ball_id)
    return members


def _bounded_windows(group, maximum: int):
    """Contiguous slices of a group that fit the mechanical per-pass limit.

    A ball between two others on the same line is driven over anyway, so only
    contiguous windows make sense; skipping a middle ball would plan to pass
    through it without collecting it.
    """
    if len(group) <= maximum:
        return (group,)
    ordered = _ordered_along_fit(group)
    return tuple(ordered[index:index + maximum] for index in range(len(ordered) - maximum + 1))


def _fit_heading(group) -> float:
    """Principal axis of the group's positions."""
    count = len(group)
    mean_x = sum(ball.position.x_m for ball in group) / count
    mean_y = sum(ball.position.y_m for ball in group) / count
    xx = sum((ball.position.x_m - mean_x) ** 2 for ball in group)
    xy = sum((ball.position.x_m - mean_x) * (ball.position.y_m - mean_y) for ball in group)
    yy = sum((ball.position.y_m - mean_y) ** 2 for ball in group)
    return 0.5 * math.atan2(2.0 * xy, xx - yy)


def _ordered_along_fit(group):
    heading = _fit_heading(group)
    direction = (math.cos(heading), math.sin(heading))
    return tuple(sorted(group, key=lambda ball: ball.position.x_m * direction[0] + ball.position.y_m * direction[1]))


def _line_candidates(group, court, configuration):
    """Both travel directions along the group's fitted line, when both work.

    The two directions are genuinely different passes: they put the run-in and
    run-out on opposite ends, which changes what the connectors either side have
    to do, so the choice belongs to the router rather than here.
    """
    base = _fit_heading(group)
    built = []
    for heading in (_normalize(base), _normalize(base + math.pi)):
        candidate = _build_line_candidate(group, heading, court, configuration)
        if candidate is not None:
            built.append(candidate)
    return tuple(built)


def _build_line_candidate(group, heading: float, court, configuration):
    mechanical = configuration.mechanical
    feasibility = configuration.feasibility
    clearance = feasibility.footprint_clearance_radius_m
    direction = (math.cos(heading), math.sin(heading))
    normal = (-direction[1], direction[0])

    ordered = tuple(sorted(group, key=lambda ball: ball.position.x_m * direction[0] + ball.position.y_m * direction[1]))
    lateral = [ball.position.x_m * normal[0] + ball.position.y_m * normal[1] for ball in ordered]
    centre_lateral = (min(lateral) + max(lateral)) / 2.0
    widths = []
    for ball, offset in zip(ordered, lateral):
        width = _effective_capture_half_width(ball, heading, configuration)
        # The fitted line is not the pair line the group was found with, so
        # membership is re-checked against it before anything is emitted.
        if width <= 0.0 or abs(offset - centre_lateral) > width:
            return None
        widths.append(width)
        # A ball near the net or fence may only be approached parallel to it.
        tangents = _active_tangent_headings(ball.position, court, feasibility.tangent_activation_distance_m)
        if tangents and not _satisfies_tangent_constraints(heading, tangents, feasibility.max_parallel_heading_error_rad):
            return None

    longitudinal = [ball.position.x_m * direction[0] + ball.position.y_m * direction[1] for ball in ordered]
    if any(later - earlier < configuration.shared_pass.minimum_mechanical_ball_spacing_m for earlier, later in zip(longitudinal, longitudinal[1:])):
        return None

    entry = _point(longitudinal[0] - mechanical.minimum_run_in_m, centre_lateral, direction, normal)
    exit_point = _point(longitudinal[-1] + mechanical.minimum_run_out_m, centre_lateral, direction, normal)
    if not _segment_is_collision_free(entry, exit_point, court, clearance):
        return None

    covered = tuple(ball.ball_id for ball in ordered)
    return FunnelPassCandidate(
        "line:" + "+".join(covered),
        covered,
        heading,
        Pose2D(entry.x_m, entry.y_m, heading),
        _point(longitudinal[0], centre_lateral, direction, normal),
        Pose2D(exit_point.x_m, exit_point.y_m, heading),
        min(widths),
        # Real ball positions, so the executor's predicted lateral error is the
        # true offset from the centreline rather than a projected zero.
        tuple(Point2D(ball.position.x_m, ball.position.y_m) for ball in ordered),
    )


def _point(longitudinal: float, lateral: float, direction, normal) -> Point2D:
    return Point2D(
        longitudinal * direction[0] + lateral * normal[0],
        longitudinal * direction[1] + lateral * normal[1],
    )


def _normalize(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))
