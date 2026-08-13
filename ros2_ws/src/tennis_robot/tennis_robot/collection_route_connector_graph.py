"""Pure Phase 3B1 CSC Dubins connector graph.

Only forward simple CSC connectors are represented here.  This module neither
orders passes nor chooses a route.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math

from tennis_robot.collection_route_planner_v2 import (
    CourtModel,
    FunnelPassCandidate,
    _effective_capture_half_width,
    _segment_is_collision_free,
)
from tennis_robot.collection_route_types import (
    CollectionRouteConfiguration,
    PlannedCrossing,
    Point2D,
    Pose2D,
    ScanSnapshot,
)


_CSC_MODES = ("LSL", "RSR", "LSR", "RSL")
_ARC_CHORD_ANGLE_RAD = math.pi / 12.0
_EPSILON = 1e-9


class ConnectorRejectionCode(str, Enum):
    TURNING_CONSTRAINT_REJECTED = "turning_constraint_rejected"
    LENGTH_REJECTED = "length_rejected"
    COLLISION_REJECTED = "collision_rejected"


@dataclass(frozen=True)
class ConnectorPrimitive:
    kind: str
    length_m: float
    arc_angle_rad: float


@dataclass(frozen=True)
class ConnectorPath:
    mode: str
    primitives: tuple[ConnectorPrimitive, ...]
    poses: tuple[Pose2D, ...]
    length_m: float
    total_turn_rad: float


@dataclass(frozen=True)
class ConnectorEdge:
    edge_id: str
    source_node_id: str
    target_node_id: str
    path: ConnectorPath | None
    maximum_curvature_per_m: float | None
    collision_free: bool
    rejection: ConnectorRejectionCode | None
    # Balls the connector itself sweeps through the funnel on a portion gentle
    # enough to capture on.  Transit motion that collects is what lets the search
    # drop a dedicated straight pass for those balls.
    swept_crossings: tuple[PlannedCrossing, ...] = ()

    def __post_init__(self) -> None:
        if self.rejection is None and (self.path is None or not self.collision_free):
            raise ValueError("accepted edge must have collision-free path")
        if self.rejection is not None and self.collision_free:
            raise ValueError("rejected edge cannot report collision-free")
        if not isinstance(self.swept_crossings, tuple) or any(
            not isinstance(item, PlannedCrossing) for item in self.swept_crossings
        ):
            raise ValueError("swept_crossings must be a tuple of PlannedCrossing")
        if self.rejection is not None and self.swept_crossings:
            raise ValueError("rejected edge cannot sweep balls")
        progresses = [crossing.progress_s for crossing in self.swept_crossings]
        if any(later <= earlier for earlier, later in zip(progresses, progresses[1:])):
            raise ValueError("swept crossings must be strictly increasing in progress")

    @property
    def swept_ball_ids(self) -> tuple[str, ...]:
        return tuple(crossing.ball_id for crossing in self.swept_crossings)


@dataclass(frozen=True)
class DirectedCandidateGraph:
    pass_nodes: tuple[tuple[str, FunnelPassCandidate], ...]
    edges: tuple[ConnectorEdge, ...]


def build_directed_candidate_graph(
    *,
    snapshot: ScanSnapshot,
    candidates: tuple[FunnelPassCandidate, ...],
    court: CourtModel,
    configuration: CollectionRouteConfiguration,
) -> DirectedCandidateGraph:
    """Build start→entry and exit→other-entry CSC edges for Phase 3B1."""
    if not isinstance(snapshot, ScanSnapshot):
        raise ValueError("snapshot must be ScanSnapshot")
    if not isinstance(court, CourtModel):
        raise ValueError("court must be CourtModel")
    if not isinstance(configuration, CollectionRouteConfiguration):
        raise ValueError("configuration must be CollectionRouteConfiguration")
    if snapshot.configuration_snapshot != configuration:
        raise ValueError("configuration must exactly match snapshot configuration")
    if not isinstance(candidates, tuple) or any(not isinstance(item, FunnelPassCandidate) for item in candidates):
        raise ValueError("candidates must be tuple of FunnelPassCandidate")

    ordered_candidates = tuple(sorted(candidates, key=lambda item: (item.ball_id, item.heading_rad, item.entry_pose.x_m, item.entry_pose.y_m, item.exit_pose.x_m, item.exit_pose.y_m, item.covered_ball_ids)))
    pass_nodes = tuple((f"pass:{candidate.ball_id}:{index}", candidate) for index, candidate in enumerate(ordered_candidates))
    edges: list[ConnectorEdge] = []
    # Only balls that survived per-ball feasibility may be swept in transit. A
    # ball with no pass candidate is keepout or otherwise unreachable, and
    # driving a connector through it is precisely what must not happen.
    eligible = {ball_id for candidate in ordered_candidates for ball_id in candidate.covered_ball_ids}
    balls = tuple(ball for ball in snapshot.balls if ball.ball_id in eligible)
    for node_id, candidate in pass_nodes:
        edges.extend(_connector_edges("start", snapshot.robot_pose_at_scan, node_id, candidate.entry_pose, court, configuration, balls))
    for source_id, source in pass_nodes:
        for target_id, target in pass_nodes:
            if source_id == target_id:
                continue
            edges.extend(_connector_edges(source_id, source.exit_pose, target_id, target.entry_pose, court, configuration, balls))
    return DirectedCandidateGraph(pass_nodes, tuple(edges))


def _path_intervals(poses):
    """(start_progress, length, local_turn_radius) per densified path interval."""
    intervals = []
    progress = 0.0
    for first, second in zip(poses, poses[1:]):
        length = math.hypot(second.x_m - first.x_m, second.y_m - first.y_m)
        if length <= _EPSILON:
            continue
        turn = abs(_wrap_angle(second.yaw_rad - first.yaw_rad))
        radius = length / turn if turn > _EPSILON else math.inf
        intervals.append((progress, length, radius, first, second))
        progress += length
    return intervals, progress


def _window_is_capture_grade(intervals, start_m, end_m, minimum_radius):
    """Is every interval overlapping [start_m, end_m] gentle enough to capture on?"""
    if start_m < -_EPSILON or end_m <= start_m:
        return False
    covered_to = start_m
    for begin, length, radius, _, _ in intervals:
        finish = begin + length
        if finish <= start_m + _EPSILON or begin >= end_m - _EPSILON:
            continue
        if radius + _EPSILON < minimum_radius:
            return False
        covered_to = max(covered_to, finish)
    return covered_to + _EPSILON >= end_m


def swept_crossings_for_path(path, balls, configuration):
    """Balls the funnel would swallow while the robot drives this connector.

    A ball qualifies only where the connector is gentle enough that the
    pure-pursuit heading lead stays inside the capture gate, and only when the
    whole run-in/run-out window around it is equally gentle and still inside
    this connector -- the tracking core requires the run-out to fit within the
    segment that owns the crossing.
    """
    mechanical = configuration.mechanical
    minimum_radius = configuration.connector.capture_minimum_turn_radius_m
    run_in, run_out = mechanical.minimum_run_in_m, mechanical.minimum_run_out_m
    intervals, total = _path_intervals(path.poses)
    if not intervals:
        return ()

    found: dict[str, tuple[float, PlannedCrossing]] = {}
    for begin, length, radius, first, second in intervals:
        if radius + _EPSILON < minimum_radius:
            continue
        dx, dy = second.x_m - first.x_m, second.y_m - first.y_m
        squared = dx * dx + dy * dy
        if squared <= _EPSILON:
            continue
        for ball in balls:
            offset_x = ball.position.x_m - first.x_m
            offset_y = ball.position.y_m - first.y_m
            ratio = (offset_x * dx + offset_y * dy) / squared
            if ratio < 0.0 or ratio > 1.0:
                continue
            lateral = abs(offset_x * (-dy) + offset_y * dx) / math.sqrt(squared)
            heading = first.yaw_rad + ratio * _wrap_angle(second.yaw_rad - first.yaw_rad)
            if lateral > _effective_capture_half_width(ball, heading, configuration):
                continue
            progress = begin + ratio * length
            if not _window_is_capture_grade(
                intervals, progress - run_in, min(progress + run_out, total), minimum_radius
            ):
                continue
            if progress - run_in < -_EPSILON or total - progress + _EPSILON < run_out:
                continue
            existing = found.get(ball.ball_id)
            if existing is None or lateral < existing[0]:
                found[ball.ball_id] = (
                    lateral,
                    PlannedCrossing(
                        ball.ball_id,
                        Point2D(ball.position.x_m, ball.position.y_m),
                        progress,
                        _wrap_angle(heading),
                        lateral,
                    ),
                )
    ordered = sorted((item[1] for item in found.values()), key=lambda crossing: crossing.progress_s)
    # The tracking core rejects non-increasing crossing progress outright.
    deduped: list[PlannedCrossing] = []
    for crossing in ordered:
        if deduped and crossing.progress_s <= deduped[-1].progress_s + _EPSILON:
            continue
        deduped.append(crossing)
    return tuple(deduped)


def _wrap_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def _connector_edges(source_id, source_pose, target_id, target_pose, court, configuration, balls=()):
    minimum = configuration.mechanical.minimum_turning_radius_m
    return tuple(
        _build_edge(source_id, source_pose, target_id, target_pose, mode, minimum * multiplier, multiplier, court, configuration, balls)
        for multiplier in configuration.connector.sweep_radius_multipliers
        for mode in _CSC_MODES
    )


def _build_edge(source_id, source, target_id, target, mode, radius, multiplier, court, configuration, balls=()):
    # The tight minimum-radius geometry keeps its original id so a single-radius
    # configuration produces a byte-identical graph.
    edge_id = f"{source_id}->{target_id}:{mode}" if multiplier == 1.0 else f"{source_id}->{target_id}:{mode}:x{multiplier:g}"
    normalized = _dubins_parameters(source, target, radius, mode)
    if normalized is None:
        return ConnectorEdge(edge_id, source_id, target_id, None, None, False, ConnectorRejectionCode.TURNING_CONSTRAINT_REJECTED)
    connector = configuration.connector
    # Analytic turning/length gate on the closed-form CSC parameters, applied
    # before the expensive path materialization.  For a CSC path the two arc
    # angles are the normalized turn lengths and the straight contributes no
    # turn, so length/arc/total-turn are known without densifying the geometry.
    # ~98% of directed pairs fail here; only survivors are materialized and
    # collision-checked.  length_m/total_turn_rad mirror _materialize_path's
    # summation order so the accept/reject verdict is byte-identical.
    first_arc, straight, second_arc = normalized
    length_m = first_arc * radius + straight * radius + second_arc * radius
    total_turn_rad = first_arc + second_arc
    if (
        length_m > connector.max_connector_length_m + _EPSILON
        or first_arc > connector.max_connector_arc_angle_rad + _EPSILON
        or second_arc > connector.max_connector_arc_angle_rad + _EPSILON
        or total_turn_rad > connector.max_connector_total_turn_rad + _EPSILON
    ):
        return ConnectorEdge(edge_id, source_id, target_id, None, None, False, ConnectorRejectionCode.TURNING_CONSTRAINT_REJECTED if length_m <= connector.max_connector_length_m + _EPSILON else ConnectorRejectionCode.LENGTH_REJECTED)
    path = _materialize_path(source, radius, mode, normalized)
    if _self_intersects(path.poses):
        return ConnectorEdge(edge_id, source_id, target_id, None, None, False, ConnectorRejectionCode.TURNING_CONSTRAINT_REJECTED)
    if not _path_is_collision_free(path, court, configuration.feasibility.footprint_clearance_radius_m, radius):
        return ConnectorEdge(edge_id, source_id, target_id, path, 1.0 / radius, False, ConnectorRejectionCode.COLLISION_REJECTED)
    return ConnectorEdge(edge_id, source_id, target_id, path, 1.0 / radius, True, None, swept_crossings_for_path(path, balls, configuration))


def _dubins_parameters(start: Pose2D, target: Pose2D, radius: float, mode: str):
    dx, dy = target.x_m - start.x_m, target.y_m - start.y_m
    distance = math.hypot(dx, dy) / radius
    theta = math.atan2(dy, dx) if distance > _EPSILON else 0.0
    alpha = _mod2pi(start.yaw_rad - theta)
    beta = _mod2pi(target.yaw_rad - theta)
    if mode == "LSL":
        p2 = 2 + distance * distance - 2 * math.cos(alpha - beta) + 2 * distance * (math.sin(alpha) - math.sin(beta))
        if p2 < 0: return None
        tmp = math.atan2(math.cos(beta) - math.cos(alpha), distance + math.sin(alpha) - math.sin(beta))
        return _mod2pi(-alpha + tmp), math.sqrt(p2), _mod2pi(beta - tmp)
    if mode == "RSR":
        p2 = 2 + distance * distance - 2 * math.cos(alpha - beta) + 2 * distance * (math.sin(beta) - math.sin(alpha))
        if p2 < 0: return None
        tmp = math.atan2(math.cos(alpha) - math.cos(beta), distance - math.sin(alpha) + math.sin(beta))
        return _mod2pi(alpha - tmp), math.sqrt(p2), _mod2pi(-beta + tmp)
    if mode == "LSR":
        p2 = -2 + distance * distance + 2 * math.cos(alpha - beta) + 2 * distance * (math.sin(alpha) + math.sin(beta))
        if p2 < 0: return None
        p = math.sqrt(p2)
        tmp = math.atan2(-math.cos(alpha) - math.cos(beta), distance + math.sin(alpha) + math.sin(beta)) - math.atan2(-2.0, p)
        return _mod2pi(-alpha + tmp), p, _mod2pi(-_mod2pi(beta) + tmp)
    if mode == "RSL":
        p2 = distance * distance - 2 + 2 * math.cos(alpha - beta) - 2 * distance * (math.sin(alpha) + math.sin(beta))
        if p2 < 0: return None
        p = math.sqrt(p2)
        tmp = math.atan2(math.cos(alpha) + math.cos(beta), distance - math.sin(alpha) - math.sin(beta)) - math.atan2(2.0, p)
        return _mod2pi(alpha - tmp), p, _mod2pi(beta - tmp)
    raise ValueError("only CSC Dubins modes are permitted")


def _materialize_path(start, radius, mode, normalized_lengths):
    kinds = tuple(mode)
    primitives = tuple(
        ConnectorPrimitive(kind, length * radius, 0.0 if kind == "S" else length)
        for kind, length in zip(kinds, normalized_lengths)
    )
    # Densify arc primitives into ≤ _ARC_CHORD_ANGLE_RAD sub-arcs so the chord
    # polyline of ``poses`` tracks the true arc length (a sparse two-pose arc has
    # a chord far shorter than its arc, which the C++ make_tracking_plan rejects
    # against progress).  This is the SAME granularity as _path_is_collision_free
    # and touches only the pose sampling — primitives, length_m, arc_angle_rad and
    # total_turn_rad stay arc-based, so scoring/cost/progress spans are unchanged.
    poses = [start]
    current = start
    for primitive in primitives:
        subdivisions = 1 if primitive.kind == "S" else max(1, math.ceil(primitive.arc_angle_rad / _ARC_CHORD_ANGLE_RAD))
        piece = ConnectorPrimitive(
            primitive.kind,
            primitive.length_m / subdivisions,
            0.0 if primitive.kind == "S" else primitive.arc_angle_rad / subdivisions,
        )
        for _ in range(subdivisions):
            current = _advance(current, piece, radius)
            poses.append(current)
    return ConnectorPath(mode, primitives, tuple(poses), sum(item.length_m for item in primitives), sum(item.arc_angle_rad for item in primitives))


def _advance(pose, primitive, radius):
    if primitive.kind == "S":
        return Pose2D(pose.x_m + primitive.length_m * math.cos(pose.yaw_rad), pose.y_m + primitive.length_m * math.sin(pose.yaw_rad), pose.yaw_rad)
    sign = 1.0 if primitive.kind == "L" else -1.0
    angle = primitive.arc_angle_rad
    next_yaw = pose.yaw_rad + sign * angle
    return Pose2D(
        pose.x_m + sign * radius * (math.sin(next_yaw) - math.sin(pose.yaw_rad)),
        pose.y_m - sign * radius * (math.cos(next_yaw) - math.cos(pose.yaw_rad)),
        next_yaw,
    )


def _path_is_collision_free(path, court, clearance, radius):
    current = path.poses[0]
    for primitive in path.primitives:
        subdivisions = 1 if primitive.kind == "S" else max(1, math.ceil(primitive.arc_angle_rad / _ARC_CHORD_ANGLE_RAD))
        delta = primitive.length_m / subdivisions
        piece = ConnectorPrimitive(primitive.kind, delta, 0.0 if primitive.kind == "S" else primitive.arc_angle_rad / subdivisions)
        sagitta = 0.0 if primitive.kind == "S" else radius * (1.0 - math.cos(piece.arc_angle_rad / 2.0))
        for _ in range(subdivisions):
            next_pose = _advance(current, piece, radius)
            if not _segment_is_collision_free(
                _point(current), _point(next_pose), court, clearance + sagitta
            ):
                return False
            current = next_pose
    return True


def _self_intersects(poses):
    # CSC limits make each primitive simple; this catches chord-level loops.
    segments = list(zip(poses, poses[1:]))
    for index, (first_start, first_end) in enumerate(segments):
        for second_index, (second_start, second_end) in enumerate(segments[index + 2:], start=index + 2):
            if second_index == index + 1:
                continue
            if _segments_intersect(first_start, first_end, second_start, second_end):
                return True
    return False


def _segments_intersect(a, b, c, d):
    def cross(first, second, third):
        return (second.x_m - first.x_m) * (third.y_m - first.y_m) - (second.y_m - first.y_m) * (third.x_m - first.x_m)
    ab_c, ab_d, cd_a, cd_b = cross(a, b, c), cross(a, b, d), cross(c, d, a), cross(c, d, b)
    return (ab_c > _EPSILON) != (ab_d > _EPSILON) and (cd_a > _EPSILON) != (cd_b > _EPSILON)


def _point(pose):
    from tennis_robot.collection_route_types import Point2D
    return Point2D(pose.x_m, pose.y_m)


def _mod2pi(value):
    return value % (2.0 * math.pi)
