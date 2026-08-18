"""Turn a chosen sequence of passes and connectors into a frozen plan.

Segment construction is shared by every router, so it lives here rather than
inside one of them.  Its one non-obvious rule is ``declare_only``.

A pass may legitimately drive over a ball an earlier segment already collected:
the physical state is which balls are in the basket, not which pass owns which
ball, and refusing such a pass would invent an artificial feasibility constraint
(debug log #57beta).  But the plan contract requires every ball to appear in
exactly one segment's ``covered_ball_ids``, with crossings matching one for one.
Both hold at once by declaring only what a segment newly collects: the geometry
still runs over the collected ball, and nothing downstream expects a second
capture there -- correctly, since that ball is no longer on the court.
"""

from __future__ import annotations

from dataclasses import replace
import hashlib
import math

from tennis_robot.collection_route_types import (
    BallReasonCode,
    BallResult,
    BallStatus,
    CollectionRouteConfiguration,
    CollectionRoutePlan,
    ObstacleConstraint,
    ObstacleConstraintKind,
    Path2D,
    PathPoint,
    PlannedCrossing,
    PlanningSearchStatus,
    PlanningStatus,
    Point2D,
    Pose2D,
    RouteSegment,
    RouteSegmentType,
    ScanSnapshot,
)


def connector_execution_profile(configuration: CollectionRouteConfiguration):
    """Default profile with the connector-specific (looser) heading hard gate.

    Connectors are transit, not capture: pure-pursuit steering leads the path
    tangent on their curves, so the capture-grade heading gate would self-abort.
    Every other profile bound (speed, curvature, lateral tube) is unchanged.
    """
    return replace(
        configuration.planning.default_execution_profile,
        max_heading_error_rad=configuration.planning.connector_max_heading_error_rad,
    )


def connector_segment(segment_id, edge, progress, configuration, declare_only=None):
    """One transit segment, collecting whichever balls it is accountable for."""
    path = Path2D(tuple(PathPoint(pose) for pose in edge.path.poses))
    # Edge-local crossing progress is chord-based while the segment span uses the
    # arc length, and arc >= chord, so rebasing keeps every crossing inside the
    # segment with at least the required run-out behind it.
    crossings = tuple(
        replace(crossing, progress_s=progress + crossing.progress_s)
        for crossing in edge.swept_crossings
        if declare_only is None or crossing.ball_id in declare_only
    )
    # A connector that collects is capture motion for that stretch, so it has to
    # hold the capture-grade heading gate rather than the loose transit one. The
    # sweep detector only admits crossings on portions gentle enough to pass it.
    profile = (
        configuration.planning.default_execution_profile
        if crossings
        else connector_execution_profile(configuration)
    )
    return RouteSegment(
        segment_id, RouteSegmentType.CONNECTOR, path, progress, progress + edge.path.length_m,
        profile, tuple(crossing.ball_id for crossing in crossings),
        ObstacleConstraint(ObstacleConstraintKind.NONE, (), 0.0), crossings,
    )


def pass_segment(segment_id, candidate, progress, configuration, declare_only=None):
    """One straight funnel pass, declaring only the balls it newly collects."""
    length = pass_length(candidate)
    direction = (math.cos(candidate.heading_rad), math.sin(candidate.heading_rad))
    normal = (-direction[1], direction[0])
    crossings = []
    for ball_id, position in zip(candidate.covered_ball_ids, candidate.crossing_positions):
        if declare_only is not None and ball_id not in declare_only:
            # Driven over, already in the basket: geometry keeps it, the plan
            # does not claim it a second time.
            continue
        dx = position.x_m - candidate.entry_pose.x_m
        dy = position.y_m - candidate.entry_pose.y_m
        along = dx * direction[0] + dy * direction[1]
        lateral = dx * normal[0] + dy * normal[1]
        centreline_position = Point2D(
            candidate.entry_pose.x_m + along * direction[0],
            candidate.entry_pose.y_m + along * direction[1],
        )
        crossings.append(
            PlannedCrossing(ball_id, centreline_position, progress + along, candidate.heading_rad, lateral)
        )
    crossings = tuple(crossings)
    path = Path2D(
        (PathPoint(candidate.entry_pose),)
        + tuple(
            PathPoint(Pose2D(item.position_xy.x_m, item.position_xy.y_m, item.heading_rad))
            for item in crossings
        )
        + (PathPoint(candidate.exit_pose),)
    )
    return RouteSegment(
        segment_id, RouteSegmentType.FUNNEL_PASS, path, progress, progress + length,
        configuration.planning.default_execution_profile,
        tuple(crossing.ball_id for crossing in crossings),
        ObstacleConstraint(ObstacleConstraintKind.NONE, (), 0.0), crossings,
    )


def transit_pass_segment(segment_id, candidate, progress, configuration):
    """Pass geometry driven as pure transit, because it collects nothing new.

    Reaching a pass whose balls are all already in the basket is occasionally
    the right move -- the connector into it may be what sweeps the ball we are
    actually after.  The geometry is still driven and still costed; it is simply
    not a capture segment, so it carries the transit heading gate and declares
    nothing.
    """
    return RouteSegment(
        segment_id, RouteSegmentType.CONNECTOR,
        Path2D((PathPoint(candidate.entry_pose), PathPoint(candidate.exit_pose))),
        progress, progress + pass_length(candidate),
        connector_execution_profile(configuration), (),
        ObstacleConstraint(ObstacleConstraintKind.NONE, (), 0.0),
    )


def terminal_segment(segment_id, exit_pose, terminal_pose, progress, length, configuration):
    return RouteSegment(
        segment_id, RouteSegmentType.TERMINAL_CONNECTOR,
        Path2D((PathPoint(exit_pose), PathPoint(terminal_pose))),
        progress, progress + length,
        configuration.planning.default_execution_profile, (),
        ObstacleConstraint(ObstacleConstraintKind.NONE, (), 0.0),
    )


def pass_length(candidate) -> float:
    return (
        math.hypot(
            candidate.crossing.x_m - candidate.entry_pose.x_m,
            candidate.crossing.y_m - candidate.entry_pose.y_m,
        )
        + math.hypot(
            candidate.exit_pose.x_m - candidate.crossing.x_m,
            candidate.exit_pose.y_m - candidate.crossing.y_m,
        )
    )


def plan_id_for(segments: tuple[RouteSegment, ...], scan_id: str) -> str:
    """Identity from the full realised geometry, not from a coarse summary.

    Two routes over the same balls in the same order can differ in heading,
    entry pose or connector mode; a seed that omits those collides, and the
    executor treats plan_id as identity.
    """
    seed = "|".join(
        f"{segment.id}:{segment.type.value}:"
        f"{segment.progress_start_m:.6f}:{segment.progress_end_m:.6f}:"
        + ",".join(segment.covered_ball_ids)
        + ";"
        + ",".join(
            f"{point.pose.x_m:.6f},{point.pose.y_m:.6f},{point.pose.yaw_rad:.6f}"
            for point in segment.path.points
        )
        for segment in segments
    )
    return "route-" + hashlib.sha256((scan_id + "|" + seed).encode()).hexdigest()[:16]


def assemble_plan(
    *,
    snapshot: ScanSnapshot,
    configuration: CollectionRouteConfiguration,
    segments: tuple[RouteSegment, ...],
    terminal_pose: Pose2D,
    ball_results: tuple[BallResult, ...],
    search_status: PlanningSearchStatus,
) -> CollectionRoutePlan:
    all_ball_ids = tuple(ball.ball_id for ball in snapshot.balls)
    covered = {result.ball_id for result in ball_results if result.status is BallStatus.COVERED}
    total_length = segments[-1].progress_end_m
    search = configuration.global_route_search
    crossing_length = sum(
        segment.progress_end_m - segment.progress_start_m
        for segment in segments
        if segment.type is RouteSegmentType.FUNNEL_PASS
    )
    duration = (
        crossing_length / search.crossing_nominal_speed_m_s
        + (total_length - crossing_length) / search.connector_nominal_speed_m_s
    )
    status = PlanningStatus.FEASIBLE if covered == set(all_ball_ids) else PlanningStatus.PARTIAL
    return CollectionRoutePlan(
        plan_id_for(segments, snapshot.scan_id), snapshot.scan_id, snapshot.map_frame,
        snapshot.robot_pose_at_scan, terminal_pose, total_length, duration,
        status, search_status, segments, all_ball_ids, ball_results, configuration,
    )


def empty_plan(snapshot, configuration, status, search_status, results) -> CollectionRoutePlan:
    return CollectionRoutePlan(
        "route-empty-" + snapshot.scan_id, snapshot.scan_id, snapshot.map_frame,
        snapshot.robot_pose_at_scan, snapshot.robot_pose_at_scan, 0.0, 0.0,
        status, search_status, (), tuple(ball.ball_id for ball in snapshot.balls),
        results, configuration,
    )


def selected_result(ball_id: str, segment_id: str) -> BallResult:
    return BallResult(ball_id, BallStatus.COVERED, BallReasonCode.SELECTED, segment_id)
