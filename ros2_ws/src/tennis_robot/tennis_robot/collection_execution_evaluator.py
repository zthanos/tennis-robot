"""Did the robot drive what the planner planned, and did it collect?

Pure, offline, and deliberately unwilling to guess.  Given the frozen snapshot,
the frozen plan and a recorded execution trace, this answers per ball:

* where the planner believed the ball was;
* which segment was meant to collect it;
* where the robot actually drove;
* whether the *executed* collector mouth swept the ball;
* whether a collection was confirmed;
* whether the ball had already been pushed somewhere else;
* and, when it failed, which subsystem the evidence points at.

Two rules shape everything here.

*A missed ball is not a diagnosis.*  Every planned collection ends in an
explicit outcome backed by measurements, and ``OBSERVATION_UNCERTAIN`` is a
real answer -- preferable to attributing a mechanical failure on evidence that
cannot support it.

*Disappearance is not collection.*  Perception losing sight of a ball proves
nothing; only a confirmed beam sequence does.  The two are recorded separately
and never merged.

Geometry note.  The planner reasons with a conservative capture corridor
(``mechanical.capture_half_width_m`` 0.17 m, reduced further by uncertainty and
margins) while the physical funnel mouth is 0.205 m half-width
(``collection_capture_geometry``).  This evaluator measures against the
*physical* mouth, because the question here is what the machine did, not what
the planner allowed itself to assume.  Both numbers are reported so the
difference stays visible.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math

from tennis_robot.collection_capture_geometry import (
    CaptureGeometry,
    INTAKE_MOUTH_PLANE_ID,
)
from tennis_robot.collection_execution_trace import ExecutionTrace
from tennis_robot.collection_route_types import (
    CollectionRoutePlan,
    Point2D,
    RouteSegmentType,
    ScanSnapshot,
)


class CrossingOutcome(str, Enum):
    """What became of one planned collection, on evidence."""

    PLANNED_AND_EXECUTED_COLLECTED = "planned_and_executed_collected"
    PLANNED_BUT_TRACKING_MISSED = "planned_but_tracking_missed"
    EXECUTED_CROSSING_NOT_COLLECTED = "executed_crossing_not_collected"
    BALL_DISPLACED_BEFORE_ATTEMPT = "ball_displaced_before_attempt"
    OBSERVATION_UNCERTAIN = "observation_uncertain"
    COLLECTED_BY_DIFFERENT_SEGMENT = "collected_by_different_segment"
    # The robot attributed a collection to this ball, but the collector mouth
    # reconstructed from the executed trajectory never swept the ball's believed
    # position.  Both statements are kept: one is the machine's own record, the
    # other is geometry computed against a *believed* ball position carrying the
    # snapshot's positional uncertainty.  Surfacing the disagreement is the
    # point -- silently trusting either one would hide a real inconsistency.
    CONFIRMED_WITHOUT_RECONSTRUCTED_CROSSING = "confirmed_without_reconstructed_crossing"
    OTHER = "other"


@dataclass(frozen=True)
class SweptCrossing:
    """The closest the collector mouth came to a ball, and whether it swept it."""

    crossed: bool
    minimum_clearance_m: float
    lateral_offset_m: float | None
    t_s: float | None
    segment_id: str | None
    speed_mps: float | None

    @property
    def observed(self) -> bool:
        """Did the trace cover this ball at all?

        Deliberately not the same as ``crossed``: a clean miss is an observation
        (we know where the robot went and it went past), whereas a trace with no
        usable samples tells us nothing and must not be read as a miss.
        """
        return math.isfinite(self.minimum_clearance_m)


@dataclass(frozen=True)
class SegmentTracking:
    """How closely one executed segment followed its planned geometry."""

    segment_id: str
    segment_type: str
    samples: int
    max_cross_track_m: float
    rms_cross_track_m: float
    mean_cross_track_m: float
    max_heading_error_rad: float
    rms_heading_error_rad: float
    endpoint_position_error_m: float
    endpoint_heading_error_rad: float
    planned_length_m: float
    executed_length_m: float
    duration_s: float


@dataclass(frozen=True)
class BallOutcome:
    """One planned ball, from the planner's belief to the physical result."""

    ball_id: str
    outcome: CrossingOutcome
    planning_position: Point2D
    latest_observed_position: Point2D | None
    displacement_m: float | None
    intended_segment_id: str | None
    intended_segment_type: str | None
    planned_crossing: bool
    executed: SweptCrossing
    confirmed: bool
    confirmation_t_s: float | None
    confirmation_latency_s: float | None
    crossing_lateral_error_m: float | None
    crossing_speed_mps: float | None
    detail: str = ""


@dataclass(frozen=True)
class DisturbanceEvent:
    """A close approach to a ball that was not the target at the time."""

    ball_id: str
    segment_id: str | None
    t_s: float
    body_clearance_m: float
    mouth_clearance_m: float
    speed_mps: float
    displacement_m: float | None
    time_to_intended_attempt_s: float | None


@dataclass(frozen=True)
class ExecutionEvaluation:
    run_id: str
    plan_id: str
    outcomes: tuple[BallOutcome, ...]
    tracking: tuple[SegmentTracking, ...]
    disturbances: tuple[DisturbanceEvent, ...]
    telemetry_rows: int

    @property
    def inconsistencies(self) -> tuple["BallOutcome", ...]:
        """Balls where the machine and the reconstruction disagree."""
        return tuple(
            item for item in self.outcomes
            if item.outcome is CrossingOutcome.CONFIRMED_WITHOUT_RECONSTRUCTED_CROSSING
        )

    def matrix(self) -> dict[tuple[str, str, str], int]:
        """Counts of (planned crossing, executed crossing, confirmed)."""
        counts: dict[tuple[str, str, str], int] = {}
        for item in self.outcomes:
            executed = (
                "yes" if item.executed.crossed
                else ("uncertain" if not item.executed.observed else "no")
            )
            key = ("yes" if item.planned_crossing else "no", executed,
                   "yes" if item.confirmed else "no")
            counts[key] = counts.get(key, 0) + 1
        return counts

    def by_segment_type(self) -> dict[str, dict[str, int]]:
        """Success and failure counts split by pass versus connector.

        Phase 5 measured that all route curvature lives in connectors, so
        whether a connector crossing is physically less reliable than a straight
        pass crossing is a question the aggregate must be able to answer.
        """
        table: dict[str, dict[str, int]] = {}
        for item in self.outcomes:
            kind = item.intended_segment_type or "unassigned"
            row = table.setdefault(
                kind, {"planned": 0, "executed": 0, "confirmed": 0}
            )
            row["planned"] += 1 if item.planned_crossing else 0
            row["executed"] += 1 if item.executed.crossed else 0
            row["confirmed"] += 1 if item.confirmed else 0
        return table


def evaluate_execution(
    *,
    snapshot: ScanSnapshot,
    plan: CollectionRoutePlan,
    trace: ExecutionTrace,
    capture_geometry: CaptureGeometry,
    displacement_threshold_m: float,
    disturbance_reporting_radius_m: float,
    crossing_window_m: float,
) -> ExecutionEvaluation:
    """Reconstruct what happened, without inferring more than was measured.

    ``displacement_threshold_m`` decides only how a *diagnostic* label reads;
    the measured displacement is always reported in metres so the threshold can
    be revisited without re-running anything.  It is not a planner constraint.
    """
    if plan.scan_id != snapshot.scan_id:
        raise ValueError("plan and snapshot are from different scans")
    if trace.plan_id != plan.plan_id:
        raise ValueError("trace was recorded for a different plan")

    mouth = capture_geometry.plane(INTAKE_MOUTH_PLANE_ID)
    intended = _intended_segments(plan)
    observations = _observations_by_ball(trace)
    confirmations = _confirmations(trace)
    attributed = _attributed(trace)

    outcomes = []
    for ball in snapshot.balls:
        outcomes.append(
            _ball_outcome(
                ball, plan, trace, mouth, intended, observations, confirmations,
                displacement_threshold_m, attributed,
            )
        )
    tracking = tuple(_segment_tracking(plan, trace))
    disturbances = tuple(
        _disturbances(
            snapshot, plan, trace, mouth, intended, observations,
            disturbance_reporting_radius_m,
        )
    )
    rows = (
        len(trace.samples) + len(trace.crossings) + len(trace.beams)
        + len(trace.observations)
    )
    return ExecutionEvaluation(
        trace.run_id, trace.plan_id, tuple(outcomes), tracking, disturbances, rows
    )


# ── geometry ────────────────────────────────────────────────────────────────

def _to_robot_frame(pose, point: Point2D) -> tuple[float, float]:
    dx, dy = point.x_m - pose.x_m, point.y_m - pose.y_m
    cos, sin = math.cos(-pose.yaw_rad), math.sin(-pose.yaw_rad)
    return dx * cos - dy * sin, dx * sin + dy * cos


def _mouth_clearance(pose, point: Point2D, mouth) -> float:
    """Distance from a ball to the mouth segment, in the robot's own frame."""
    x, y = _to_robot_frame(pose, point)
    half = mouth.half_width_m
    if y > half:
        return math.hypot(x - mouth.longitudinal_offset_m, y - half)
    if y < -half:
        return math.hypot(x - mouth.longitudinal_offset_m, y + half)
    return abs(x - mouth.longitudinal_offset_m)


def _body_clearance(pose, point: Point2D, length_m: float, width_m: float) -> float:
    """Distance to the chassis rectangle, centred on base_footprint."""
    x, y = _to_robot_frame(pose, point)
    dx = max(abs(x) - length_m / 2.0, 0.0)
    dy = max(abs(y) - width_m / 2.0, 0.0)
    return math.hypot(dx, dy)


def _swept(trace, point: Point2D, mouth, *, restrict_to=None) -> SweptCrossing:
    """Did the mouth plane pass over this point, and how close did it come?

    A sweep is a sign change of the ball's longitudinal position relative to the
    mouth plane between two consecutive samples, while the ball is inside the
    mouth's half width.  Interpolating between samples matters: at 0.35 m/s and
    0.1 m spacing the ball is between samples more often than not.
    """
    best = None
    crossed = False
    lateral = None
    moment = None
    segment = None
    speed = None
    samples = [
        sample for sample in trace.samples
        if restrict_to is None or sample.segment_id in restrict_to
    ]
    for sample in samples:
        clearance = _mouth_clearance(sample, point, mouth)
        if best is None or clearance < best:
            best = clearance
    for first, second in zip(samples, samples[1:]):
        x1, y1 = _to_robot_frame(first, point)
        x2, y2 = _to_robot_frame(second, point)
        ahead = x1 - mouth.longitudinal_offset_m
        behind = x2 - mouth.longitudinal_offset_m
        if ahead > 0.0 >= behind or (ahead == 0.0 and behind <= 0.0):
            span = ahead - behind
            ratio = 0.0 if span == 0.0 else ahead / span
            offset = y1 + ratio * (y2 - y1)
            if abs(offset) <= mouth.half_width_m:
                crossed = True
                if lateral is None or abs(offset) < abs(lateral):
                    lateral = offset
                    moment = first.t_s + ratio * (second.t_s - first.t_s)
                    segment = first.segment_id
                    speed = first.linear_mps
    if best is None:
        return SweptCrossing(False, math.inf, None, None, None, None)
    return SweptCrossing(crossed, best, lateral, moment, segment, speed)


# ── plan and trace views ────────────────────────────────────────────────────

def _intended_segments(plan) -> dict[str, tuple[str, str, float]]:
    """ball_id -> (segment id, segment type, planned crossing progress)."""
    intended = {}
    for segment in plan.segments:
        for crossing in segment.planned_crossings:
            intended[crossing.ball_id] = (
                segment.id, segment.type.value, crossing.progress_s
            )
    return intended


def _observations_by_ball(trace) -> dict[str, list]:
    grouped: dict[str, list] = {}
    for item in trace.observations:
        grouped.setdefault(item.ball_id, []).append(item)
    return grouped


def _confirmations(trace) -> list:
    """Rising confirmed-beam edges, in order.

    Weak evidence: a beam edge says something reached the basket, not which
    ball.  It is used only when no attributed confirmation exists.
    """
    return [event for event in trace.beams if event.beam == "confirmed" and event.rising]


def _attributed(trace) -> dict:
    """The runtime's own ball attributions, which outrank any timing guess.

    ``ExecutionTrace.confirmations`` comes straight from the controller, which
    associated the physical beam with a planned crossing while it was happening
    and had state the offline evaluator does not.  When one exists for a ball,
    no heuristic is consulted.
    """
    by_ball: dict = {}
    for event in getattr(trace, "confirmations", ()):
        by_ball.setdefault(event.ball_id, event)
    return by_ball


def _ball_outcome(
    ball, plan, trace, mouth, intended, observations, confirmations, threshold,
    attributed=None,
):
    ball_id = ball.ball_id
    planning_position = ball.position
    target = intended.get(ball_id)
    segment_id = target[0] if target else None
    segment_type = target[1] if target else None
    planned = target is not None

    seen = observations.get(ball_id, [])
    latest = Point2D(seen[-1].x_m, seen[-1].y_m) if seen else None
    displacement = (
        math.hypot(latest.x_m - planning_position.x_m, latest.y_m - planning_position.y_m)
        if latest is not None else None
    )

    # The physical question is always asked about where the ball actually was,
    # falling back to the planning belief when nothing was observed.
    physical = latest if latest is not None else planning_position
    executed = _swept(trace, physical, mouth)

    authoritative = (attributed or {}).get(ball_id)
    if authoritative is not None:
        matched = authoritative
    else:
        matched = _attribute_confirmation(ball_id, segment_id, executed, confirmations)
    confirmed = matched is not None
    confirmation_t = matched.t_s if matched else None
    latency = (
        confirmation_t - executed.t_s
        if confirmed and executed.t_s is not None else None
    )

    crossing_rows = [
        row for row in trace.crossings if row.ball_id == ball_id
    ]
    lateral_error = (
        min((row.lateral_error_m for row in crossing_rows), key=abs)
        if crossing_rows else None
    )
    speed = crossing_rows[-1].measured_speed_mps if crossing_rows else None

    outcome, detail = _classify(
        planned=planned, executed=executed, confirmed=confirmed,
        displacement=displacement, threshold=threshold,
        intended_segment=segment_id, observed_any=bool(seen),
        authoritative=authoritative,
    )
    return BallOutcome(
        ball_id, outcome, planning_position, latest, displacement, segment_id,
        segment_type, planned, executed, confirmed, confirmation_t, latency,
        lateral_error, speed, detail,
    )


def _attribute_confirmation(ball_id, segment_id, executed, confirmations):
    """The confirmation edge, if any, that belongs to this ball.

    Preference order, strongest evidence first: an edge the executor already
    tagged with this ball; otherwise the first edge after this ball's physical
    sweep that is not tagged with a different ball.  An untagged edge with no
    sweep is never attributed -- that would be inferring a collection from a
    beam that could belong to anything.
    """
    tagged = [event for event in confirmations if event.active_ball_id == ball_id]
    if tagged:
        return tagged[0]
    if executed.t_s is None:
        return None
    for event in confirmations:
        if event.active_ball_id not in (None, ball_id):
            continue
        if event.t_s >= executed.t_s:
            return event
    return None


def _classify(*, planned, executed, confirmed, displacement, threshold,
              intended_segment, observed_any, authoritative=None):
    moved = displacement is not None and displacement > threshold
    if authoritative is not None and not executed.crossed:
        # The machine says it collected this ball; our reconstruction says the
        # mouth never passed over where the ball was believed to be.  Neither
        # fact is rewritten -- the disagreement itself is the finding.
        return (
            CrossingOutcome.CONFIRMED_WITHOUT_RECONSTRUCTED_CROSSING,
            f"confirmed as {authoritative.association} at progress "
            f"{authoritative.progress_s if authoritative.progress_s is not None else float('nan'):.3f}"
            f" while the mouth came no closer than {executed.minimum_clearance_m:.3f} m",
        )
    if confirmed:
        if executed.segment_id and intended_segment and executed.segment_id != intended_segment:
            return (
                CrossingOutcome.COLLECTED_BY_DIFFERENT_SEGMENT,
                f"swept by {executed.segment_id}, planned for {intended_segment}",
            )
        if not planned:
            return (
                CrossingOutcome.COLLECTED_BY_DIFFERENT_SEGMENT,
                "collected without a planned crossing of its own",
            )
        return (CrossingOutcome.PLANNED_AND_EXECUTED_COLLECTED, "")
    if moved:
        # Established before blaming tracking or the collector: a ball that was
        # pushed away was never where the attempt was aimed.
        return (
            CrossingOutcome.BALL_DISPLACED_BEFORE_ATTEMPT,
            f"moved {displacement:.3f} m from its planning position",
        )
    if not executed.observed:
        return (
            CrossingOutcome.OBSERVATION_UNCERTAIN,
            "no trajectory samples covered this ball",
        )
    if planned and not executed.crossed:
        return (
            CrossingOutcome.PLANNED_BUT_TRACKING_MISSED,
            f"closest mouth approach {executed.minimum_clearance_m:.3f} m",
        )
    if executed.crossed:
        if not observed_any:
            # The mouth went over where the ball was believed to be, and nothing
            # since has confirmed either a collection or the ball's position.
            return (
                CrossingOutcome.OBSERVATION_UNCERTAIN,
                "mouth swept the planning position but neither collection nor "
                "position was observed afterwards",
            )
        return (
            CrossingOutcome.EXECUTED_CROSSING_NOT_COLLECTED,
            f"mouth swept at lateral {executed.lateral_offset_m:.3f} m without confirmation",
        )
    return (CrossingOutcome.OTHER, "no planned crossing and no executed sweep")


# ── tracking metrics ────────────────────────────────────────────────────────

def _segment_tracking(plan, trace):
    by_segment: dict[str, list] = {}
    for sample in trace.samples:
        if sample.segment_id:
            by_segment.setdefault(sample.segment_id, []).append(sample)
    for segment in plan.segments:
        samples = by_segment.get(segment.id)
        if not samples:
            continue
        points = [point.pose for point in segment.path.points]
        cross = [abs(_cross_track(points, sample)) for sample in samples]
        heading = [abs(_heading_error(points, sample)) for sample in samples]
        executed_length = sum(
            math.hypot(second.x_m - first.x_m, second.y_m - first.y_m)
            for first, second in zip(samples, samples[1:])
        )
        end = samples[-1]
        target = points[-1]
        yield SegmentTracking(
            segment.id, segment.type.value, len(samples),
            max(cross), _rms(cross), sum(cross) / len(cross),
            max(heading), _rms(heading),
            math.hypot(end.x_m - target.x_m, end.y_m - target.y_m),
            abs(_wrap(end.yaw_rad - target.yaw_rad)),
            segment.progress_end_m - segment.progress_start_m,
            executed_length,
            samples[-1].t_s - samples[0].t_s,
        )


def _cross_track(points, sample) -> float:
    best = math.inf
    for first, second in zip(points, points[1:]):
        distance = _point_segment_signed(first, second, sample)
        if abs(distance) < abs(best):
            best = distance
    return 0.0 if best is math.inf else best


def _point_segment_signed(first, second, sample) -> float:
    dx, dy = second.x_m - first.x_m, second.y_m - first.y_m
    span = dx * dx + dy * dy
    if span <= 1e-12:
        return math.hypot(sample.x_m - first.x_m, sample.y_m - first.y_m)
    ratio = max(0.0, min(1.0, ((sample.x_m - first.x_m) * dx + (sample.y_m - first.y_m) * dy) / span))
    px = first.x_m + ratio * dx
    py = first.y_m + ratio * dy
    normal_x, normal_y = -dy, dx
    length = math.hypot(normal_x, normal_y)
    return ((sample.x_m - px) * normal_x + (sample.y_m - py) * normal_y) / length


def _heading_error(points, sample) -> float:
    best = math.inf
    heading = 0.0
    for first, second in zip(points, points[1:]):
        distance = abs(_point_segment_signed(first, second, sample))
        if distance < best:
            best = distance
            heading = math.atan2(second.y_m - first.y_m, second.x_m - first.x_m)
    return _wrap(sample.yaw_rad - heading)


def _rms(values) -> float:
    return math.sqrt(sum(value * value for value in values) / len(values))


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


# ── disturbance ─────────────────────────────────────────────────────────────

def _attempt_end_times(trace, intended) -> dict[str, float]:
    """When each ball's own attempt segment finished, by ball id."""
    last_seen: dict[str, float] = {}
    for sample in trace.samples:
        if sample.segment_id:
            last_seen[sample.segment_id] = sample.t_s
    return {
        ball_id: last_seen[target[0]]
        for ball_id, target in intended.items()
        if target[0] in last_seen
    }


def _disturbances(snapshot, plan, trace, mouth, intended, observations, radius):
    """Close approaches to balls that were not the target at that moment.

    Reported continuously in metres rather than as a verdict: the purpose is to
    build the empirical relation between closest approach and observed movement,
    which is what a disturbance penalty would eventually need.
    """
    mechanical = snapshot.configuration_snapshot.mechanical
    crossing_times = {
        row.ball_id: row.t_s for row in trace.crossings
    }
    attempt_ends = _attempt_end_times(trace, intended)
    for ball in snapshot.balls:
        target = intended.get(ball.ball_id)
        attempt_t = crossing_times.get(ball.ball_id)
        # Only approaches *before* the ball's own attempt can disturb it.  Once
        # the mouth has been over it the chassis follows through the same place,
        # and reporting that as a near miss would bury the real cases.
        deadline = attempt_ends.get(ball.ball_id)
        closest = None
        for sample in trace.samples:
            if target and sample.segment_id == target[0]:
                continue  # its own attempt is not a disturbance
            if deadline is not None and sample.t_s >= deadline:
                continue
            body = _body_clearance(
                sample, ball.position, mechanical.robot_length_m, mechanical.robot_width_m
            )
            if body > radius:
                continue
            if closest is None or body < closest[0]:
                closest = (
                    body,
                    _mouth_clearance(sample, ball.position, mouth),
                    sample,
                )
        if closest is None:
            continue
        body, mouth_clearance, sample = closest
        seen = observations.get(ball.ball_id, [])
        displacement = (
            math.hypot(seen[-1].x_m - ball.position.x_m, seen[-1].y_m - ball.position.y_m)
            if seen else None
        )
        yield DisturbanceEvent(
            ball.ball_id, sample.segment_id, sample.t_s, body, mouth_clearance,
            sample.linear_mps, displacement,
            None if attempt_t is None else attempt_t - sample.t_s,
        )
