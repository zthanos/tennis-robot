"""Pure Phase 5A execution-view and measurement contract.

No ROS action, controller, speed-limit update, or mutable plan operation lives
here.  The object only projects supplied poses onto a frozen route and emits
typed follower status plus telemetry values.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math

from tennis_robot.collection_route_executor import PathFollowerResult, PathFollowerStatus
from tennis_robot.collection_route_types import (
    CollectionRoutePlan, ExecutionProfile, PlannedCrossing, Pose2D, RouteSegment,
)


class _StringEnum(str, Enum):
    def __str__(self) -> str:
        return self.value


class ProfileViolationReason(_StringEnum):
    SPEED_BELOW_MIN = "speed_below_min"
    SPEED_ABOVE_MAX = "speed_above_max"
    LATERAL_ERROR_EXCEEDED = "lateral_error_exceeded"
    HEADING_ERROR_EXCEEDED = "heading_error_exceeded"


class NominalTracking(_StringEnum):
    WITHIN_TOLERANCE = "within_tolerance"
    DEVIATED = "deviated"


class FollowerTelemetryCode(_StringEnum):
    CROSSING_MEASUREMENT = "crossing_measurement"
    PROFILE_VIOLATION = "profile_violation"
    NOMINAL_SPEED_DEVIATION = "nominal_speed_deviation"
    TRAJECTORY_TUBE_VIOLATION = "trajectory_tube_violation"


@dataclass(frozen=True)
class ProfileComplianceVerdict:
    hard_compliant: bool
    hard_violation_reason: ProfileViolationReason | None
    nominal_tracking: NominalTracking
    measured_speed_mps: float
    nominal_speed_error_mps: float

    def __post_init__(self) -> None:
        if self.hard_compliant != (self.hard_violation_reason is None):
            raise ValueError("hard compliance must agree with violation reason")
        if not isinstance(self.nominal_tracking, NominalTracking):
            raise ValueError("invalid nominal tracking")
        for value in (self.measured_speed_mps, self.nominal_speed_error_mps):
            if not math.isfinite(value):
                raise ValueError("profile measurements must be finite")
        if self.measured_speed_mps < 0.0 or self.nominal_speed_error_mps < 0.0:
            raise ValueError("speed measurements must be non-negative")


@dataclass(frozen=True)
class CrossingMeasurement:
    ball_id: str
    progress_s: float
    measured_speed_mps: float
    lateral_error_m: float
    heading_error_rad: float
    verdict: ProfileComplianceVerdict

    def __post_init__(self) -> None:
        if not self.ball_id or not math.isfinite(self.progress_s) or self.progress_s < 0.0:
            raise ValueError("invalid crossing identity")
        for value in (self.measured_speed_mps, self.lateral_error_m, self.heading_error_rad):
            if not math.isfinite(value):
                raise ValueError("crossing measurement must be finite")


@dataclass(frozen=True)
class FollowerTelemetryEvent:
    code: FollowerTelemetryCode
    crossing: CrossingMeasurement | None = None


@dataclass(frozen=True)
class FlattenedExecutionSegment:
    segment: RouteSegment
    progress_start_s: float
    progress_end_s: float
    crossings: tuple[PlannedCrossing, ...]


@dataclass(frozen=True)
class FlattenedExecutionView:
    plan_id: str
    segments: tuple[FlattenedExecutionSegment, ...]
    crossings: tuple[PlannedCrossing, ...]
    terminal_pose: Pose2D


@dataclass(frozen=True)
class FollowerUpdate:
    result: PathFollowerResult
    telemetry: tuple[FollowerTelemetryEvent, ...]


class CollectionPathFollower:
    """Stateful monotonic projection over an immutable, executable plan."""

    def __init__(self, plan: CollectionRoutePlan) -> None:
        if not isinstance(plan, CollectionRoutePlan) or not plan.is_executable:
            raise ValueError("path follower requires executable CollectionRoutePlan")
        self._plan = plan
        self.view = FlattenedExecutionView(
            plan.plan_id,
            tuple(FlattenedExecutionSegment(segment, segment.progress_start_m, segment.progress_end_m, segment.planned_crossings) for segment in plan.segments),
            tuple(crossing for segment in plan.segments for crossing in segment.planned_crossings),
            plan.terminal_pose,
        )
        self._last_progress_s = 0.0
        self._emitted_crossings: set[tuple[str, float]] = set()

    @property
    def progress_s(self) -> float:
        return self._last_progress_s

    def next_crossing(self, progress_s: float | None = None) -> PlannedCrossing | None:
        current = self._last_progress_s if progress_s is None else progress_s
        return next((item for item in self.view.crossings if item.progress_s >= current), None)

    def remaining_run_in_m(self, progress_s: float | None = None) -> float:
        current = self._last_progress_s if progress_s is None else progress_s
        crossing = self.next_crossing(current)
        return 0.0 if crossing is None else max(0.0, crossing.progress_s - current)

    def observe(self, pose: Pose2D, measured_speed_mps: float) -> FollowerUpdate:
        if not isinstance(pose, Pose2D) or not isinstance(measured_speed_mps, (int, float)) or not math.isfinite(measured_speed_mps) or measured_speed_mps < 0.0:
            raise ValueError("observe requires a finite pose and non-negative speed")
        projected_s, tube_distance = self._project(pose)
        previous_s = self._last_progress_s
        self._last_progress_s = max(previous_s, projected_s)
        tube_ok = tube_distance <= self._plan.configuration_snapshot.safety.trajectory_tube_radius_m
        telemetry: list[FollowerTelemetryEvent] = []
        if not tube_ok:
            telemetry.append(FollowerTelemetryEvent(FollowerTelemetryCode.TRAJECTORY_TUBE_VIOLATION))
        for crossing in self.view.crossings:
            identity = (crossing.ball_id, crossing.progress_s)
            if identity not in self._emitted_crossings and previous_s < crossing.progress_s <= self._last_progress_s:
                measurement = self._measure_crossing(crossing, pose, measured_speed_mps)
                self._emitted_crossings.add(identity)
                telemetry.append(FollowerTelemetryEvent(FollowerTelemetryCode.CROSSING_MEASUREMENT, measurement))
                if not measurement.verdict.hard_compliant:
                    telemetry.append(FollowerTelemetryEvent(FollowerTelemetryCode.PROFILE_VIOLATION, measurement))
                elif measurement.verdict.nominal_tracking is NominalTracking.DEVIATED:
                    telemetry.append(FollowerTelemetryEvent(FollowerTelemetryCode.NOMINAL_SPEED_DEVIATION, measurement))
        if self._last_progress_s >= self._plan.total_length_m:
            result = PathFollowerResult(PathFollowerStatus.COMPLETED)
        else:
            result = PathFollowerResult(PathFollowerStatus.RUNNING, self._last_progress_s, tube_ok, self.remaining_run_in_m(), False, False)
        return FollowerUpdate(result, tuple(telemetry))

    def _measure_crossing(self, crossing: PlannedCrossing, pose: Pose2D, speed: float) -> CrossingMeasurement:
        profile = self._profile_for_crossing(crossing)
        normal = (-math.sin(crossing.heading_rad), math.cos(crossing.heading_rad))
        dx, dy = pose.x_m - crossing.position_xy.x_m, pose.y_m - crossing.position_xy.y_m
        lateral_error = abs(dx * normal[0] + dy * normal[1])
        heading_error = abs(_angle_delta(pose.yaw_rad, crossing.heading_rad))
        violation = None
        if speed < profile.min_speed_mps:
            violation = ProfileViolationReason.SPEED_BELOW_MIN
        elif speed > profile.max_speed_mps:
            violation = ProfileViolationReason.SPEED_ABOVE_MAX
        elif lateral_error > profile.max_lateral_error_m:
            violation = ProfileViolationReason.LATERAL_ERROR_EXCEEDED
        elif heading_error > profile.max_heading_error_rad:
            violation = ProfileViolationReason.HEADING_ERROR_EXCEEDED
        nominal_error = abs(speed - profile.nominal_speed_mps)
        tracking = NominalTracking.DEVIATED if profile.min_speed_mps <= speed <= profile.max_speed_mps and nominal_error > profile.nominal_speed_warning_tolerance_mps else NominalTracking.WITHIN_TOLERANCE
        return CrossingMeasurement(crossing.ball_id, crossing.progress_s, speed, lateral_error, heading_error, ProfileComplianceVerdict(violation is None, violation, tracking, speed, nominal_error))

    def _profile_for_crossing(self, crossing: PlannedCrossing) -> ExecutionProfile:
        for segment in self._plan.segments:
            if crossing in segment.planned_crossings:
                return segment.execution_profile
        raise RuntimeError("crossing does not belong to frozen plan")

    def _project(self, pose: Pose2D) -> tuple[float, float]:
        best: tuple[float, float] | None = None
        for segment in self._plan.segments:
            points = segment.path.points
            lengths = tuple(math.hypot(second.pose.x_m - first.pose.x_m, second.pose.y_m - first.pose.y_m) for first, second in zip(points, points[1:]))
            total = sum(lengths)
            accumulated = 0.0
            for first, second, length in zip(points, points[1:], lengths):
                if length == 0.0:
                    continue
                fraction, distance = _project_point(pose, first.pose, second.pose)
                local = accumulated + fraction * length
                fraction_of_segment = local / total if total else 0.0
                progress = segment.progress_start_m + fraction_of_segment * (segment.progress_end_m - segment.progress_start_m)
                candidate = (progress, distance)
                if best is None or distance < best[1] or (distance == best[1] and progress > best[0]):
                    best = candidate
                accumulated += length
        if best is None:
            raise RuntimeError("executable plan has no projectable path")
        return best


def _project_point(point: Pose2D, start: Pose2D, end: Pose2D) -> tuple[float, float]:
    dx, dy = end.x_m - start.x_m, end.y_m - start.y_m
    length_sq = dx * dx + dy * dy
    raw = ((point.x_m - start.x_m) * dx + (point.y_m - start.y_m) * dy) / length_sq
    fraction = min(1.0, max(0.0, raw))
    x, y = start.x_m + fraction * dx, start.y_m + fraction * dy
    return fraction, math.hypot(point.x_m - x, point.y_m - y)


def _angle_delta(first: float, second: float) -> float:
    return (first - second + math.pi) % (2.0 * math.pi) - math.pi
