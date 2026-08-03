"""Pure Phase 4A lifecycle executor for immutable collection routes.

This module intentionally owns only lifecycle state.  Every external action is
an injected polling port; it has no ROS, Nav2, controller, or perception
dependency.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Protocol

from tennis_robot.collection_route_types import (
    CollectionRoutePlan,
    PlanningStatus,
    RouteSegmentType,
    ScanSnapshot,
)


class _StringEnum(str, Enum):
    def __str__(self) -> str:
        return self.value


class ExecutorState(_StringEnum):
    IDLE = "idle"
    NAVIGATING_TO_SCAN_POSE = "navigating_to_scan_pose"
    SCANNING = "scanning"
    PLANNING = "planning"
    COLLECTOR_STARTING = "collector_starting"
    EXECUTING_ROUTE = "executing_route"
    WAITING_PATH_CLEAR = "waiting_path_clear"
    ROUTE_COMPLETED = "route_completed"
    COLLECTOR_STOPPING = "collector_stopping"
    EVALUATING_RESULTS = "evaluating_results"
    COMPLETED = "completed"
    COMPLETED_NO_TARGETS = "completed_no_targets"
    INCOMPLETE_TARGETS = "incomplete_targets"
    ABORTED_SCAN = "aborted_scan"
    ABORTED_PLANNING = "aborted_planning"
    ABORTED_COLLECTOR = "aborted_collector"
    ABORTED_SAFETY = "aborted_safety"
    ABORTED_TRACKING = "aborted_tracking"


class ExecutorReasonCode(_StringEnum):
    NAVIGATION_FAILED = "navigation_failed"
    NAVIGATION_UNAVAILABLE = "navigation_unavailable"
    SCAN_FAILED = "scan_failed"
    PLANNING_FAILED = "planning_failed"
    COLLECTOR_START_FAILED = "collector_start_failed"
    COLLECTOR_START_TIMEOUT = "collector_start_timeout"
    COLLECTOR_JAM = "collector_jam"
    COLLECTOR_FULL = "collector_full"
    COLLECTOR_HEALTH_FAILURE = "collector_health_failure"
    PATH_FAILED = "path_failed"
    SAFETY_TIMEOUT = "safety_timeout"
    SAFETY_RESUME_INVALID = "safety_resume_invalid"
    COLLECTOR_STOP_FAULT = "collector_stop_fault"


class NavigatorStatus(_StringEnum):
    RUNNING = "running"
    SUCCEEDED = "succeeded"
    FAILED = "failed"
    UNAVAILABLE = "unavailable"


@dataclass(frozen=True)
class NavigatorResult:
    status: NavigatorStatus
    reason: ExecutorReasonCode | None = None

    def __post_init__(self) -> None:
        if self.status in {NavigatorStatus.FAILED, NavigatorStatus.UNAVAILABLE} and self.reason is None:
            raise ValueError("failed/unavailable navigation requires a reason")
        if self.status in {NavigatorStatus.RUNNING, NavigatorStatus.SUCCEEDED} and self.reason is not None:
            raise ValueError("running/succeeded navigation cannot have a reason")


class ScanSessionStatus(_StringEnum):
    RUNNING = "running"
    SNAPSHOT_READY = "snapshot_ready"
    FAILED = "failed"


@dataclass(frozen=True)
class ScanSessionResult:
    status: ScanSessionStatus
    snapshot: ScanSnapshot | None = None
    reason: ExecutorReasonCode | None = None

    def __post_init__(self) -> None:
        if self.status is ScanSessionStatus.SNAPSHOT_READY and not isinstance(self.snapshot, ScanSnapshot):
            raise ValueError("snapshot_ready requires ScanSnapshot")
        if self.status is ScanSessionStatus.FAILED and self.reason is None:
            raise ValueError("failed scan requires a reason")
        if self.status is not ScanSessionStatus.SNAPSHOT_READY and self.snapshot is not None:
            raise ValueError("only snapshot_ready may contain a snapshot")


class CollectorStartStatus(_StringEnum):
    STARTING = "starting"
    READY = "ready"
    FAILED = "failed"


@dataclass(frozen=True)
class CollectorStartResult:
    status: CollectorStartStatus
    reason: ExecutorReasonCode | None = None


class CollectorStopStatus(_StringEnum):
    STOPPING = "stopping"
    STOPPED = "stopped"
    FAILED = "failed"


@dataclass(frozen=True)
class CollectorStopResult:
    status: CollectorStopStatus
    reason: ExecutorReasonCode | None = None


class PathFollowerStatus(_StringEnum):
    RUNNING = "running"
    COMPLETED = "completed"
    FAILED = "failed"


@dataclass(frozen=True)
class PathFollowerResult:
    status: PathFollowerStatus
    progress_s: float | None = None
    trajectory_tube_ok: bool | None = None
    remaining_run_in_m: float | None = None
    requires_reverse: bool | None = None
    requires_standalone_rotate: bool | None = None
    reason: ExecutorReasonCode | None = None
    # Human-readable diagnostic for a failure (specific controller failure label
    # plus geometry); opaque to the executor, surfaced verbatim in telemetry.
    detail: str | None = None

    def __post_init__(self) -> None:
        running = self.status is PathFollowerStatus.RUNNING
        fields = (self.progress_s, self.trajectory_tube_ok, self.remaining_run_in_m, self.requires_reverse, self.requires_standalone_rotate)
        if running and any(value is None for value in fields):
            raise ValueError("running follower result requires progress and profile fields")
        if not running and any(value is not None for value in fields):
            raise ValueError("terminal follower result cannot include running fields")
        if self.progress_s is not None and self.progress_s < 0.0:
            raise ValueError("progress_s must be non-negative")
        if self.remaining_run_in_m is not None and self.remaining_run_in_m < 0.0:
            raise ValueError("remaining_run_in_m must be non-negative")
        if self.status is PathFollowerStatus.FAILED and self.reason is None:
            raise ValueError("failed follower result requires a reason")


class SafetyStatus(_StringEnum):
    CLEAR = "clear"
    BLOCKED = "blocked"
    TIMEOUT = "timeout"


@dataclass(frozen=True)
class SafetyResult:
    status: SafetyStatus


class TelemetryEventCode(_StringEnum):
    STATE_CHANGED = "state_changed"
    COLLECTOR_STOP_FAULT = "collector_stop_fault"
    ROUTE_OUTCOME = "route_outcome"


@dataclass(frozen=True)
class TelemetryEvent:
    code: TelemetryEventCode
    state: ExecutorState
    reason: ExecutorReasonCode | None = None
    detail: str | None = None


class ScanPoseNavigator(Protocol):
    def start(self) -> None: ...
    def result(self) -> NavigatorResult: ...


class ScanSession(Protocol):
    def start(self) -> None: ...
    def result(self) -> ScanSessionResult: ...


class PurePlanner(Protocol):
    def plan(self, snapshot: ScanSnapshot) -> CollectionRoutePlan: ...


class Collector(Protocol):
    def start(self) -> None: ...
    def start_result(self) -> CollectorStartResult: ...
    def active_fault(self) -> ExecutorReasonCode | None: ...
    def stop(self) -> None: ...
    def stop_result(self) -> CollectorStopResult: ...
    def force_disable(self) -> None: ...


class PathFollower(Protocol):
    def start(self, plan: CollectionRoutePlan) -> None: ...
    def result(self) -> PathFollowerResult: ...
    def pause(self) -> None: ...
    def resume(self) -> None: ...


class SafetyMonitor(Protocol):
    def result(self) -> SafetyResult: ...


class TelemetrySink(Protocol):
    def emit(self, event: TelemetryEvent) -> None: ...


class MonotonicClock(Protocol):
    def now_s(self) -> float: ...


class CollectionRouteExecutor:
    """Deterministic, polling pure executor with no hidden retry/fallback path."""

    def __init__(self, *, navigator: ScanPoseNavigator, scan_session: ScanSession,
                 planner: PurePlanner, collector: Collector, path_follower: PathFollower,
                 safety_monitor: SafetyMonitor, telemetry: TelemetrySink,
                 clock: MonotonicClock, drive_observer=None) -> None:
        self._drive_observer = drive_observer
        self._navigator = navigator
        self._scan_session = scan_session
        self._planner = planner
        self._collector = collector
        self._path_follower = path_follower
        self._safety_monitor = safety_monitor
        self._telemetry = telemetry
        self._clock = clock
        self.state = ExecutorState.IDLE
        self.run_count = 0
        self.snapshot: ScanSnapshot | None = None
        self.plan: CollectionRoutePlan | None = None
        self.route_outcome: ExecutorState | None = None
        self.terminal_reason: ExecutorReasonCode | None = None
        self.terminal_detail: str | None = None
        self._collector_started_at_s: float | None = None
        self._collector_stopped_at_s: float | None = None
        self._s_before_pause: float | None = None
        self._last_follower_result: PathFollowerResult | None = None

    @property
    def is_terminal(self) -> bool:
        return self.state in {
            ExecutorState.COMPLETED, ExecutorState.COMPLETED_NO_TARGETS,
            ExecutorState.INCOMPLETE_TARGETS,
            ExecutorState.ABORTED_SCAN, ExecutorState.ABORTED_PLANNING,
            ExecutorState.ABORTED_COLLECTOR, ExecutorState.ABORTED_SAFETY,
            ExecutorState.ABORTED_TRACKING,
        }

    def start(self) -> None:
        if self.state is not ExecutorState.IDLE:
            raise RuntimeError("executor can start only from idle")
        self._begin_navigation()

    def tick(self) -> ExecutorState:
        if self.state is ExecutorState.NAVIGATING_TO_SCAN_POSE:
            result = self._navigator.result()
            if result.status is NavigatorStatus.SUCCEEDED:
                self._scan_session.start()
                self._transition(ExecutorState.SCANNING)
            elif result.status in {NavigatorStatus.FAILED, NavigatorStatus.UNAVAILABLE}:
                self._transition(ExecutorState.ABORTED_SCAN, result.reason)
        elif self.state is ExecutorState.SCANNING:
            result = self._scan_session.result()
            if result.status is ScanSessionStatus.SNAPSHOT_READY:
                self.snapshot = result.snapshot
                self._transition(ExecutorState.PLANNING)
            elif result.status is ScanSessionStatus.FAILED:
                self._transition(ExecutorState.ABORTED_SCAN, result.reason)
        elif self.state is ExecutorState.PLANNING:
            self._plan_snapshot()
        elif self.state is ExecutorState.COLLECTOR_STARTING:
            self._tick_collector_start()
        elif self.state is ExecutorState.EXECUTING_ROUTE:
            self._tick_execution()
        elif self.state is ExecutorState.WAITING_PATH_CLEAR:
            self._tick_waiting_path_clear()
        elif self.state is ExecutorState.COLLECTOR_STOPPING:
            self._tick_collector_stop()
        elif self.state is ExecutorState.EVALUATING_RESULTS:
            self._telemetry.emit(
                TelemetryEvent(
                    TelemetryEventCode.ROUTE_OUTCOME,
                    self.route_outcome or ExecutorState.COMPLETED,
                    self.terminal_reason,
                    self.terminal_detail,
                )
            )
            if self._can_follow_up():
                # Prefer the balls actually seen while driving over repeating
                # the same 360 from the same pose, which can only re-observe the
                # same court.  Falling back keeps today's behaviour when the
                # drive saw nothing new.
                if not self._begin_off_route_pass():
                    self._begin_navigation()
            else:
                self._transition(self._terminal_completion_state())
        return self.state

    def _begin_off_route_pass(self) -> bool:
        """Plan the follow-up from off-route discoveries, in place of a rescan.

        Consumes the same bounded follow-up budget as a rescan, so a mission
        still runs at most ``follow_up.max_total_runs`` routes however many new
        balls keep appearing.
        """
        if self._drive_observer is None or self.snapshot is None:
            return False
        if self.run_count >= self.snapshot.configuration_snapshot.follow_up.max_total_runs:
            return False
        known_positions = tuple(
            (ball.position.x_m, ball.position.y_m) for ball in self.snapshot.balls
        )
        snapshot = self._drive_observer.result(known_positions=known_positions)
        if snapshot is None or not snapshot.balls:
            return False
        self.run_count += 1
        self.snapshot = snapshot
        self.plan = None
        self.route_outcome = None
        self.terminal_reason = None
        self.terminal_detail = None
        # No navigation and no 360: the follow-up is planned from where the
        # route ended, against targets already observed from two viewpoints.
        self._transition(ExecutorState.PLANNING)
        return True

    def _begin_navigation(self) -> None:
        configuration = self.snapshot.configuration_snapshot if self.snapshot is not None else self.plan.configuration_snapshot if self.plan is not None else None
        # The first configuration is learned from the finalized snapshot.  A
        # follow-up uses that same immutable lifecycle policy.
        if configuration is not None:
            follow_up = configuration.follow_up
            if self.run_count >= follow_up.max_total_runs:
                self._transition(self._terminal_completion_state())
                return
        self.run_count += 1
        self.snapshot = None
        self.plan = None
        self.route_outcome = None
        self.terminal_reason = None
        self.terminal_detail = None
        self._navigator.start()
        self._transition(ExecutorState.NAVIGATING_TO_SCAN_POSE)

    def _plan_snapshot(self) -> None:
        if not isinstance(self.snapshot, ScanSnapshot):
            self._transition(ExecutorState.ABORTED_PLANNING, ExecutorReasonCode.PLANNING_FAILED)
            return
        try:
            plan = self._planner.plan(self.snapshot)
            if not isinstance(plan, CollectionRoutePlan):
                raise ValueError("planner did not return CollectionRoutePlan")
            plan.validate_against(self.snapshot)
        except (ValueError, TypeError):
            self._transition(ExecutorState.ABORTED_PLANNING, ExecutorReasonCode.PLANNING_FAILED)
            return
        self.plan = plan
        if plan.planning_status is PlanningStatus.EMPTY_NO_BALLS:
            self._transition(ExecutorState.COMPLETED_NO_TARGETS)
        elif plan.planning_status is PlanningStatus.EMPTY_NO_FEASIBLE_TARGETS:
            # Targets were observed and classified, but none has an executable
            # route.  This is not the same outcome as an empty court: callers
            # must surface the unresolved targets and their planner blockers.
            self._transition(ExecutorState.INCOMPLETE_TARGETS)
        elif not plan.is_executable:
            self._transition(ExecutorState.ABORTED_PLANNING, ExecutorReasonCode.PLANNING_FAILED)
        else:
            self._collector.start()
            self._collector_started_at_s = self._clock.now_s()
            self._transition(ExecutorState.COLLECTOR_STARTING)

    def _tick_collector_start(self) -> None:
        assert self.plan is not None
        result = self._collector.start_result()
        if result.status is CollectorStartStatus.READY:
            self._path_follower.start(self.plan)
            if self._drive_observer is not None:
                self._drive_observer.start()
            self._transition(ExecutorState.EXECUTING_ROUTE)
        elif result.status is CollectorStartStatus.FAILED or self._elapsed(self._collector_started_at_s, self.plan.configuration_snapshot.safety.collector_start_timeout_s):
            self._abort_active_route(ExecutorState.ABORTED_COLLECTOR, result.reason or ExecutorReasonCode.COLLECTOR_START_TIMEOUT)

    def _tick_execution(self) -> None:
        assert self.plan is not None
        if self._drive_observer is not None:
            # Balls the 360 never confirmed are only ever seen from here.  This
            # feeds a separate list; the frozen plan being executed is untouched.
            self._drive_observer.observe()
        safety = self._safety_monitor.result()
        if safety.status is SafetyStatus.TIMEOUT:
            self._abort_active_route(ExecutorState.ABORTED_SAFETY, ExecutorReasonCode.SAFETY_TIMEOUT)
            return
        follower = self._path_follower.result()
        self._last_follower_result = follower
        if safety.status is SafetyStatus.BLOCKED:
            self._path_follower.pause()
            self._s_before_pause = follower.progress_s if follower.status is PathFollowerStatus.RUNNING else None
            self._transition(ExecutorState.WAITING_PATH_CLEAR)
            return
        fault = self._collector.active_fault()
        if fault is not None:
            self._abort_active_route(ExecutorState.ABORTED_COLLECTOR, fault)
        elif follower.status is PathFollowerStatus.FAILED:
            self._abort_active_route(ExecutorState.ABORTED_TRACKING, follower.reason or ExecutorReasonCode.PATH_FAILED, follower.detail)
        elif follower.status is PathFollowerStatus.COMPLETED:
            self.route_outcome = ExecutorState.ROUTE_COMPLETED
            self._collector.stop()
            self._collector_stopped_at_s = self._clock.now_s()
            self._transition(ExecutorState.ROUTE_COMPLETED)
            self._transition(ExecutorState.COLLECTOR_STOPPING)

    def _tick_waiting_path_clear(self) -> None:
        assert self.plan is not None
        safety = self._safety_monitor.result()
        if safety.status is SafetyStatus.TIMEOUT:
            self._abort_active_route(ExecutorState.ABORTED_SAFETY, ExecutorReasonCode.SAFETY_TIMEOUT)
            return
        if safety.status is SafetyStatus.BLOCKED:
            return
        follower = self._path_follower.result()
        self._last_follower_result = follower
        if not self._resume_is_valid(follower):
            self._abort_active_route(ExecutorState.ABORTED_TRACKING, ExecutorReasonCode.SAFETY_RESUME_INVALID)
            return
        self._path_follower.resume()
        self._transition(ExecutorState.EXECUTING_ROUTE)

    def _resume_is_valid(self, follower: PathFollowerResult) -> bool:
        if follower.status is not PathFollowerStatus.RUNNING or self._s_before_pause is None:
            return False
        required_run_in = self._required_next_run_in(follower.progress_s or 0.0)
        return bool(
            follower.progress_s is not None and follower.progress_s >= self._s_before_pause
            and follower.trajectory_tube_ok is True
            and follower.remaining_run_in_m is not None and follower.remaining_run_in_m >= required_run_in
            and follower.requires_reverse is False and follower.requires_standalone_rotate is False
        )

    def _required_next_run_in(self, progress_s: float) -> float:
        assert self.plan is not None
        for segment in self.plan.segments:
            if segment.type is RouteSegmentType.FUNNEL_PASS and segment.progress_end_m >= progress_s:
                return segment.execution_profile.required_run_in_m
        return 0.0

    def _abort_active_route(self, outcome: ExecutorState, reason: ExecutorReasonCode, detail: str | None = None) -> None:
        self.route_outcome = outcome
        self._collector.stop()
        self._collector_stopped_at_s = self._clock.now_s()
        self._transition(outcome, reason, detail)
        self._transition(ExecutorState.COLLECTOR_STOPPING)

    def _tick_collector_stop(self) -> None:
        assert self.plan is not None
        result = self._collector.stop_result()
        if result.status is CollectorStopStatus.STOPPED:
            self._transition(ExecutorState.EVALUATING_RESULTS)
        elif result.status is CollectorStopStatus.FAILED or self._elapsed(self._collector_stopped_at_s, self.plan.configuration_snapshot.safety.collector_stop_timeout_s):
            self._collector.force_disable()
            self._telemetry.emit(TelemetryEvent(TelemetryEventCode.COLLECTOR_STOP_FAULT, self.route_outcome or ExecutorState.ROUTE_COMPLETED, result.reason or ExecutorReasonCode.COLLECTOR_STOP_FAULT))
            self._transition(ExecutorState.EVALUATING_RESULTS)

    def _can_follow_up(self) -> bool:
        assert self.plan is not None
        # A follow-up run is "collect more balls in further passes", not a
        # retry: it is allowed only after a clean route completion, never after
        # a safety/tracking/collector abort of the active route.
        policy = self.plan.configuration_snapshot.follow_up
        return (
            self.route_outcome is ExecutorState.ROUTE_COMPLETED
            and policy.enabled
            and self.run_count < policy.max_total_runs
        )

    def _terminal_completion_state(self) -> ExecutorState:
        """Report mission completion separately from sub-route completion.

        A clean FollowPath result only proves that the executable subset ended.
        If the final immutable plan is partial, targets remain deferred or
        unreachable and the collection mission must not be labelled completed.
        """
        if self.route_outcome in {
            ExecutorState.ABORTED_COLLECTOR,
            ExecutorState.ABORTED_SAFETY,
            ExecutorState.ABORTED_TRACKING,
        }:
            return self.route_outcome
        if (
            self.route_outcome is ExecutorState.ROUTE_COMPLETED
            and self.plan is not None
            and self.plan.planning_status is PlanningStatus.PARTIAL
        ):
            return ExecutorState.INCOMPLETE_TARGETS
        return ExecutorState.COMPLETED

    def _elapsed(self, started_at_s: float | None, timeout_s: float) -> bool:
        return started_at_s is not None and self._clock.now_s() - started_at_s >= timeout_s

    def _transition(self, state: ExecutorState, reason: ExecutorReasonCode | None = None, detail: str | None = None) -> None:
        if state.value.startswith("aborted_"):
            if reason is not None:
                self.terminal_reason = reason
            if detail is not None:
                self.terminal_detail = detail
        self.state = state
        self._telemetry.emit(TelemetryEvent(TelemetryEventCode.STATE_CHANGED, state, reason, detail))
