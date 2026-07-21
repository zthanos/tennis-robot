"""Pure Phase 4A executor tests: injected fakes only, no ROS runtime."""

from dataclasses import replace
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "ros2_ws", "src", "tennis_robot"))

from collection_route_fixtures import default_configuration
from tennis_robot.collection_route_executor import (
    CollectionRouteExecutor, CollectorStartResult, CollectorStartStatus,
    CollectorStopResult, CollectorStopStatus, ExecutorReasonCode, ExecutorState,
    NavigatorResult, NavigatorStatus, PathFollowerResult, PathFollowerStatus,
    SafetyResult, SafetyStatus, ScanSessionResult, ScanSessionStatus,
)
from tennis_robot.collection_route_planner_v2 import CourtModel, plan_collection_route
from tennis_robot.collection_route_types import (
    FollowUpConfiguration, Point2D, Pose2D, PositionCovariance2D,
    PlanningSearchStatus, PlanningStatus, ScanSnapshot, SnapshotBall,
)


class Clock:
    def __init__(self): self.value = 0.0
    def now_s(self): return self.value
    def advance(self, seconds): self.value += seconds


class Navigator:
    def __init__(self, results): self.results = list(results); self.starts = 0
    def start(self): self.starts += 1
    def result(self): return self.results.pop(0) if len(self.results) > 1 else self.results[0]


class Session:
    def __init__(self, results): self.results = list(results); self.starts = 0
    def start(self): self.starts += 1
    def result(self): return self.results.pop(0) if len(self.results) > 1 else self.results[0]


class Planner:
    def __init__(self, plan=None, failure=False): self.plan_value = plan; self.failure = failure; self.calls = []
    def plan(self, snapshot):
        self.calls.append(snapshot)
        if self.failure: raise ValueError("planner failure")
        return self.plan_value


class Collector:
    def __init__(self, starts=(CollectorStartResult(CollectorStartStatus.READY),), stops=(CollectorStopResult(CollectorStopStatus.STOPPED),), faults=()):
        self.starts, self.stops, self.faults = list(starts), list(stops), list(faults)
        self.start_calls = self.stop_calls = self.force_disable_calls = 0
    def start(self): self.start_calls += 1
    def start_result(self): return self.starts.pop(0) if len(self.starts) > 1 else self.starts[0]
    def active_fault(self): return self.faults.pop(0) if self.faults else None
    def stop(self): self.stop_calls += 1
    def stop_result(self): return self.stops.pop(0) if len(self.stops) > 1 else self.stops[0]
    def force_disable(self): self.force_disable_calls += 1


class Follower:
    def __init__(self, results): self.results = list(results); self.started = []; self.pauses = 0; self.resumes = 0
    def start(self, plan): self.started.append(plan)
    def result(self): return self.results.pop(0) if len(self.results) > 1 else self.results[0]
    def pause(self): self.pauses += 1
    def resume(self): self.resumes += 1


class Safety:
    def __init__(self, results): self.results = list(results)
    def result(self): return self.results.pop(0) if len(self.results) > 1 else self.results[0]


class Telemetry:
    def __init__(self): self.events = []
    def emit(self, event): self.events.append(event)


def court():
    return CourtModel((Point2D(-20.0, -20.0), Point2D(20.0, -20.0), Point2D(20.0, 20.0), Point2D(-20.0, 20.0)), ())


def snapshot(*, balls=("ball",), follow_up=FollowUpConfiguration(False, 1), scan_id="scan"):
    config = replace(default_configuration(), follow_up=follow_up)
    return ScanSnapshot(scan_id, 100.0, "map", Pose2D(0.0, 0.0, 0.0), tuple(SnapshotBall(ball, Point2D(3.0 + index, 0.0), 0.9, PositionCovariance2D(1e-4, 0.0, 1e-4)) for index, ball in enumerate(balls)), config)


def executable_plan(snap): return plan_collection_route(snapshot=snap, court=court(), configuration=snap.configuration_snapshot).plan
def empty_plan(snap): return plan_collection_route(snapshot=snap, court=court(), configuration=snap.configuration_snapshot).plan


def make_executor(*, snap, nav=(NavigatorResult(NavigatorStatus.SUCCEEDED),), scan=None, planner=None,
                  collector=None, follower=None, safety=(SafetyResult(SafetyStatus.CLEAR),), clock=None):
    clock = clock or Clock()
    return CollectionRouteExecutor(
        navigator=Navigator(nav), scan_session=Session(scan or (ScanSessionResult(ScanSessionStatus.SNAPSHOT_READY, snap),)),
        planner=planner or Planner(executable_plan(snap)), collector=collector or Collector(),
        path_follower=follower or Follower((PathFollowerResult(PathFollowerStatus.COMPLETED),)),
        safety_monitor=Safety(safety), telemetry=Telemetry(), clock=clock,
    ), clock


def advance_to_execution(executor):
    executor.start(); executor.tick(); executor.tick(); executor.tick(); executor.tick()
    assert executor.state is ExecutorState.EXECUTING_ROUTE


def finish(executor):
    for _ in range(12):
        if executor.state in {ExecutorState.COMPLETED, ExecutorState.COMPLETED_NO_TARGETS, ExecutorState.ABORTED_SCAN, ExecutorState.ABORTED_PLANNING}:
            return
        executor.tick()


def test_scan_and_planning_terminal_states_and_planner_requires_snapshot():
    snap = snapshot()
    aborted_scan, _ = make_executor(snap=snap, nav=(NavigatorResult(NavigatorStatus.FAILED, ExecutorReasonCode.NAVIGATION_FAILED),))
    aborted_scan.start(); aborted_scan.tick()
    assert aborted_scan.state is ExecutorState.ABORTED_SCAN

    failed_planner, _ = make_executor(snap=snap, planner=Planner(failure=True))
    failed_planner.start(); failed_planner.tick(); failed_planner.tick(); failed_planner.tick()
    assert failed_planner.state is ExecutorState.ABORTED_PLANNING

    empty = snapshot(balls=())
    no_targets, _ = make_executor(snap=empty, planner=Planner(empty_plan(empty)))
    no_targets.start(); no_targets.tick(); no_targets.tick(); no_targets.tick()
    assert no_targets.state is ExecutorState.COMPLETED_NO_TARGETS
    assert no_targets._collector.start_calls == 0


def test_successful_route_and_frozen_plan_are_preserved():
    snap = snapshot(); plan = executable_plan(snap)
    executor, _ = make_executor(snap=snap, planner=Planner(plan))
    advance_to_execution(executor); executor.tick(); finish(executor)
    assert executor.state is ExecutorState.COMPLETED
    assert executor.plan is plan
    assert executor.plan.to_dict() == plan.to_dict()


def test_collector_start_timeout_and_active_jam_full_health_abort():
    snap = snapshot(); clock = Clock()
    starting = Collector(starts=(CollectorStartResult(CollectorStartStatus.STARTING),))
    timeout, _ = make_executor(snap=snap, collector=starting, clock=clock)
    timeout.start(); timeout.tick(); timeout.tick(); timeout.tick(); clock.advance(3.0); timeout.tick()
    assert timeout.route_outcome is ExecutorState.ABORTED_COLLECTOR
    assert timeout.state is ExecutorState.COLLECTOR_STOPPING

    for fault in (ExecutorReasonCode.COLLECTOR_JAM, ExecutorReasonCode.COLLECTOR_FULL, ExecutorReasonCode.COLLECTOR_HEALTH_FAILURE):
        executor, _ = make_executor(snap=snap, collector=Collector(faults=(fault,)), follower=Follower((PathFollowerResult(PathFollowerStatus.RUNNING, 0.0, True, 2.0, False, False),)))
        advance_to_execution(executor); executor.tick()
        assert executor.route_outcome is ExecutorState.ABORTED_COLLECTOR


def test_stop_timeout_force_disables_without_changing_completed_outcome():
    snap = snapshot(); clock = Clock()
    collector = Collector(stops=(CollectorStopResult(CollectorStopStatus.STOPPING),))
    executor, _ = make_executor(snap=snap, collector=collector, clock=clock)
    advance_to_execution(executor); executor.tick(); clock.advance(3.0); executor.tick(); executor.tick()
    assert collector.force_disable_calls == 1
    assert executor.route_outcome is ExecutorState.ROUTE_COMPLETED
    assert executor.state is ExecutorState.COMPLETED
    assert any(event.code.value == "collector_stop_fault" for event in executor._telemetry.events)


def test_safety_valid_resume_and_invalid_resume_never_backtracks():
    snap = snapshot()
    running = PathFollowerResult(PathFollowerStatus.RUNNING, 1.0, True, 2.0, False, False)
    follower = Follower((running, running, PathFollowerResult(PathFollowerStatus.COMPLETED)))
    executor, _ = make_executor(snap=snap, follower=follower, safety=(SafetyResult(SafetyStatus.BLOCKED), SafetyResult(SafetyStatus.CLEAR), SafetyResult(SafetyStatus.CLEAR)))
    advance_to_execution(executor); executor.tick(); assert executor.state is ExecutorState.WAITING_PATH_CLEAR
    executor.tick(); assert executor.state is ExecutorState.EXECUTING_ROUTE and follower.resumes == 1

    invalid = Follower((running, PathFollowerResult(PathFollowerStatus.RUNNING, 0.5, True, 2.0, False, False)))
    aborted, _ = make_executor(snap=snap, follower=invalid, safety=(SafetyResult(SafetyStatus.BLOCKED), SafetyResult(SafetyStatus.CLEAR)))
    advance_to_execution(aborted); aborted.tick(); aborted.tick()
    assert aborted.route_outcome is ExecutorState.ABORTED_TRACKING
    assert invalid.resumes == 0


def test_safety_timeout_and_follower_failure_are_distinct_terminal_outcomes():
    snap = snapshot()
    safety_abort, _ = make_executor(snap=snap, safety=(SafetyResult(SafetyStatus.TIMEOUT),))
    advance_to_execution(safety_abort); safety_abort.tick()
    assert safety_abort.route_outcome is ExecutorState.ABORTED_SAFETY
    tracking, _ = make_executor(snap=snap, follower=Follower((PathFollowerResult(PathFollowerStatus.FAILED, reason=ExecutorReasonCode.PATH_FAILED),)))
    advance_to_execution(tracking); tracking.tick()
    assert tracking.route_outcome is ExecutorState.ABORTED_TRACKING


def test_executable_partial_planning_timeout_runs_route():
    # A planner's executable PARTIAL result—including budget exhaustion—uses the
    # same frozen execution path and is not confused with non-executable timeout.
    snap = snapshot(balls=("a", "b"))
    limited_configuration = replace(
        snap.configuration_snapshot,
        global_route_search=replace(snap.configuration_snapshot.global_route_search, max_search_expansions=1),
    )
    snap = replace(snap, configuration_snapshot=limited_configuration)
    partial = executable_plan(snap)
    assert partial.is_executable
    assert partial.planning_status is PlanningStatus.PARTIAL
    assert partial.planning_search_status is PlanningSearchStatus.BUDGET_EXHAUSTED
    executor, _ = make_executor(snap=snap, planner=Planner(partial))
    advance_to_execution(executor)
    assert executor.plan is partial


def test_bounded_follow_up_enabled_disabled_and_limit():
    first = snapshot(follow_up=FollowUpConfiguration(True, 2), scan_id="first")
    second = snapshot(follow_up=FollowUpConfiguration(True, 2), scan_id="second")
    planner = Planner(executable_plan(first))
    executor, _ = make_executor(
        snap=first,
        scan=(ScanSessionResult(ScanSessionStatus.SNAPSHOT_READY, first), ScanSessionResult(ScanSessionStatus.SNAPSHOT_READY, second)),
        planner=planner,
        follower=Follower((PathFollowerResult(PathFollowerStatus.COMPLETED), PathFollowerResult(PathFollowerStatus.COMPLETED))),
    )
    # Update result per scan while retaining the identity validation contract.
    original_plan = planner.plan
    def plan_for_snapshot(s): return executable_plan(s)
    planner.plan = plan_for_snapshot
    executor.start()
    for _ in range(16): executor.tick()
    assert executor.state is ExecutorState.COMPLETED and executor.run_count == 2
    assert executor._navigator.starts == 2

    disabled = snapshot(follow_up=FollowUpConfiguration(False, 1))
    one, _ = make_executor(snap=disabled)
    advance_to_execution(one); one.tick(); finish(one)
    assert one.run_count == 1


def test_post_scan_events_do_not_replan_or_mutate_geometry():
    snap = snapshot(); planner = Planner(executable_plan(snap))
    executor, _ = make_executor(snap=snap, planner=planner)
    advance_to_execution(executor)
    before = executor.plan.to_dict()
    for _ in range(3): executor.tick()  # execution/pure telemetry has no perception input path
    assert len(planner.calls) == 1
    assert executor.plan.to_dict() == before
