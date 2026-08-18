"""The route executor must drive the trace lifecycle, not just own the object.

Phase 9's unit tests drove ``ExecutionTraceCapture`` by hand, so they passed
while the executor never called ``start`` or ``finish`` and every live run threw
its samples away (debug log #66).  These tests therefore go through
``CollectionRouteExecutor`` itself: if the wiring is removed, they fail.
"""

import json
import os
import sys
from dataclasses import replace

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "ros2_ws", "src", "tennis_robot"))

from collection_route_fixtures import default_configuration  # noqa: E402
from tennis_robot.collection_execution_recorder import ExecutionTraceCapture  # noqa: E402
from tennis_robot.collection_execution_trace import ExecutionTrace  # noqa: E402
from tennis_robot.collection_route_executor import (  # noqa: E402
    CollectionRouteExecutor,
    CollectorStartResult,
    CollectorStartStatus,
    CollectorStopResult,
    CollectorStopStatus,
    ExecutorReasonCode,
    ExecutorState,
    NavigatorResult,
    NavigatorStatus,
    PathFollowerResult,
    PathFollowerStatus,
    SafetyResult,
    SafetyStatus,
    ScanSessionResult,
    ScanSessionStatus,
)
from tennis_robot.collection_route_planner_v2 import CourtModel, plan_collection_route  # noqa: E402
from tennis_robot.collection_route_types import (  # noqa: E402
    FollowUpConfiguration,
    Point2D,
    Pose2D,
    PositionCovariance2D,
    ScanSnapshot,
    SnapshotBall,
)


# ── minimal doubles, matching the executor's ports ──────────────────────────

class Clock:
    """The executor's monotonic clock port."""

    def __init__(self):
        self.value = 0.0

    def now_s(self):
        self.value += 0.1
        return round(self.value, 6)


def ticking_clock():
    """A plain callable, which is what the trace capture takes."""
    state = {"t": 0.0}

    def now():
        state["t"] += 0.1
        return round(state["t"], 6)

    return now


class Navigator:
    def start(self):
        pass

    def result(self):
        return NavigatorResult(NavigatorStatus.SUCCEEDED)


class Session:
    def __init__(self, snapshot):
        self._snapshot = snapshot

    def start(self):
        pass

    def result(self):
        return ScanSessionResult(ScanSessionStatus.SNAPSHOT_READY, self._snapshot)


class Planner:
    def __init__(self, plan):
        self._plan = plan

    def plan(self, snapshot):
        return self._plan


class Collector:
    def __init__(self):
        self.started = self.stopped = 0

    def start(self):
        self.started += 1

    def start_result(self):
        return CollectorStartResult(CollectorStartStatus.READY)

    def active_fault(self):
        return None

    def stop(self):
        self.stopped += 1

    def stop_result(self):
        return CollectorStopResult(CollectorStopStatus.STOPPED)

    def force_disable(self):
        pass


class Follower:
    """Reports RUNNING for a few ticks, then whatever ending is asked for."""

    def __init__(self, ending, running_ticks=3):
        self._ending = ending
        self._running = running_ticks
        self.started = []

    def start(self, plan):
        self.started.append(plan)

    def result(self):
        if self._running > 0:
            self._running -= 1
            return PathFollowerResult(PathFollowerStatus.RUNNING, 1.0, True, 0.0, False, False)
        return self._ending

    def pause(self):
        pass

    def resume(self):
        pass


class Safety:
    def result(self):
        return SafetyResult(SafetyStatus.CLEAR)


class Telemetry:
    def __init__(self):
        self.events = []

    def emit(self, event):
        self.events.append(event)


class SpyState:
    """Stands in for the controller state message the transport forwards."""

    def __init__(self, segment_id, progress, ball_id=None):
        self.active_segment_id = segment_id
        self.progress_s = progress
        self.measured_speed_mps = 0.35
        self.has_active_crossing = ball_id is not None
        self.active_ball_id = ball_id
        self.active_crossing_progress_s = progress
        self.lateral_error_m = 0.01
        self.heading_error_rad = 0.02


def court():
    return CourtModel(
        (Point2D(-20.0, -20.0), Point2D(20.0, -20.0), Point2D(20.0, 20.0), Point2D(-20.0, 20.0)),
        (),
    )


def snapshot(scan_id="scan-int", balls=(("a", 3.0, 0.0), ("b", 4.0, 0.0))):
    configuration = replace(
        default_configuration(maximum_candidate_count=40),
        follow_up=FollowUpConfiguration(False, 1),
    )
    return ScanSnapshot(
        scan_id, 100.0, "map", Pose2D(0.0, 0.0, 0.0),
        tuple(
            SnapshotBall(ball_id, Point2D(x, y), 0.9, PositionCovariance2D(1e-4, 0.0, 1e-4))
            for ball_id, x, y in balls
        ),
        configuration,
    )


def build(tmp_path, ending, scan_id="scan-int"):
    scan = snapshot(scan_id)
    plan = plan_collection_route(
        snapshot=scan, court=court(), configuration=scan.configuration_snapshot
    ).plan
    capture = ExecutionTraceCapture(
        directory=tmp_path, run_id="run-int", clock_fn=ticking_clock(), spacing_m=0.0, interval_s=0.0
    )
    follower = Follower(ending)
    executor = CollectionRouteExecutor(
        navigator=Navigator(), scan_session=Session(scan), planner=Planner(plan),
        collector=Collector(), path_follower=follower, safety_monitor=Safety(),
        telemetry=Telemetry(), clock=Clock(), execution_trace=capture,
    )
    return executor, capture, plan, follower


def drive(executor, capture, *, samples=3, ticks=20):
    """Run the executor, feeding trace samples while the route is executing."""
    executor.start()
    fed = 0
    for _ in range(ticks):
        if executor.is_terminal:
            break
        executor.tick()
        if executor.state is ExecutorState.EXECUTING_ROUTE and fed < samples:
            capture.record_state(
                pose=Pose2D(float(fed), 0.0, 0.0),
                state=SpyState("pass-0", float(fed), "a" if fed == 1 else None),
            )
            fed += 1
    return fed


def written(tmp_path):
    return sorted(tmp_path.glob("*.trace.json"))


def test_a_successful_route_persists_its_trace(tmp_path):
    executor, capture, plan, _ = build(
        tmp_path, PathFollowerResult(PathFollowerStatus.COMPLETED)
    )
    fed = drive(executor, capture)
    assert fed == 3
    assert executor.is_terminal
    files = written(tmp_path)
    assert len(files) == 1
    trace = ExecutionTrace.from_dict(json.loads(files[0].read_text()))
    assert trace.plan_id == plan.plan_id
    assert trace.scan_id == plan.scan_id
    assert len(trace.samples) == 3
    assert [item.ball_id for item in trace.crossings] == ["a"]


def test_the_trace_only_starts_when_the_route_starts(tmp_path):
    executor, capture, _, _ = build(
        tmp_path, PathFollowerResult(PathFollowerStatus.COMPLETED)
    )
    # Nothing is recording during navigation, scanning or planning.
    executor.start()
    assert not capture.active
    while executor.state is not ExecutorState.EXECUTING_ROUTE and not executor.is_terminal:
        executor.tick()
        if executor.state in (ExecutorState.SCANNING, ExecutorState.PLANNING):
            assert not capture.active, f"trace started during {executor.state}"
    assert capture.active, "trace must be recording once the route is executing"


def test_an_aborted_route_still_persists_its_trace(tmp_path):
    # The real terminal abort seen live: trajectory_tube_exceeded 2 cm from the
    # end.  That run is the most valuable evidence there is, so it must survive.
    executor, capture, plan, _ = build(
        tmp_path,
        PathFollowerResult(
            PathFollowerStatus.FAILED,
            reason=ExecutorReasonCode.PATH_FAILED,
            # The detail string the live run produced 2 cm from the end.
            detail="trajectory_tube_exceeded | seg terminal progress 20.347m lat_err 0.000m",
        ),
    )
    drive(executor, capture)
    assert executor.state is ExecutorState.ABORTED_TRACKING
    files = written(tmp_path)
    assert len(files) == 1
    trace = ExecutionTrace.from_dict(json.loads(files[0].read_text()))
    assert trace.plan_id == plan.plan_id
    assert len(trace.samples) == 3, "partial evidence must be kept"


def test_a_safety_abort_persists_its_trace(tmp_path):
    scan = snapshot()
    plan = plan_collection_route(
        snapshot=scan, court=court(), configuration=scan.configuration_snapshot
    ).plan
    capture = ExecutionTraceCapture(
        directory=tmp_path, run_id="run-int", clock_fn=ticking_clock(), spacing_m=0.0, interval_s=0.0
    )

    class Timeout:
        def result(self):
            return SafetyResult(SafetyStatus.TIMEOUT)

    executor = CollectionRouteExecutor(
        navigator=Navigator(), scan_session=Session(scan), planner=Planner(plan),
        collector=Collector(),
        path_follower=Follower(PathFollowerResult(PathFollowerStatus.COMPLETED)),
        safety_monitor=Timeout(), telemetry=Telemetry(), clock=Clock(),
        execution_trace=capture,
    )
    drive(executor, capture, samples=1)
    assert executor.state is ExecutorState.ABORTED_SAFETY
    assert len(written(tmp_path)) == 1


def test_disabled_tracing_is_an_exact_no_op(tmp_path):
    scan = snapshot()
    plan = plan_collection_route(
        snapshot=scan, court=court(), configuration=scan.configuration_snapshot
    ).plan
    executor = CollectionRouteExecutor(
        navigator=Navigator(), scan_session=Session(scan), planner=Planner(plan),
        collector=Collector(),
        path_follower=Follower(PathFollowerResult(PathFollowerStatus.COMPLETED)),
        safety_monitor=Safety(), telemetry=Telemetry(), clock=Clock(),
        execution_trace=None,
    )
    executor.start()
    for _ in range(20):
        if executor.is_terminal:
            break
        executor.tick()
    assert executor.is_terminal
    assert not list(tmp_path.iterdir())


def test_repeated_terminal_transitions_do_not_corrupt_the_artifact(tmp_path):
    executor, capture, _, _ = build(
        tmp_path, PathFollowerResult(PathFollowerStatus.COMPLETED)
    )
    drive(executor, capture)
    first = written(tmp_path)
    assert len(first) == 1
    payload = first[0].read_text()
    # Terminal notified again: idempotent, no second file, no rewrite.
    executor._transition(executor.state)
    executor._transition(ExecutorState.ABORTED_TRACKING)
    assert written(tmp_path) == first
    assert first[0].read_text() == payload


def test_instrumentation_failure_never_aborts_the_route(tmp_path):
    class Exploding:
        def start(self, plan):
            raise RuntimeError("start blew up")

        def finish(self):
            raise RuntimeError("finish blew up")

    scan = snapshot()
    plan = plan_collection_route(
        snapshot=scan, court=court(), configuration=scan.configuration_snapshot
    ).plan
    executor = CollectionRouteExecutor(
        navigator=Navigator(), scan_session=Session(scan), planner=Planner(plan),
        collector=Collector(),
        path_follower=Follower(PathFollowerResult(PathFollowerStatus.COMPLETED)),
        safety_monitor=Safety(), telemetry=Telemetry(), clock=Clock(),
        execution_trace=Exploding(),
    )
    executor.start()
    for _ in range(20):
        if executor.is_terminal:
            break
        executor.tick()
    assert executor.state is ExecutorState.COMPLETED


def test_two_routes_in_one_process_produce_two_identified_traces(tmp_path):
    plans = []
    for index, scan_id in enumerate(("scan-first", "scan-second")):
        executor, capture, plan, _ = build(
            tmp_path, PathFollowerResult(PathFollowerStatus.COMPLETED), scan_id=scan_id
        )
        drive(executor, capture)
        plans.append(plan)
    files = written(tmp_path)
    assert len(files) == 2, "each route needs its own artifact"
    traces = [ExecutionTrace.from_dict(json.loads(path.read_text())) for path in files]
    assert {trace.plan_id for trace in traces} == {plan.plan_id for plan in plans}
    assert {trace.scan_id for trace in traces} == {"scan-first", "scan-second"}


def test_audit_and_trace_identities_match(tmp_path):
    """What the evaluator joins on: same plan, same scan, same run."""
    executor, capture, plan, _ = build(
        tmp_path, PathFollowerResult(PathFollowerStatus.COMPLETED)
    )
    drive(executor, capture)
    trace = ExecutionTrace.from_dict(json.loads(written(tmp_path)[0].read_text()))
    assert trace.plan_id == plan.plan_id
    assert trace.scan_id == plan.scan_id
    assert trace.run_id == "run-int"


def test_the_evaluator_refuses_to_join_a_trace_from_another_plan(tmp_path):
    from tennis_robot.collection_capture_geometry import (
        PlaneProvenance,
        repo_base_footprint_capture_geometry,
    )
    from tennis_robot.collection_execution_evaluator import evaluate_execution

    executor, capture, plan, _ = build(
        tmp_path, PathFollowerResult(PathFollowerStatus.COMPLETED)
    )
    drive(executor, capture)
    trace = ExecutionTrace.from_dict(json.loads(written(tmp_path)[0].read_text()))
    other = snapshot("scan-other", balls=(("z", 5.0, 1.0),))
    other_plan = plan_collection_route(
        snapshot=other, court=court(), configuration=other.configuration_snapshot
    ).plan
    geometry = repo_base_footprint_capture_geometry(
        required_pre_contact_straight_m=0.3,
        required_pre_contact_provenance=PlaneProvenance.CONFIGURED,
    )
    with pytest.raises(ValueError):
        evaluate_execution(
            snapshot=other, plan=other_plan, trace=trace, capture_geometry=geometry,
            displacement_threshold_m=0.1, disturbance_reporting_radius_m=1.5,
            crossing_window_m=0.5,
        )


# ── attributed confirmations (Phase 9C) ─────────────────────────────────────

def test_an_attributed_confirmation_reaches_the_persisted_trace(tmp_path):
    """The runtime already attributes confirmations; the trace must keep them."""
    executor, capture, plan, _ = build(
        tmp_path, PathFollowerResult(PathFollowerStatus.COMPLETED)
    )
    executor.start()
    for _ in range(20):
        if executor.state is ExecutorState.EXECUTING_ROUTE:
            break
        executor.tick()
    # Exactly the dict the controller builds for collect_route.confirmations.
    capture.record_confirmation({
        "confirmation_id": 1,
        "association": "intake_lead_crossing",
        "ball_id": "a",
        "segment_id": "pass-0",
        "progress_s": 11.638,
        "crossing_progress_s": 12.349,
        "lateral_error_m": 0.0258,
        "heading_error_rad": 0.0264,
        "measured_speed_mps": 0.0,
        "plan_id": plan.plan_id,
    })
    for _ in range(20):
        if executor.is_terminal:
            break
        executor.tick()
    trace = ExecutionTrace.from_dict(json.loads(written(tmp_path)[0].read_text()))
    assert len(trace.confirmations) == 1
    event = trace.confirmations[0]
    assert event.ball_id == "a"
    assert event.association == "intake_lead_crossing"
    assert event.confirmation_id == 1
    assert event.progress_s == pytest.approx(11.638)
    assert event.lateral_error_m == pytest.approx(0.0258)


def test_an_unattributed_confirmation_is_not_recorded_against_any_ball(tmp_path):
    executor, capture, _, _ = build(
        tmp_path, PathFollowerResult(PathFollowerStatus.COMPLETED)
    )
    executor.start()
    for _ in range(20):
        if executor.state is ExecutorState.EXECUTING_ROUTE:
            break
        executor.tick()
    capture.record_confirmation({"confirmation_id": 1, "association": "unassigned", "ball_id": None})
    for _ in range(20):
        if executor.is_terminal:
            break
        executor.tick()
    trace = ExecutionTrace.from_dict(json.loads(written(tmp_path)[0].read_text()))
    assert trace.confirmations == ()


# ── multi-route sessions (debug log #70) ────────────────────────────────────

class RestartingFollower:
    """Runs a few ticks after every start, so each route of a session executes."""

    def __init__(self, ending, running_ticks=3):
        self._ending = ending
        self._ticks = running_ticks
        self._running = 0
        self.started = []

    def start(self, plan):
        self.started.append(plan)
        self._running = self._ticks

    def result(self):
        if self._running > 0:
            self._running -= 1
            return PathFollowerResult(PathFollowerStatus.RUNNING, 1.0, True, 0.0, False, False)
        return self._ending

    def pause(self):
        pass

    def resume(self):
        pass


class SequenceSession:
    """A different snapshot per 360, which is what a follow-up scan produces."""

    def __init__(self, snapshots):
        self._snapshots = list(snapshots)
        self._index = -1

    def start(self):
        self._index += 1

    def result(self):
        index = min(max(self._index, 0), len(self._snapshots) - 1)
        return ScanSessionResult(ScanSessionStatus.SNAPSHOT_READY, self._snapshots[index])


class LivePlanner:
    """Plans each snapshot for real, so the two runs get different plan ids."""

    def plan(self, snapshot):
        return plan_collection_route(
            snapshot=snapshot, court=court(), configuration=snapshot.configuration_snapshot
        ).plan


def follow_up_snapshot(scan_id, balls):
    configuration = replace(
        default_configuration(maximum_candidate_count=40),
        follow_up=FollowUpConfiguration(True, 2),
    )
    return ScanSnapshot(
        scan_id, 100.0, "map", Pose2D(0.0, 0.0, 0.0),
        tuple(
            SnapshotBall(ball_id, Point2D(x, y), 0.9, PositionCovariance2D(1e-4, 0.0, 1e-4))
            for ball_id, x, y in balls
        ),
        configuration,
    )


def test_every_route_of_a_multi_run_session_persists_its_own_trace(tmp_path):
    """A session that collects, rescans and collects again must keep both.

    Live evidence (debug log #70): only the *last* route of a two-run session
    reached disk, because a finished route passes through ROUTE_COMPLETED and
    the session reaches a terminal state exactly once, at the very end.  The
    first route -- the one that collects the most balls -- was thrown away.
    """
    first = follow_up_snapshot("scan-run-1", (("a", 3.0, 0.0), ("b", 4.0, 0.0)))
    second = follow_up_snapshot("scan-run-2", (("c", 3.0, 2.0),))
    capture = ExecutionTraceCapture(
        directory=tmp_path, run_id="run-multi", clock_fn=ticking_clock(),
        spacing_m=0.0, interval_s=0.0,
    )
    executor = CollectionRouteExecutor(
        navigator=Navigator(), scan_session=SequenceSession((first, second)),
        planner=LivePlanner(), collector=Collector(),
        path_follower=RestartingFollower(PathFollowerResult(PathFollowerStatus.COMPLETED)),
        safety_monitor=Safety(), telemetry=Telemetry(), clock=Clock(),
        execution_trace=capture,
    )

    executor.start()
    seen = []
    for _ in range(80):
        if executor.is_terminal:
            break
        executor.tick()
        if executor.state is ExecutorState.EXECUTING_ROUTE:
            seen.append(executor.plan.plan_id)
            capture.record_state(
                pose=Pose2D(float(len(seen)), 0.0, 0.0),
                state=SpyState("pass-0", float(len(seen))),
            )

    assert executor.run_count == 2, "the session must have executed two routes"
    plan_ids = sorted(set(seen))
    assert len(plan_ids) == 2, f"expected two distinct plans, got {plan_ids}"

    files = written(tmp_path)
    assert len(files) == 2, f"every executed route needs its own trace, got {files}"
    traces = [ExecutionTrace.from_dict(json.loads(path.read_text())) for path in files]
    assert sorted(trace.plan_id for trace in traces) == plan_ids
    assert sorted(trace.scan_id for trace in traces) == ["scan-run-1", "scan-run-2"]
    for trace in traces:
        assert trace.samples, f"{trace.plan_id} persisted with no trajectory"


# ── follow-up after a classified tracking abort (Phase 20B) ─────────────────

class FailingThenRunningFollower:
    """Fails the first route with a given reason, then drives the next cleanly."""

    def __init__(self, reason, detail, running_ticks=3):
        self._reason, self._detail = reason, detail
        self._ticks = running_ticks
        self._running = 0
        self._route = 0
        self.started = []

    def start(self, plan):
        self.started.append(plan)
        self._route += 1
        self._running = self._ticks

    def result(self):
        if self._running > 0:
            self._running -= 1
            return PathFollowerResult(PathFollowerStatus.RUNNING, 1.0, True, 0.0, False, False)
        if self._route == 1:
            return PathFollowerResult(
                PathFollowerStatus.FAILED, reason=self._reason, detail=self._detail)
        return PathFollowerResult(PathFollowerStatus.COMPLETED)

    def pause(self):
        pass

    def resume(self):
        pass


def run_session(reason, detail):
    first = follow_up_snapshot("scan-run-1", (("a", 3.0, 0.0), ("b", 4.0, 0.0)))
    second = follow_up_snapshot("scan-run-2", (("c", 3.0, 2.0),))
    follower = FailingThenRunningFollower(reason, detail)
    executor = CollectionRouteExecutor(
        navigator=Navigator(), scan_session=SequenceSession((first, second)),
        planner=LivePlanner(), collector=Collector(), path_follower=follower,
        safety_monitor=Safety(), telemetry=Telemetry(), clock=Clock(),
    )
    executor.start()
    for _ in range(80):
        if executor.is_terminal:
            break
        executor.tick()
    return executor, follower


def test_a_classified_tracking_abort_continues_with_a_follow_up_route():
    """The measured case: that pass is lost, the rest of the route is not."""
    executor, follower = run_session(
        ExecutorReasonCode.PATH_FAILED,
        "trajectory_tube_exceeded | seg pass-0 progress 21.5m lat_err 0.310m")
    assert executor.run_count == 2, "the remaining targets must get a second route"
    assert len(follower.started) == 2
    assert follower.started[0].plan_id != follower.started[1].plan_id
    assert follower.started[0].scan_id == "scan-run-1"
    assert follower.started[1].scan_id == "scan-run-2", "the follow-up rescans"


def test_a_heading_abort_is_treated_the_same_way():
    executor, follower = run_session(
        ExecutorReasonCode.PATH_FAILED,
        "heading_error_exceeded | seg pass-0 progress 6.0m head_err 0.20rad")
    assert executor.run_count == 2
    assert len(follower.started) == 2


def test_a_safety_resume_failure_still_ends_the_mission():
    executor, follower = run_session(ExecutorReasonCode.SAFETY_RESUME_INVALID, None)
    assert executor.run_count == 1, "a safety failure must not be continued"
    assert len(follower.started) == 1
    assert executor.state is ExecutorState.ABORTED_TRACKING


def test_an_unrecognised_tracking_failure_still_ends_the_mission():
    executor, follower = run_session(
        ExecutorReasonCode.PATH_FAILED, "curvature_exceeded | seg connector-0")
    assert executor.run_count == 1
    assert len(follower.started) == 1


def test_the_run_budget_still_bounds_the_mission():
    """abort -> rescan -> abort must not cycle: max_total_runs is the guard."""
    first = follow_up_snapshot("scan-run-1", (("a", 3.0, 0.0), ("b", 4.0, 0.0)))
    second = follow_up_snapshot("scan-run-2", (("c", 3.0, 2.0),))
    detail = "trajectory_tube_exceeded | seg pass-0 progress 21.5m"

    class AlwaysFailing(FailingThenRunningFollower):
        def result(self):
            if self._running > 0:
                self._running -= 1
                return PathFollowerResult(PathFollowerStatus.RUNNING, 1.0, True, 0.0, False, False)
            return PathFollowerResult(
                PathFollowerStatus.FAILED, reason=self._reason, detail=self._detail)

    follower = AlwaysFailing(ExecutorReasonCode.PATH_FAILED, detail)
    executor = CollectionRouteExecutor(
        navigator=Navigator(), scan_session=SequenceSession((first, second)),
        planner=LivePlanner(), collector=Collector(), path_follower=follower,
        safety_monitor=Safety(), telemetry=Telemetry(), clock=Clock(),
    )
    executor.start()
    for _ in range(160):
        if executor.is_terminal:
            break
        executor.tick()
    assert executor.is_terminal
    assert executor.run_count == 2, "the budget is max_total_runs, not unlimited"
    assert executor.state is ExecutorState.ABORTED_TRACKING
