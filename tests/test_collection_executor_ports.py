"""Phase 6C.1: ROS-adapter executor ports — fake-ROS unit tests, no rclpy.

Every port is exercised with plain fakes (fake node/clock/navigator/collector/
laserscan/detection frames).  The decision logic is pure, so importing this
module and running it needs no ROS at all.
"""

import math
import os
import sys
from types import SimpleNamespace as N

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "ros2_ws", "src", "tennis_robot"))

from collection_route_fixtures import SCAN_POSE, default_configuration
from tennis_robot.collection_executor_ports import (
    CallbackTelemetrySink,
    ExecutorPortError,
    ForwardSectorSafetyLogic,
    GazeboCollectorAdapter,
    LidarSafetyMonitor,
    RosMonotonicClock,
    ScanPoseNavigatorAdapter,
    ScanRotationFsm,
    ScanSample,
    ScanSessionDriver,
    forward_sector_blocked,
    navigator_result_for_state,
    telemetry_event_to_dict,
)
from tennis_robot.collection_route_executor import (
    CollectorStartStatus,
    CollectorStopStatus,
    ExecutorReasonCode,
    ExecutorState,
    NavigatorStatus,
    SafetyStatus,
    ScanSessionStatus,
    TelemetryEvent,
    TelemetryEventCode,
)
from tennis_robot.collection_route_types import ScanSnapshot


class Clock:
    def __init__(self, value=0.0):
        self.value = value

    def now_s(self):
        return self.value

    def advance(self, seconds):
        self.value += seconds


# ── 1. MonotonicClock ────────────────────────────────────────────────────────
def test_ros_monotonic_clock_converts_nanoseconds_to_seconds():
    node = N(get_clock=lambda: N(now=lambda: N(nanoseconds=1_500_000_000)))
    assert RosMonotonicClock(node).now_s() == pytest.approx(1.5)


# ── 2. TelemetrySink ─────────────────────────────────────────────────────────
def test_telemetry_event_serialization_is_pure_dict():
    event = TelemetryEvent(TelemetryEventCode.STATE_CHANGED, ExecutorState.SCANNING, ExecutorReasonCode.SCAN_FAILED)
    assert telemetry_event_to_dict(event) == {"code": "state_changed", "state": "scanning", "reason": "scan_failed"}
    no_reason = TelemetryEvent(TelemetryEventCode.ROUTE_OUTCOME, ExecutorState.COMPLETED)
    assert telemetry_event_to_dict(no_reason) == {"code": "route_outcome", "state": "completed", "reason": None}
    detailed = TelemetryEvent(TelemetryEventCode.STATE_CHANGED, ExecutorState.ABORTED_TRACKING, ExecutorReasonCode.PATH_FAILED, "trajectory_tube_exceeded | seg connector-0")
    assert telemetry_event_to_dict(detailed) == {"code": "state_changed", "state": "aborted_tracking", "reason": "path_failed", "detail": "trajectory_tube_exceeded | seg connector-0"}


def test_callback_telemetry_sink_forwards_serialized_dict():
    received = []
    sink = CallbackTelemetrySink(received.append)
    sink.emit(TelemetryEvent(TelemetryEventCode.STATE_CHANGED, ExecutorState.PLANNING))
    assert received == [{"code": "state_changed", "state": "planning", "reason": None}]


def test_callback_telemetry_sink_rejects_non_callable():
    with pytest.raises(ExecutorPortError):
        CallbackTelemetrySink(object())


# ── 3. ScanPoseNavigator ─────────────────────────────────────────────────────
class FakeLaneNavigator:
    def __init__(self, state_value="pending"):
        self.state = N(value=state_value)
        self.requests = []

    def request(self, x, y, yaw):
        self.requests.append((x, y, yaw))


@pytest.mark.parametrize(
    "state_value,expected_status,expected_reason",
    [
        ("idle", NavigatorStatus.RUNNING, None),
        ("pending", NavigatorStatus.RUNNING, None),
        ("active", NavigatorStatus.RUNNING, None),
        ("reached", NavigatorStatus.SUCCEEDED, None),
        ("failed", NavigatorStatus.FAILED, ExecutorReasonCode.NAVIGATION_FAILED),
        ("unavailable", NavigatorStatus.UNAVAILABLE, ExecutorReasonCode.NAVIGATION_UNAVAILABLE),
    ],
)
def test_navigator_result_mapping(state_value, expected_status, expected_reason):
    result = navigator_result_for_state(state_value)
    assert result.status is expected_status
    assert result.reason is expected_reason


def test_navigator_result_unknown_state_raises():
    with pytest.raises(ExecutorPortError):
        navigator_result_for_state("wandering")


def test_scan_pose_navigator_requests_scan_pose_and_maps_state():
    navigator = FakeLaneNavigator("failed")
    adapter = ScanPoseNavigatorAdapter(lane_navigator=navigator, scan_pose=(1.0, -2.0, 0.5))
    adapter.start()
    assert navigator.requests == [(1.0, -2.0, 0.5)]
    result = adapter.result()
    assert result.status is NavigatorStatus.FAILED
    assert result.reason is ExecutorReasonCode.NAVIGATION_FAILED


def test_scan_pose_navigator_retries_nav2_startup_unavailability():
    navigator = FakeLaneNavigator("unavailable")
    adapter = ScanPoseNavigatorAdapter(
        lane_navigator=navigator, scan_pose=(1.0, -2.0, 0.5)
    )
    adapter.start()

    result = adapter.result()

    assert result.status is NavigatorStatus.RUNNING
    assert result.reason is None
    assert navigator.requests == [(1.0, -2.0, 0.5), (1.0, -2.0, 0.5)]


def test_scan_pose_navigator_rejects_bad_pose():
    with pytest.raises(ExecutorPortError):
        ScanPoseNavigatorAdapter(lane_navigator=FakeLaneNavigator(), scan_pose=(1.0, 2.0))
    with pytest.raises(ExecutorPortError):
        ScanPoseNavigatorAdapter(lane_navigator=FakeLaneNavigator(), scan_pose=(1.0, 2.0, float("nan")))


# ── 4. Collector ─────────────────────────────────────────────────────────────
class FakeCollectorInterface:
    def __init__(self):
        self.events = []

    def start(self):
        self.events.append("start")

    def stop(self):
        self.events.append("stop")


def test_gazebo_collector_adapter_maps_ports_without_inventing_faults():
    collector = FakeCollectorInterface()
    adapter = GazeboCollectorAdapter(collector)
    adapter.start()
    assert adapter.start_result().status is CollectorStartStatus.READY
    assert adapter.active_fault() is None
    adapter.stop()
    assert adapter.stop_result().status is CollectorStopStatus.STOPPED
    adapter.force_disable()
    assert collector.events == ["start", "stop"]


def test_gazebo_collector_adapter_drains_entry_until_confirmation():
    collector = FakeCollectorInterface()
    beams = {"entry": False, "confirmed": False}
    now = [10.0]
    adapter = GazeboCollectorAdapter(
        collector,
        entry_beam_provider=lambda: beams["entry"],
        confirmed_beam_provider=lambda: beams["confirmed"],
        minimum_drain_s=1.5,
        maximum_drain_s=5.0,
        clock_fn=lambda: now[0],
    )

    adapter.start()
    beams["entry"] = True
    assert adapter.active_fault() is None
    beams["entry"] = False
    adapter.stop()
    assert adapter.stop_result().status is CollectorStopStatus.STOPPING

    now[0] += 1.6
    assert adapter.stop_result().status is CollectorStopStatus.STOPPING
    beams["confirmed"] = True
    assert adapter.stop_result().status is CollectorStopStatus.STOPPED
    assert collector.events == ["start", "stop"]


def test_gazebo_collector_adapter_drain_has_bounded_timeout():
    collector = FakeCollectorInterface()
    beams = {"entry": True, "confirmed": False}
    now = [20.0]
    adapter = GazeboCollectorAdapter(
        collector,
        entry_beam_provider=lambda: beams["entry"],
        confirmed_beam_provider=lambda: beams["confirmed"],
        minimum_drain_s=1.5,
        maximum_drain_s=5.0,
        clock_fn=lambda: now[0],
    )

    adapter.start()
    adapter.active_fault()
    adapter.stop()
    now[0] += 5.1
    assert adapter.stop_result().status is CollectorStopStatus.STOPPED
    assert collector.events == ["start", "stop"]


def test_gazebo_collector_adapter_requires_new_confirmation_crossing():
    collector = FakeCollectorInterface()
    beams = {"entry": False, "confirmed": True}
    now = [30.0]
    adapter = GazeboCollectorAdapter(
        collector,
        entry_beam_provider=lambda: beams["entry"],
        confirmed_beam_provider=lambda: beams["confirmed"],
        minimum_drain_s=1.5,
        maximum_drain_s=5.0,
        clock_fn=lambda: now[0],
    )

    adapter.start()
    adapter.active_fault()
    beams["entry"] = True
    adapter.active_fault()
    beams["entry"] = False
    adapter.stop()
    now[0] += 1.6
    assert adapter.stop_result().status is CollectorStopStatus.STOPPING

    beams["confirmed"] = False
    adapter.stop_result()
    beams["confirmed"] = True
    assert adapter.stop_result().status is CollectorStopStatus.STOPPED
    assert collector.events == ["start", "stop"]


# ── 5. SafetyMonitor ─────────────────────────────────────────────────────────
def _scan(ranges, *, stamp_s=0.0, angle_min=-math.pi / 2, angle_increment=math.pi / 4, range_min=0.1, range_max=30.0):
    sec = int(stamp_s)
    return N(header=N(stamp=N(sec=sec, nanosec=int(round((stamp_s - sec) * 1e9)))),
             ranges=list(ranges), angle_min=angle_min, angle_increment=angle_increment, range_min=range_min, range_max=range_max)


def _sample(ranges, *, stamp_s, angle_min=-math.pi / 2, angle_increment=math.pi / 4, range_min=0.1, range_max=30.0):
    return ScanSample(stamp_s=stamp_s, ranges=tuple(ranges), angle_min=angle_min,
                      angle_increment=angle_increment, range_min=range_min, range_max=range_max)


def _logic():
    return ForwardSectorSafetyLogic(forward_half_angle_rad=math.radians(20), stop_distance_m=1.0,
                                    safety_pause_timeout_s=2.0, max_scan_age_s=0.5)


# indices 0..4 -> bearings -90, -45, 0, 45, 90 deg
_CLEAR = [10, 10, 10, 10, 10]
_BLOCKED = [10, 10, 0.5, 10, 10]


def test_forward_sector_blocked_detects_only_valid_forward_returns():
    common = dict(angle_min=-math.pi / 2, angle_increment=math.pi / 4, range_min=0.1, range_max=30.0,
                  forward_half_angle_rad=math.radians(20), stop_distance_m=1.0)
    # Obstacle straight ahead (index 2, bearing 0) at 0.5 m -> blocked.
    assert forward_sector_blocked(ranges=[10, 10, 0.5, 10, 10], **common) is True
    # Same close return but off to the side (index 4, bearing 90) -> clear.
    assert forward_sector_blocked(ranges=[10, 10, 10, 10, 0.5], **common) is False
    # Forward return that is invalid (inf / beyond range_max) -> ignored.
    assert forward_sector_blocked(ranges=[10, 10, float("inf"), 10, 10], **common) is False
    assert forward_sector_blocked(ranges=[10, 10, 99.0, 10, 10], **common) is False


def test_safety_logic_clear_blocked_then_timeout():
    logic = _logic()
    # Fresh clear scan (age 0) -> CLEAR.
    assert logic.evaluate(scan=_sample(_CLEAR, stamp_s=0.0), now_s=0.0).status is SafetyStatus.CLEAR
    # Fresh blocked scan -> BLOCKED (timer starts).
    assert logic.evaluate(scan=_sample(_BLOCKED, stamp_s=0.0), now_s=0.0).status is SafetyStatus.BLOCKED
    # Still blocked, under timeout -> BLOCKED.
    assert logic.evaluate(scan=_sample(_BLOCKED, stamp_s=1.5), now_s=1.5).status is SafetyStatus.BLOCKED
    # Sustained past the timeout -> TIMEOUT.
    assert logic.evaluate(scan=_sample(_BLOCKED, stamp_s=2.0), now_s=2.0).status is SafetyStatus.TIMEOUT
    # Clearing resets the block timer.
    assert logic.evaluate(scan=_sample(_CLEAR, stamp_s=3.0), now_s=3.0).status is SafetyStatus.CLEAR
    assert logic.evaluate(scan=_sample(_BLOCKED, stamp_s=3.0), now_s=3.0).status is SafetyStatus.BLOCKED


def test_safety_watchdog_missing_and_stale_scan_are_failsafe_blocked():
    logic = _logic()
    # No scan at all -> fail-safe BLOCKED (not clear).
    assert logic.evaluate(scan=None, now_s=0.0).status is SafetyStatus.BLOCKED
    # A fresh clear scan clears it.
    assert logic.evaluate(scan=_sample(_CLEAR, stamp_s=0.0), now_s=0.0).status is SafetyStatus.CLEAR
    # A clear-content but STALE scan (age 0.6 > max 0.5) -> fail-safe BLOCKED.
    assert logic.evaluate(scan=_sample(_CLEAR, stamp_s=0.0), now_s=0.6).status is SafetyStatus.BLOCKED


def test_safety_watchdog_sustained_stale_scan_times_out():
    logic = _logic()
    # Prolonged absence of a fresh scan escalates BLOCKED -> TIMEOUT like an obstacle.
    assert logic.evaluate(scan=None, now_s=0.0).status is SafetyStatus.BLOCKED
    assert logic.evaluate(scan=None, now_s=1.0).status is SafetyStatus.BLOCKED
    assert logic.evaluate(scan=None, now_s=2.0).status is SafetyStatus.TIMEOUT


def test_safety_logic_rejects_missing_thresholds():
    base = {"forward_half_angle_rad": 0.3, "stop_distance_m": 1.0, "safety_pause_timeout_s": 2.0, "max_scan_age_s": 0.5}
    for bad in ({"forward_half_angle_rad": 0.0}, {"stop_distance_m": -1.0},
                {"safety_pause_timeout_s": float("inf")}, {"max_scan_age_s": 0.0}):
        with pytest.raises(ExecutorPortError):
            ForwardSectorSafetyLogic(**{**base, **bad})


def test_lidar_safety_monitor_missing_scan_is_failsafe_blocked():
    clock = Clock()
    monitor = LidarSafetyMonitor(logic=_logic(), clock=clock, scan_provider=lambda: None)
    # No scan yet -> fail-safe BLOCKED.
    assert monitor.result().status is SafetyStatus.BLOCKED


def test_lidar_safety_monitor_reads_fresh_scan_stamp():
    clock = Clock(value=100.0)
    box = {"scan": _scan(_BLOCKED, stamp_s=100.0)}
    monitor = LidarSafetyMonitor(logic=_logic(), clock=clock, scan_provider=lambda: box["scan"])
    # Fresh blocked scan -> BLOCKED.
    assert monitor.result().status is SafetyStatus.BLOCKED
    # Fresh clear scan -> CLEAR.
    box["scan"] = _scan(_CLEAR, stamp_s=100.0)
    assert monitor.result().status is SafetyStatus.CLEAR
    # Scan stops updating; clock advances past max_scan_age_s -> fail-safe BLOCKED.
    clock.value = 100.6
    assert monitor.result().status is SafetyStatus.BLOCKED


# ── 6. ScanSession ───────────────────────────────────────────────────────────
def test_scan_rotation_fsm_captures_each_step_and_completes_360():
    fsm = ScanRotationFsm(step_count=4, yaw_tolerance_rad=0.05, start_yaw_rad=0.0)
    assert fsm.expected_step_ids == ("scan-step-0", "scan-step-1", "scan-step-2", "scan-step-3")
    targets = [0.0, math.pi / 2, math.pi, -math.pi / 2]
    captured = []
    for target in targets:
        assert not fsm.is_complete
        # target_yaw_rad is wrapped to (-pi, pi], so compare modulo 2*pi.
        assert ((fsm.target_yaw_rad - target + math.pi) % (2 * math.pi)) - math.pi == pytest.approx(0.0, abs=1e-6)
        captured.append(fsm.observe(target))
    assert captured == ["scan-step-0", "scan-step-1", "scan-step-2", "scan-step-3"]
    assert fsm.is_complete
    assert fsm.target_yaw_rad is None
    assert fsm.observe(0.0) is None


def test_scan_rotation_fsm_ignores_yaw_away_from_target():
    fsm = ScanRotationFsm(step_count=4, yaw_tolerance_rad=0.05, start_yaw_rad=0.0)
    fsm.observe(0.0)  # capture step 0, next target is +pi/2
    assert fsm.observe(0.3) is None  # not near pi/2
    assert fsm.observe(math.pi / 2) == "scan-step-1"


def test_scan_rotation_fsm_rejects_bad_config():
    with pytest.raises(ExecutorPortError):
        ScanRotationFsm(step_count=0, yaw_tolerance_rad=0.05, start_yaw_rad=0.0)
    with pytest.raises(ExecutorPortError):
        ScanRotationFsm(step_count=4, yaw_tolerance_rad=0.0, start_yaw_rad=0.0)


class FakeSnapshotSession:
    def __init__(self, snapshot=None, fail=False):
        self._snapshot = snapshot
        self._fail = fail
        self.forwarded = []
        self.finalized_at = None
        self.started_at = None

    def start(self, now_s):
        self.started_at = now_s

    def forward_frame(self, frame, *, scan_step_id):
        self.forwarded.append((frame, scan_step_id))

    def finalize(self, now_s):
        self.finalized_at = now_s
        if self._fail:
            raise RuntimeError("insufficient coverage")
        return self._snapshot


def _snapshot():
    return ScanSnapshot("scan-parity", 1000.0, "map", SCAN_POSE, (), default_configuration())


def _driver(session, *, yaws, frame=N(detections=[]), timeout=100.0, clock=None):
    clock = clock or Clock()
    yaw_box = {"queue": list(yaws)}

    def yaw_provider():
        return yaw_box["queue"].pop(0) if yaw_box["queue"] else None

    cmd = []
    driver = ScanSessionDriver(
        fsm=ScanRotationFsm(step_count=4, yaw_tolerance_rad=0.05, start_yaw_rad=0.0),
        snapshot_session=session,
        yaw_provider=yaw_provider,
        frame_provider=lambda: frame,
        cmd_vel=cmd.append,
        clock=clock,
        angular_speed_rad_s=0.5,
        scan_timeout_s=timeout,
    )
    return driver, cmd, clock


def test_scan_session_driver_completes_and_returns_snapshot_ready():
    snapshot = _snapshot()
    session = FakeSnapshotSession(snapshot=snapshot)
    frame = N(detections=[])
    driver, cmd, _ = _driver(session, yaws=[0.0, math.pi / 2, math.pi, -math.pi / 2], frame=frame)
    driver.start()
    assert cmd[0] == pytest.approx(0.5)  # started rotating
    assert session.started_at == pytest.approx(0.0)
    results = [driver.result() for _ in range(4)]
    statuses = [r.status for r in results]
    assert statuses[:3] == [ScanSessionStatus.RUNNING] * 3
    assert statuses[3] is ScanSessionStatus.SNAPSHOT_READY
    assert results[3].snapshot is snapshot
    # One frame forwarded per captured step, in order.
    assert session.forwarded == [(frame, f"scan-step-{k}") for k in range(4)]
    assert cmd[-1] == pytest.approx(0.0)  # stopped at completion
    # Terminal result is cached.
    assert driver.result().status is ScanSessionStatus.SNAPSHOT_READY


def test_scan_session_driver_finalize_failure_is_scan_failed():
    session = FakeSnapshotSession(fail=True)
    driver, _, _ = _driver(session, yaws=[0.0, math.pi / 2, math.pi, -math.pi / 2])
    driver.start()
    results = [driver.result() for _ in range(4)]
    assert results[3].status is ScanSessionStatus.FAILED
    assert results[3].reason is ExecutorReasonCode.SCAN_FAILED


def test_scan_session_driver_times_out_when_rotation_stalls():
    session = FakeSnapshotSession(fail=True)  # finalize on a stalled scan fails
    clock = Clock()
    # Yaw never matches the first target (+0.0 captured immediately, then stuck at 0.4).
    driver, _, _ = _driver(session, yaws=[0.4] * 10, timeout=5.0, clock=clock)
    driver.start()
    assert driver.result().status is ScanSessionStatus.RUNNING
    clock.advance(6.0)  # past scan_timeout_s
    result = driver.result()
    assert result.status is ScanSessionStatus.FAILED
    assert result.reason is ExecutorReasonCode.SCAN_FAILED
    assert session.finalized_at is not None


def test_scan_session_driver_polled_before_start_raises():
    session = FakeSnapshotSession(snapshot=_snapshot())
    driver, _, _ = _driver(session, yaws=[0.0])
    with pytest.raises(ExecutorPortError):
        driver.result()


def test_scan_session_driver_resets_rotation_for_follow_up_cycle():
    session = FakeSnapshotSession(snapshot=_snapshot())
    driver, cmd, _ = _driver(
        session,
        yaws=[
            0.0, math.pi / 2, math.pi, -math.pi / 2,
            0.0, math.pi / 2, math.pi, -math.pi / 2,
        ],
    )

    driver.start()
    first = [driver.result() for _ in range(4)]
    driver.start()
    second = [driver.result() for _ in range(4)]

    assert first[-1].status is ScanSessionStatus.SNAPSHOT_READY
    assert second[-1].status is ScanSessionStatus.SNAPSHOT_READY
    assert [step for _, step in session.forwarded] == [
        "scan-step-0", "scan-step-1", "scan-step-2", "scan-step-3",
        "scan-step-0", "scan-step-1", "scan-step-2", "scan-step-3",
    ]
    assert cmd.count(pytest.approx(0.0)) == 2


def test_scan_session_start_failure_is_typed_instead_of_escaping():
    class BrokenStartSession(FakeSnapshotSession):
        def start(self, now_s):
            raise RuntimeError("already started")

    driver, cmd, _ = _driver(BrokenStartSession(), yaws=[])

    driver.start()
    result = driver.result()

    assert result.status is ScanSessionStatus.FAILED
    assert result.reason is ExecutorReasonCode.SCAN_FAILED
    assert driver.last_failure_detail == "RuntimeError('already started')"
    assert cmd == [0.0]
