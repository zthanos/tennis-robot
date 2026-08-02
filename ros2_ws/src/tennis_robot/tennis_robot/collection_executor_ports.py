"""ROS adapters for the pure Phase 4A executor ports (Phase 6C.1).

Each adapter implements one executor Protocol from
``collection_route_executor`` and returns EXACTLY its typed result values.
The design keeps decision logic pure and ROS-free: this module imports no
``rclpy`` and no ROS message types.  Every ROS touch-point is an injected,
duck-typed handle (a node, a publisher callable, a "latest message" provider)
supplied by the Phase 6C.2 node wiring, so the whole module — and its decision
logic — is unit-testable offline with plain fakes.

Ports implemented here (sensor/actuator half of 6C):

* :class:`RosMonotonicClock`         -> MonotonicClock
* :class:`CallbackTelemetrySink`     -> TelemetrySink
* :class:`ScanPoseNavigatorAdapter`  -> ScanPoseNavigator (wraps Nav2LaneNavigator)
* :class:`GazeboCollectorAdapter`    -> Collector (wraps CollectorInterface)
* :class:`LidarSafetyMonitor`        -> SafetyMonitor (/scan forward sector)
* :class:`ScanSessionDriver`         -> ScanSession (360deg step-rotate + snapshot)

The genuinely non-trivial logic (forward-sector obstacle test + blocked-duration
timeout, and the 360deg rotation-step FSM) lives in pure classes/functions that
take plain values, separate from the thin ROS wrappers.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import time

from tennis_robot.collection_route_executor import (
    CollectorStartResult,
    CollectorStartStatus,
    CollectorStopResult,
    CollectorStopStatus,
    ExecutorReasonCode,
    NavigatorResult,
    NavigatorStatus,
    SafetyResult,
    SafetyStatus,
    ScanSessionResult,
    ScanSessionStatus,
    TelemetryEvent,
)

_TWO_PI = 2.0 * math.pi


class ExecutorPortError(ValueError):
    """A port was configured with a missing or invalid required value."""


def _wrap_angle(angle: float) -> float:
    return (angle + math.pi) % _TWO_PI - math.pi


def _require_positive_finite(value: float, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0.0:
        raise ExecutorPortError(f"{name} must be a finite number > 0")
    return float(value)


def _require_finite(value: float, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ExecutorPortError(f"{name} must be a finite number")
    return float(value)


# ── 1. MonotonicClock ────────────────────────────────────────────────────────
class RosMonotonicClock:
    """MonotonicClock backed by an injected ROS node's clock."""

    def __init__(self, node) -> None:
        self._node = node

    def now_s(self) -> float:
        return self._node.get_clock().now().nanoseconds * 1e-9


# ── 2. TelemetrySink ─────────────────────────────────────────────────────────
def telemetry_event_to_dict(event: TelemetryEvent) -> dict:
    """Pure serialization of a TelemetryEvent to a robot_status.json-style dict."""
    if not isinstance(event, TelemetryEvent):
        raise ExecutorPortError("event must be a TelemetryEvent")
    payload = {
        "code": event.code.value,
        "state": event.state.value,
        "reason": event.reason.value if event.reason is not None else None,
    }
    if event.detail is not None:
        payload["detail"] = event.detail
    return payload


class CallbackTelemetrySink:
    """TelemetrySink that serializes each event and hands the dict to a callback."""

    def __init__(self, sink) -> None:
        if not callable(sink):
            raise ExecutorPortError("telemetry sink must be callable")
        self._sink = sink

    def emit(self, event: TelemetryEvent) -> None:
        self._sink(telemetry_event_to_dict(event))


# ── 3. ScanPoseNavigator ─────────────────────────────────────────────────────
# Pure mapping keyed on the plain state string so it never imports the ROS-bound
# LaneNavState enum (nav2_lane_navigator pulls in rclpy/nav2_msgs).
_LANE_NAV_RESULT = {
    "idle": NavigatorResult(NavigatorStatus.RUNNING),
    "pending": NavigatorResult(NavigatorStatus.RUNNING),
    "active": NavigatorResult(NavigatorStatus.RUNNING),
    "reached": NavigatorResult(NavigatorStatus.SUCCEEDED),
    "failed": NavigatorResult(NavigatorStatus.FAILED, ExecutorReasonCode.NAVIGATION_FAILED),
    "unavailable": NavigatorResult(NavigatorStatus.UNAVAILABLE, ExecutorReasonCode.NAVIGATION_UNAVAILABLE),
}


def navigator_result_for_state(state_value: str) -> NavigatorResult:
    """Map a Nav2LaneNavigator state string to the executor's NavigatorResult."""
    try:
        return _LANE_NAV_RESULT[state_value]
    except (KeyError, TypeError):
        raise ExecutorPortError(f"unknown lane navigation state {state_value!r}") from None


class ScanPoseNavigatorAdapter:
    """ScanPoseNavigator that drives a Nav2LaneNavigator to the scan pose.

    ``scan_pose`` is the (x_m, y_m, yaw_rad) centre of the current side's
    service line, supplied by the caller; this adapter does not compute it.
    """

    def __init__(self, *, lane_navigator, scan_pose) -> None:
        self._navigator = lane_navigator
        if len(scan_pose) != 3:
            raise ExecutorPortError("scan_pose must be (x_m, y_m, yaw_rad)")
        self._scan_pose = (
            _require_finite(scan_pose[0], "scan_pose.x_m"),
            _require_finite(scan_pose[1], "scan_pose.y_m"),
            _require_finite(scan_pose[2], "scan_pose.yaw_rad"),
        )

    def start(self) -> None:
        x_m, y_m, yaw_rad = self._scan_pose
        self._navigator.request(x_m, y_m, yaw_rad)

    def result(self) -> NavigatorResult:
        state_value = self._navigator.state.value
        if state_value == "unavailable":
            # The controller and web panel can be ready a few seconds before
            # Nav2's delayed lifecycle bring-up.  Keep the executor in its
            # explicit navigation state and retry instead of turning a normal
            # startup race into an immediate aborted_scan with 0/18 steps.
            self.start()
            return NavigatorResult(NavigatorStatus.RUNNING)
        return navigator_result_for_state(state_value)


# ── 4. Collector ─────────────────────────────────────────────────────────────
class GazeboCollectorAdapter:
    """Collector port over CollectorInterface for the Gazebo MVP.

    Gazebo has no collector health/jam/full/ready sensors, so start is reported
    READY immediately and there is never an ``active_fault``.  A route stop
    includes a bounded transfer drain: after ENTRY, the wheels keep running
    until CONFIRMED or the maximum drain timeout.  Real hardware will wire the
    driver's jam/full/health signals into ``start_result`` and ``active_fault``
    in a later phase.
    """

    def __init__(
        self,
        collector_interface,
        *,
        entry_beam_provider=None,
        confirmed_beam_provider=None,
        minimum_drain_s: float = 0.0,
        maximum_drain_s: float = 0.0,
        clock_fn=time.monotonic,
    ) -> None:
        if minimum_drain_s < 0.0 or maximum_drain_s < minimum_drain_s:
            raise ValueError("collector drain bounds must satisfy 0 <= minimum <= maximum")
        self._collector = collector_interface
        self._entry_beam_provider = entry_beam_provider or (lambda: False)
        self._confirmed_beam_provider = confirmed_beam_provider or (lambda: False)
        self._minimum_drain_s = float(minimum_drain_s)
        self._maximum_drain_s = float(maximum_drain_s)
        self._clock_fn = clock_fn
        self._last_entry = False
        self._last_confirmed = False
        self._entry_pending = False
        self._stop_requested_at_s: float | None = None
        self._stopped = True

    def start(self) -> None:
        self._last_entry = False
        self._last_confirmed = False
        self._entry_pending = False
        self._stop_requested_at_s = None
        self._stopped = False
        self._collector.start()

    def start_result(self) -> CollectorStartResult:
        return CollectorStartResult(CollectorStartStatus.READY)

    def active_fault(self) -> ExecutorReasonCode | None:
        self._sample_beams()
        return None

    def stop(self) -> None:
        self._sample_beams()
        if self._maximum_drain_s <= 0.0:
            self._stop_now()
            return
        if self._stop_requested_at_s is None:
            self._stop_requested_at_s = self._clock_fn()

    def stop_result(self) -> CollectorStopResult:
        if self._stopped:
            return CollectorStopResult(CollectorStopStatus.STOPPED)
        self._sample_beams()
        if self._stop_requested_at_s is None:
            return CollectorStopResult(CollectorStopStatus.STOPPING)
        elapsed_s = max(0.0, self._clock_fn() - self._stop_requested_at_s)
        minimum_elapsed = elapsed_s >= self._minimum_drain_s
        drain_complete = minimum_elapsed and not self._entry_pending
        drain_timed_out = elapsed_s >= self._maximum_drain_s
        if drain_complete or drain_timed_out:
            self._stop_now()
            return CollectorStopResult(CollectorStopStatus.STOPPED)
        return CollectorStopResult(CollectorStopStatus.STOPPING)

    def force_disable(self) -> None:
        self._stop_now()

    def _sample_beams(self) -> None:
        entry = bool(self._entry_beam_provider())
        confirmed = bool(self._confirmed_beam_provider())
        if entry and not self._last_entry:
            self._entry_pending = True
        if confirmed and not self._last_confirmed and self._entry_pending:
            self._entry_pending = False
        self._last_entry = entry
        self._last_confirmed = confirmed

    def _stop_now(self) -> None:
        if not self._stopped:
            self._collector.stop()
        self._stopped = True


# ── 5. SafetyMonitor (pure forward-sector logic + thin /scan wrapper) ─────────
def forward_sector_blocked(
    *,
    ranges,
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    forward_half_angle_rad: float,
    stop_distance_m: float,
) -> bool:
    """True iff any valid lidar return inside the forward sector is closer than
    ``stop_distance_m``.  A return is valid when it is finite and within
    [range_min, range_max]; the forward sector is |wrapped bearing| <= half."""
    if angle_increment == 0.0:
        return False
    for index, distance in enumerate(ranges):
        if not isinstance(distance, (int, float)) or not math.isfinite(distance):
            continue
        if distance < range_min or distance > range_max:
            continue
        bearing = angle_min + index * angle_increment
        if abs(_wrap_angle(bearing)) <= forward_half_angle_rad and distance < stop_distance_m:
            return True
    return False


@dataclass(frozen=True)
class ScanSample:
    """One lidar frame reduced to the plain values the safety logic needs."""

    stamp_s: float
    ranges: tuple[float, ...]
    angle_min: float
    angle_increment: float
    range_min: float
    range_max: float


class ForwardSectorSafetyLogic:
    """Pure SafetyResult decision: forward-sector obstacle + stale-scan watchdog.

    A block is raised when a valid forward return is closer than
    ``stop_distance_m`` OR when there is no fresh scan — a missing scan, or one
    older than ``max_scan_age_s``, is BLOCKED (fail-safe), never CLEAR.  A block
    sustained for at least ``safety_pause_timeout_s`` becomes TIMEOUT; otherwise
    CLEAR.  All thresholds are required (no defaults).
    """

    def __init__(
        self,
        *,
        forward_half_angle_rad: float,
        stop_distance_m: float,
        safety_pause_timeout_s: float,
        max_scan_age_s: float,
    ) -> None:
        self._forward_half_angle_rad = _require_positive_finite(forward_half_angle_rad, "forward_half_angle_rad")
        if self._forward_half_angle_rad > math.pi:
            raise ExecutorPortError("forward_half_angle_rad must be <= pi")
        self._stop_distance_m = _require_positive_finite(stop_distance_m, "stop_distance_m")
        self._safety_pause_timeout_s = _require_positive_finite(safety_pause_timeout_s, "safety_pause_timeout_s")
        self._max_scan_age_s = _require_positive_finite(max_scan_age_s, "max_scan_age_s")
        self._blocked_since_s: float | None = None

    def evaluate(self, *, scan: "ScanSample | None", now_s: float) -> SafetyResult:
        now_s = _require_finite(now_s, "now_s")
        blocked = self._is_blocked(scan, now_s)
        if not blocked:
            self._blocked_since_s = None
            return SafetyResult(SafetyStatus.CLEAR)
        if self._blocked_since_s is None:
            self._blocked_since_s = now_s
        if now_s - self._blocked_since_s >= self._safety_pause_timeout_s:
            return SafetyResult(SafetyStatus.TIMEOUT)
        return SafetyResult(SafetyStatus.BLOCKED)

    def _is_blocked(self, scan: "ScanSample | None", now_s: float) -> bool:
        # Fail-safe: no scan, or a scan older than max_scan_age_s, blocks.
        if scan is None or now_s - scan.stamp_s > self._max_scan_age_s:
            return True
        return forward_sector_blocked(
            ranges=scan.ranges,
            angle_min=scan.angle_min,
            angle_increment=scan.angle_increment,
            range_min=scan.range_min,
            range_max=scan.range_max,
            forward_half_angle_rad=self._forward_half_angle_rad,
            stop_distance_m=self._stop_distance_m,
        )


def _stamp_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class LidarSafetyMonitor:
    """SafetyMonitor port that feeds the latest /scan LaserScan into the pure logic.

    ``scan_provider`` returns the most recent ``sensor_msgs/LaserScan`` (or
    ``None`` if none received yet).  A missing or stale scan is fail-safe BLOCKED
    (see :class:`ForwardSectorSafetyLogic`).  The node owns the subscription;
    this wrapper never subscribes.
    """

    def __init__(self, *, logic: ForwardSectorSafetyLogic, clock, scan_provider) -> None:
        if not isinstance(logic, ForwardSectorSafetyLogic):
            raise ExecutorPortError("logic must be a ForwardSectorSafetyLogic")
        if not callable(scan_provider):
            raise ExecutorPortError("scan_provider must be callable")
        self._logic = logic
        self._clock = clock
        self._scan_provider = scan_provider

    def result(self) -> SafetyResult:
        now_s = self._clock.now_s()
        scan = self._scan_provider()
        sample = None
        if scan is not None:
            sample = ScanSample(
                stamp_s=_stamp_seconds(scan.header.stamp),
                ranges=tuple(scan.ranges),
                angle_min=scan.angle_min,
                angle_increment=scan.angle_increment,
                range_min=scan.range_min,
                range_max=scan.range_max,
            )
        return self._logic.evaluate(scan=sample, now_s=now_s)


# ── 6. ScanSession (pure rotation-step FSM + thin 360deg driver) ──────────────
class ScanRotationFsm:
    """Pure 360deg discrete rotation-step FSM.

    Steps are captured at yaw targets ``start + k*step_angle`` for
    ``k = 0 .. step_count-1``.  After the last distinct sample, the robot must
    return to ``start`` before the FSM is complete.  This closes the full turn
    geometrically and keeps the physical yaw equal to the yaw frozen into the
    planner snapshot.  ``observe(current_yaw)`` returns the step id only for a
    newly captured sample; the final return emits no duplicate sample.
    """

    def __init__(self, *, step_count: int, yaw_tolerance_rad: float, start_yaw_rad: float) -> None:
        if isinstance(step_count, bool) or not isinstance(step_count, int) or step_count < 1:
            raise ExecutorPortError("step_count must be a positive int")
        self._step_count = step_count
        self._tolerance = _require_positive_finite(yaw_tolerance_rad, "yaw_tolerance_rad")
        if self._tolerance >= math.pi:
            raise ExecutorPortError("yaw_tolerance_rad must be < pi")
        self._start = _require_finite(start_yaw_rad, "start_yaw_rad")
        self._step_angle = _TWO_PI / step_count
        self._captured = 0
        self._returned_to_start = False

    @property
    def expected_step_ids(self) -> tuple[str, ...]:
        return tuple(f"scan-step-{index}" for index in range(self._step_count))

    @property
    def is_complete(self) -> bool:
        return self._captured >= self._step_count and self._returned_to_start

    @property
    def target_yaw_rad(self) -> float | None:
        if self.is_complete:
            return None
        if self._captured >= self._step_count:
            return _wrap_angle(self._start)
        return _wrap_angle(self._start + self._captured * self._step_angle)

    def observe(self, current_yaw_rad: float) -> str | None:
        if self.is_complete:
            return None
        current_yaw_rad = _require_finite(current_yaw_rad, "current_yaw_rad")
        if abs(_wrap_angle(current_yaw_rad - self.target_yaw_rad)) <= self._tolerance:
            if self._captured >= self._step_count:
                self._returned_to_start = True
                return None
            step_id = f"scan-step-{self._captured}"
            self._captured += 1
            return step_id
        return None

    def reset(self) -> None:
        """Start a fresh rotation cycle with the same immutable scan geometry."""
        self._captured = 0
        self._returned_to_start = False


class ScanSessionDriver:
    """ScanSession port: rotate 360deg in steps, forward one detection frame per
    step to the snapshot session, then finalize into a ScanSnapshot.

    Injected duck-typed handles (the node wires them; this class touches no ROS):

    * ``fsm``: a :class:`ScanRotationFsm` (its ``expected_step_ids`` must equal
      the ones the ``snapshot_session`` was built with).
    * ``snapshot_session``: exposes ``forward_frame(frame, *, scan_step_id)`` and
      ``finalize(now_s) -> ScanSnapshot`` (raises on failure).
    * ``yaw_provider()``: latest robot yaw in rad, or ``None`` if unknown yet.
    * ``frame_provider()``: latest BallDetectionArray, or ``None``.
    * ``cmd_vel(angular_z)``: publishes an angular-only rotate command.
    * ``clock``: MonotonicClock.
    * ``angular_speed_rad_s`` / ``scan_timeout_s``: required, no defaults.
    """

    def __init__(
        self,
        *,
        fsm: ScanRotationFsm,
        snapshot_session,
        yaw_provider,
        frame_provider,
        cmd_vel,
        clock,
        angular_speed_rad_s: float,
        scan_timeout_s: float,
    ) -> None:
        if not isinstance(fsm, ScanRotationFsm):
            raise ExecutorPortError("fsm must be a ScanRotationFsm")
        for name, handle in (("yaw_provider", yaw_provider), ("frame_provider", frame_provider), ("cmd_vel", cmd_vel)):
            if not callable(handle):
                raise ExecutorPortError(f"{name} must be callable")
        self._fsm = fsm
        self._session = snapshot_session
        self._yaw_provider = yaw_provider
        self._frame_provider = frame_provider
        self._cmd_vel = cmd_vel
        self._clock = clock
        self._angular_speed_rad_s = _require_positive_finite(angular_speed_rad_s, "angular_speed_rad_s")
        self._scan_timeout_s = _require_positive_finite(scan_timeout_s, "scan_timeout_s")
        self._started_at_s: float | None = None
        self._terminal: ScanSessionResult | None = None
        # Diagnostic: the specific snapshot-builder failure code that produced a
        # SCAN_FAILED (e.g. scan_timeout vs insufficient_coverage). Surfaced by
        # the node when it logs an aborted_scan terminal.
        self.last_failure_detail: str | None = None

    @property
    def scan_diagnostics(self):
        """Builder telemetry (rejection histogram + per-track step counts) for
        explaining an empty/partial snapshot. None if the session does not
        expose it."""
        return getattr(self._session, "diagnostics", None)

    def start(self) -> None:
        self._started_at_s = self._clock.now_s()
        self._terminal = None
        self.last_failure_detail = None
        self._fsm.reset()
        start_session = getattr(self._session, "start", None)
        try:
            if callable(start_session):
                start_session(self._started_at_s)
        except Exception as exc:
            code = getattr(getattr(exc, "code", None), "value", None)
            self.last_failure_detail = f"{code}: {exc}" if code else repr(exc)
            self._terminal = ScanSessionResult(
                ScanSessionStatus.FAILED,
                reason=ExecutorReasonCode.SCAN_FAILED,
            )
            self._cmd_vel(0.0)
            return
        self._cmd_vel(self._angular_speed_rad_s)

    def result(self) -> ScanSessionResult:
        if self._terminal is not None:
            return self._terminal
        if self._started_at_s is None:
            raise ExecutorPortError("scan session polled before start()")

        yaw = self._yaw_provider()
        if yaw is not None:
            captured_id = self._fsm.observe(yaw)
            if captured_id is not None:
                frame = self._frame_provider()
                if frame is not None:
                    self._session.forward_frame(frame, scan_step_id=captured_id)

        if self._fsm.is_complete:
            self._cmd_vel(0.0)
            return self._finalize()

        # Fail loud rather than spin forever if the rotation never completes.
        if self._clock.now_s() - self._started_at_s > self._scan_timeout_s:
            self._cmd_vel(0.0)
            return self._finalize()

        self._cmd_vel(self._angular_speed_rad_s)
        return ScanSessionResult(ScanSessionStatus.RUNNING)

    def _finalize(self) -> ScanSessionResult:
        try:
            snapshot = self._session.finalize(self._clock.now_s())
        except Exception as exc:  # any snapshot-builder failure is a scan failure
            code = getattr(getattr(exc, "code", None), "value", None)
            self.last_failure_detail = f"{code}: {exc}" if code else repr(exc)
            self._terminal = ScanSessionResult(ScanSessionStatus.FAILED, reason=ExecutorReasonCode.SCAN_FAILED)
            return self._terminal
        self._terminal = ScanSessionResult(ScanSessionStatus.SNAPSHOT_READY, snapshot=snapshot)
        return self._terminal
