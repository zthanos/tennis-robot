"""Throwing Mode application service and robot/court orchestration.

This service owns session state (never the browser).  Its robot port is the
existing :class:`RosService`; tests inject a deterministic fake with the same
small surface.
"""

from __future__ import annotations

import json
import math
import threading
import time
from dataclasses import dataclass
from pathlib import Path

from tennis_robot.throwing_mode import (
    PROGRAMS,
    BasketState,
    FlywheelState,
    ProgramId,
    RobotReadiness,
    SessionState,
    ThrowConfiguration,
    TiltState,
    ThrowingError,
    ThrowingSession,
    session_to_mapping,
)


@dataclass(frozen=True)
class ThrowingPose:
    x_m: float
    y_m: float
    yaw_rad: float
    semantic_name: str = "THROWING_POSITION"


def resolve_throwing_pose(boundary: dict, robot_x_m: float, robot_y_m: float,
                          inset_m: float = 0.75) -> ThrowingPose:
    """Resolve a baseline-relative pose from the measured Court Knowledge Model."""
    try:
        net = boundary["net"]
        center = net["center"]
        axis = net["axis_length"]
        baselines = sorted(float(v) for v in boundary["court"]["lines_court_frame"]["baselines_x"])
        cx, cy = float(center["x_m"]), float(center["y_m"])
        ux, uy = float(axis["x_m"]), float(axis["y_m"])
    except (KeyError, TypeError, ValueError) as exc:
        raise ThrowingError(f"court model cannot resolve THROWING_POSITION: {exc}") from exc
    norm = math.hypot(ux, uy)
    if norm < 0.5 or not math.isfinite(norm):
        raise ThrowingError("court length axis is invalid")
    ux, uy = ux / norm, uy / norm
    robot_court_x = (robot_x_m - cx) * ux + (robot_y_m - cy) * uy
    baseline = baselines[0] if robot_court_x < 0.0 else baselines[-1]
    # Move inward toward the net, not into the run-off behind the baseline.
    court_x = baseline - math.copysign(inset_m, baseline)
    x_m, y_m = cx + court_x * ux, cy + court_x * uy
    toward_net_x = -math.copysign(1.0, court_x) * ux
    toward_net_y = -math.copysign(1.0, court_x) * uy
    return ThrowingPose(x_m, y_m, math.atan2(toward_net_y, toward_net_x))


class ThrowingService:
    POSITION_TOLERANCE_M = 0.30
    ORIENTATION_TOLERANCE_RAD = math.radians(8.0)
    STATIONARY_TOLERANCE_MPS = 0.04
    # How early the feed request is handed to the publisher, which then
    # waits out the remainder and emits on the beat. Must exceed the
    # publisher's discovery cost (measured 1.5-3.1 s).
    FEED_EMISSION_LEAD_S = 3.6
    # Session states in which the drivetrain, basket carriage and launcher all
    # belong to Throwing Mode. Collection must not drive the robot from under a
    # spinning launcher, which is the software half of the doc's
    # "collect ⇒ launcher inhibited / launch ⇒ intake disabled" interlock.
    MACHINE_OWNED_STATES = frozenset({
        SessionState.POSITIONING, SessionState.RAISING_BASKET, SessionState.ARMING,
        SessionState.READY, SessionState.THROWING, SessionState.PAUSED,
        SessionState.STOPPING,
    })

    def __init__(self, *, ros, status_store, sensor_store, boundary_path: Path, camera) -> None:
        self._ros = ros
        self._status_store = status_store
        self._sensor_store = sensor_store
        self._boundary_path = boundary_path
        self._camera = camera
        self._lock = threading.RLock()
        self._session = ThrowingSession(ThrowConfiguration.defaults(ProgramId.FOREHAND))
        self._basket_state = BasketState.UNKNOWN
        # No tilt axis is modelled; see TiltState. Held as state (not a
        # constant) so a validated tilt mechanism only has to write here.
        self._basket_tilt_state = TiltState.MECHANICAL_VALIDATION_PENDING
        self._basket_requested = BasketState.LOWERED
        self._flywheel_state = FlywheelState.UNAVAILABLE
        self._positioned = False
        self._orientation_ok = False
        self._active_thread: threading.Thread | None = None
        self._probe_thread: threading.Thread | None = None
        self._stop_requested = threading.Event()
        self._last_throw: dict[str, object] | None = None
        self._last_error: str | None = None

    # ------------------------------------------------------------------
    # Machine-ownership interlocks (both directions)
    # ------------------------------------------------------------------
    def machine_owner_conflict(self) -> str | None:
        """Why another subsystem may not drive the robot, or None if it may."""
        with self._lock:
            if self._session.state in self.MACHINE_OWNED_STATES:
                return (
                    "Throwing Mode owns the robot "
                    f"(session {self._session.state.value}); stop it first."
                )
            if self._active_thread is not None and self._active_thread.is_alive():
                return "A Throwing Mode basket movement is still in progress."
        return None

    def _collector_conflict(self) -> str | None:
        """The launcher must not arm while the intake is running."""
        status = self._ros.collector_status()
        if isinstance(status, dict) and status.get("running"):
            return "The collector is running; stop it before arming the launcher."
        return None

    def programs(self) -> list[dict[str, object]]:
        return [{
            "id": item.program_id.value, "name": item.name,
            "target_zones": [zone.value for zone in item.target_zones],
            "defaults": {
                "throw_speed_mps": item.throw_speed_mps,
                "throw_angle_deg": item.throw_angle_deg,
                "interval_s": item.interval_s,
                "ball_count": item.ball_count,
            },
            "placement_strategy": item.placement_strategy,
        } for item in PROGRAMS.values()]

    def _robot_status(self) -> dict:
        return self._status_store.read()

    def _readiness(self) -> RobotReadiness:
        robot = self._robot_status()
        simulated = self._ros.runtime_kind() == "simulation"
        speed = robot.get("measured_speed_mps")
        stationary = isinstance(speed, (int, float)) and abs(speed) <= self.STATIONARY_TOLERANCE_MPS
        # Simulation has an explicit software E-stop/motor-power boundary.  Real
        # hardware must publish confirmed fields; absence is deliberately false.
        estop_clear = bool(robot.get("estop_clear", simulated))
        motor_power = bool(robot.get("motor_power_available", simulated))
        blocking_fault = robot.get("blocking_hardware_fault")
        hopper_count = robot.get("hopper_ball_count")
        if not isinstance(hopper_count, int) or hopper_count < 0:
            hopper_count = None
        return RobotReadiness(
            positioned=self._positioned,
            stationary=stationary,
            orientation_ok=self._orientation_ok,
            basket_state=self._basket_state,
            basket_tilt_state=self._basket_tilt_state,
            flywheel_state=self._flywheel_state,
            motor_power_available=motor_power,
            estop_clear=estop_clear,
            blocking_fault=str(blocking_fault) if blocking_fault else None,
            hopper_ball_count=hopper_count,
            hopper_count_source="MEASURED" if hopper_count is not None else "UNKNOWN",
        )

    def status(self) -> dict[str, object]:
        with self._lock:
            if (self._basket_state == BasketState.UNKNOWN
                    or self._flywheel_state == FlywheelState.UNAVAILABLE):
                if self._probe_thread is None or not self._probe_thread.is_alive():
                    self._probe_thread = threading.Thread(
                        target=self._refresh_actuator_feedback,
                        daemon=True,
                        name="throwing-actuator-feedback",
                    )
                    self._probe_thread.start()
            readiness = self._readiness()
            sensors = self._sensor_store.read()
            front_camera = sensors.get("front_camera") if isinstance(sensors, dict) else None
            camera_ready = bool(isinstance(front_camera, dict) and front_camera.get("data_url"))
            return {
                "ok": True,
                "capability": {
                    "runtime": self._ros.runtime_kind().upper(),
                    "basket_lift": "SIMULATION_READY" if self._ros.runtime_kind() == "simulation" else "PHYSICAL_HARDWARE_PENDING",
                    "basket_tilt": self._basket_tilt_state.value,
                    "flywheel": (
                        "SOFTWARE_READY"
                        if self._flywheel_state != FlywheelState.UNAVAILABLE
                        else "HARDWARE_INTERFACE_PENDING"
                    ),
                    "ball_feed": "PLACEHOLDER_EVENT_SIMULATION",
                    "analytics": "PERCEPTION_PENDING",
                },
                "programs": self.programs(),
                "session": session_to_mapping(self._session),
                "readiness": {
                    "positioned": readiness.positioned,
                    "stationary": readiness.stationary,
                    "orientation_ok": readiness.orientation_ok,
                    "basket_state": readiness.basket_state.value,
                    "basket_tilt_state": readiness.basket_tilt_state.value,
                    "throwing_pose_confirmed": readiness.throwing_pose_confirmed,
                    "basket_requested": self._basket_requested.value,
                    "flywheel_state": readiness.flywheel_state.value,
                    "estop_clear": readiness.estop_clear,
                    "motor_power_available": readiness.motor_power_available,
                    "blocking_fault": readiness.blocking_fault,
                    "throw_blockers": list(readiness.throw_blockers),
                    "hopper_ball_count": readiness.hopper_ball_count,
                    "hopper_count_source": readiness.hopper_count_source,
                    "oak_d_state": "READY" if camera_ready else (
                        "READY" if self._camera.available else "NOT_CONNECTED"
                    ),
                },
                "last_throw": self._last_throw,
                "last_error": self._last_error,
            }

    def _refresh_actuator_feedback(self) -> None:
        basket_state = BasketState.UNKNOWN
        try:
            basket_state = BasketState(self._ros.basket_position_state())
        except (AttributeError, ValueError):
            pass
        flywheel_available = self._ros.flywheel_available()
        with self._lock:
            if self._basket_state == BasketState.UNKNOWN:
                self._basket_state = basket_state
            if self._flywheel_state == FlywheelState.UNAVAILABLE and flywheel_available:
                self._flywheel_state = FlywheelState.IDLE

    @staticmethod
    def _configuration(data: dict) -> ThrowConfiguration:
        try:
            program = ProgramId(str(data.get("program", ProgramId.FOREHAND.value)))
        except ValueError as exc:
            raise ThrowingError("unknown training program") from exc
        defaults = ThrowConfiguration.defaults(program)
        try:
            return ThrowConfiguration(
                program,
                float(data.get("throw_speed_mps", defaults.throw_speed_mps)),
                float(data.get("throw_angle_deg", defaults.throw_angle_deg)),
                float(data.get("interval_s", defaults.interval_s)),
                int(data.get("ball_count", defaults.ball_count)),
            )
        except (TypeError, ValueError) as exc:
            raise ThrowingError(f"invalid throw configuration: {exc}") from exc

    def start(self, data: dict, *, test_throw: bool = False) -> dict[str, object]:
        with self._lock:
            if self._active_thread is not None and self._active_thread.is_alive():
                return {"ok": False, "message": "a preparation/session sequence is already active"}
            if test_throw and self._session.state == SessionState.FAULT:
                return {"ok": False, "message": "Test Throw is blocked while the session is faulted"}
            conflict = self._collector_conflict()
            if conflict:
                return {"ok": False, "message": conflict}
            try:
                config = self._configuration(data)
                if test_throw:
                    config = ThrowConfiguration(config.program_id, config.throw_speed_mps,
                                                config.throw_angle_deg, config.interval_s, 1)
                self._session = ThrowingSession(config)
                self._session.start(self._readiness())
            except ThrowingError as exc:
                return {"ok": False, "message": str(exc)}
            self._positioned = False
            self._orientation_ok = False
            self._last_error = None
            self._stop_requested.clear()
            self._active_thread = threading.Thread(
                target=self._run_session, args=(test_throw,), daemon=True,
                name="throwing-mode-session",
            )
            self._active_thread.start()
            return {"ok": True, "session_id": self._session.session_id,
                    "state": self._session.state.value, "test_throw": test_throw}

    def _run_session(self, test_throw: bool) -> None:
        try:
            robot = self._robot_status()
            boundary = json.loads(self._boundary_path.read_text(encoding="utf-8"))
            pose = resolve_throwing_pose(
                boundary, float(robot["robot_x_m"]), float(robot["robot_y_m"])
            )
            nav = self._ros.navigate_to_pose(pose.x_m, pose.y_m, pose.yaw_rad)
            if self._stop_requested.is_set(): return
            if not nav.get("succeeded"):
                raise ThrowingError(nav.get("message") or "navigation did not succeed")
            positioned, stationary, orientation_ok = self._confirm_navigation_pose(pose)
            if positioned and not orientation_ok and not self._stop_requested.is_set():
                # Nav2 delivers the position but not the heading (see
                # RosService.rotate_by). Close the heading once, then re-measure
                # — the gate below still decides, so a failed correction faults
                # the session instead of arming a robot pointing the wrong way.
                orientation_ok = self._align_heading(pose)
                positioned, stationary, orientation_ok = self._confirm_navigation_pose(pose)
            self._positioned = positioned
            self._orientation_ok = orientation_ok
            with self._lock:
                self._session.navigation_confirmed(
                    reached=positioned, stationary=stationary,
                    orientation_ok=orientation_ok,
                    basket_state=self._basket_state,
                )
            if self._session.state == SessionState.RAISING_BASKET:
                if not self._move_basket(BasketState.RAISED):
                    raise ThrowingError("basket failed to reach raised position")
                if self._stop_requested.is_set(): return
                with self._lock: self._session.basket_confirmed(self._basket_state)
            if self._stop_requested.is_set(): return
            self._flywheel_state = FlywheelState.SPINNING_UP
            if not self._ros.set_flywheel_speed(self._session.configuration.throw_speed_mps):
                self._flywheel_state = FlywheelState.UNAVAILABLE
                raise ThrowingError("flywheel controller is unavailable")
            flywheel_ready = self._ros.wait_flywheel_ready(
                self._session.configuration.throw_speed_mps
            )
            if self._stop_requested.is_set(): return
            if not flywheel_ready:
                raise ThrowingError("flywheel readiness was not confirmed")
            self._flywheel_state = FlywheelState.READY
            with self._lock: self._session.flywheel_confirmed_ready(self._readiness())
            # `interval_s` is the launch-to-launch PERIOD, not a rest between
            # throws: each throw is scheduled at previous_launch + interval, so
            # the time the feed request itself takes is absorbed by the period
            # instead of being added to it. Measuring the period from the
            # instant the feed request is issued (rather than from when it
            # returns) is what makes the cadence observed at the consumer equal
            # the configured value, because every request spends the same setup
            # time before it publishes.
            #
            # monotonic() deliberately: this is application cadence, not
            # simulation time, and it must not jump if the wall clock is
            # adjusted.
            next_launch_at: float | None = None
            configuration_interval = self._session.configuration.interval_s
            # The publisher needs this much time to discover the consumer before
            # it can emit on the beat; clamped so short intervals still work.
            emission_lead = min(self.FEED_EMISSION_LEAD_S, 0.9 * configuration_interval)
            while self._session.statistics.balls_thrown < self._session.configuration.ball_count:
                if self._stop_requested.is_set(): break
                if self._session.state == SessionState.PAUSED:
                    # Suspend the interval clock instead of letting it run down,
                    # so resuming neither fires a burst of "overdue" throws nor
                    # makes the operator wait out a fresh full interval.
                    remaining_at_pause = (
                        None if next_launch_at is None
                        else max(0.0, next_launch_at - time.monotonic())
                    )
                    while (self._session.state == SessionState.PAUSED
                           and not self._stop_requested.wait(0.1)):
                        pass
                    if self._stop_requested.is_set(): break
                    if remaining_at_pause is not None:
                        next_launch_at = time.monotonic() + remaining_at_pause
                    continue
                if next_launch_at is not None:
                    # Hand off to the feed publisher FEED_EMISSION_LEAD_S before
                    # the launch instant: it needs that time to discover the
                    # consumer, and it then waits out the remainder itself so the
                    # event is emitted exactly on the beat.
                    remaining = (next_launch_at - emission_lead) - time.monotonic()
                    if remaining > 0:
                        # Short slices so Pause and Stop stay responsive during
                        # the interval.
                        self._stop_requested.wait(min(remaining, 0.1))
                        continue
                readiness = self._readiness()
                with self._lock:
                    throw_id, target = self._session.prepare_throw(readiness)
                    configuration = self._session.configuration
                # Always leave the publisher its full lead, including for the
                # first throw: emitting as soon as possible would set the beat
                # from a late first emission and drag the whole cadence with it.
                earliest = time.monotonic() + emission_lead
                launch_at = earliest if next_launch_at is None else max(next_launch_at, earliest)
                # Rebase from the ACTUAL launch instant, never from a missed
                # deadline: a late throw shifts the cadence forward, it never
                # accumulates a debt that later fires as a catch-up burst.
                next_launch_at = launch_at + configuration.interval_s
                if not self._ros.request_ball_feed(
                    session_id=self._session.session_id,
                    throw_id=throw_id,
                    target_zone=target.value,
                    throw_speed_mps=configuration.throw_speed_mps,
                    throw_angle_deg=configuration.throw_angle_deg,
                    publish_at_unix=time.time() + (launch_at - time.monotonic()),
                ):
                    raise ThrowingError("ball feed request was not accepted")
                with self._lock:
                    if self._session.state in {SessionState.COMPLETED, SessionState.FAULT}:
                        # Stop landed while this feed request was in flight. The
                        # event was already emitted, but the session is already
                        # closed — that is a clean stop, not a fault. (A PAUSE
                        # landing mid-feed is different: the throw still counts,
                        # see ThrowingSession.confirm_successful_throw.)
                        break
                    self._session.confirm_successful_throw(readiness, throw_id, target)
                    self._last_throw = {"throw_id": throw_id, "target_zone": target.value,
                                        "source": "PLACEHOLDER_EVENT_SIMULATION",
                                        "at": time.time()}
                if test_throw: break
            self._safe_stop(completed=True)
        except (OSError, KeyError, TypeError, ValueError, ThrowingError) as exc:
            # Stop the actuators BEFORE taking the lock: a concurrent status()
            # poll must not wait behind actuator I/O while the machine is still
            # driving, and the carriage may have aborted mid-move.
            self._ros.set_flywheel_speed(0.0)
            self._ros.stop_basket()
            with self._lock:
                self._last_error = str(exc)
                if self._session.state not in {SessionState.COMPLETED, SessionState.FAULT}:
                    self._session.fault(str(exc))

    def _align_heading(self, pose: ThrowingPose) -> bool:
        """Rotate in place to close the residual yaw error, once."""
        robot = self._robot_status()
        try:
            yaw_rad = float(robot["robot_yaw_rad"])
        except (KeyError, TypeError, ValueError):
            return False
        error = math.atan2(math.sin(pose.yaw_rad - yaw_rad),
                           math.cos(pose.yaw_rad - yaw_rad))
        if abs(error) <= self.ORIENTATION_TOLERANCE_RAD:
            return True
        # Aim inside the gate's tolerance so a correction that only just lands
        # on the boundary does not fail the re-measurement that follows.
        return self._ros.align_heading(
            pose.yaw_rad, 0.6 * self.ORIENTATION_TOLERANCE_RAD
        )

    def _confirm_navigation_pose(self, pose: ThrowingPose,
                                 timeout_s: float = 3.0) -> tuple[bool, bool, bool]:
        """Confirm the goal against measured robot telemetry after Nav2 succeeds."""
        deadline = time.monotonic() + timeout_s
        result = (False, False, False)
        while time.monotonic() < deadline:
            robot = self._robot_status()
            try:
                x_m = float(robot["robot_x_m"])
                y_m = float(robot["robot_y_m"])
                yaw_rad = float(robot["robot_yaw_rad"])
                speed_mps = float(robot["measured_speed_mps"])
            except (KeyError, TypeError, ValueError):
                time.sleep(0.1)
                continue
            positioned = math.hypot(x_m - pose.x_m, y_m - pose.y_m) <= self.POSITION_TOLERANCE_M
            yaw_error = abs(math.atan2(math.sin(yaw_rad - pose.yaw_rad),
                                       math.cos(yaw_rad - pose.yaw_rad)))
            orientation_ok = yaw_error <= self.ORIENTATION_TOLERANCE_RAD
            stationary = abs(speed_mps) <= self.STATIONARY_TOLERANCE_MPS
            result = positioned, stationary, orientation_ok
            if all(result):
                return result
            time.sleep(0.1)
        return result

    def _safe_stop(self, *, completed: bool) -> None:
        self._ros.set_flywheel_speed(0.0)
        self._ros.stop_basket()
        self._flywheel_state = FlywheelState.IDLE if self._ros.flywheel_available() else FlywheelState.UNAVAILABLE
        with self._lock:
            if completed and self._session.state not in {SessionState.COMPLETED, SessionState.FAULT}:
                self._session.complete()

    def pause(self) -> dict[str, object]:
        with self._lock:
            try: self._session.pause()
            except ThrowingError as exc: return {"ok": False, "message": str(exc)}
            return {"ok": True, "state": self._session.state.value}

    def resume(self) -> dict[str, object]:
        with self._lock:
            try: self._session.resume()
            except ThrowingError as exc: return {"ok": False, "message": str(exc)}
            return {"ok": True, "state": self._session.state.value}

    def stop(self) -> dict[str, object]:
        with self._lock:
            if self._session.state in {SessionState.IDLE, SessionState.COMPLETED}:
                return {"ok": False, "message": f"cannot stop from {self._session.state.value}"}
        self._stop_requested.set()
        self._ros.nav_cancel()
        self._safe_stop(completed=True)
        return {"ok": True, "state": self._session.state.value}

    def basket_command(self, raised: bool) -> dict[str, object]:
        with self._lock:
            if self._session.state == SessionState.THROWING:
                return {"ok": False, "message": "basket movement is blocked during throwing"}
            # Moving the carriage swings the whole loaded basket through the
            # launcher's feed interface, so the wheels must be at rest first —
            # not merely "not currently throwing".
            if self._flywheel_state not in {FlywheelState.UNAVAILABLE, FlywheelState.IDLE}:
                return {"ok": False, "message":
                        f"basket movement is blocked while the flywheels are "
                        f"{self._flywheel_state.value}"}
            target = BasketState.RAISED if raised else BasketState.LOWERED
            if self._active_thread is not None and self._active_thread.is_alive():
                return {"ok": False, "message": "basket is owned by the active preparation sequence"}
            # Lowering is always allowed — it is the safe direction. Raising is
            # launch preparation and must not happen with the intake running.
            if raised:
                conflict = self._collector_conflict()
                if conflict:
                    return {"ok": False, "message": conflict}
            self._active_thread = threading.Thread(
                target=self._manual_basket_worker, args=(target,), daemon=True,
                name="basket-manual-command",
            )
            self._active_thread.start()
            return {"ok": True, "requested": target.value}

    def _manual_basket_worker(self, target: BasketState) -> None:
        if not self._move_basket(target): self._last_error = "basket position was not confirmed"

    def _move_basket(self, target: BasketState) -> bool:
        self._basket_requested = target
        self._basket_state = BasketState.RAISING if target == BasketState.RAISED else BasketState.LOWERING
        confirmed = self._ros.set_basket_position(target == BasketState.RAISED)
        self._basket_state = target if confirmed else BasketState.FAULT
        return confirmed
