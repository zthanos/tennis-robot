"""Pure domain model for the first Throwing Mode vertical slice.

The module deliberately contains no ROS or HTTP code.  The console and future
robot-native behaviour node can therefore share the same transition and
readiness rules while supplying different actuator/navigation ports.
"""

from __future__ import annotations

import math
import time
import uuid
from dataclasses import dataclass, field
from enum import Enum


class ThrowingError(ValueError):
    pass


class ProgramId(str, Enum):
    FOREHAND = "forehand"
    BACKHAND = "backhand"
    MIXED = "mixed"


class TargetZone(str, Enum):
    FOREHAND = "forehand_zone"
    BACKHAND = "backhand_zone"


class BasketState(str, Enum):
    """State of the basket LIFT axis only.

    RAISED means the lift axis reached its configured raised endpoint (the CAD
    baseline is 100 mm of travel). It is NOT the complete physical throwing
    pose: the mechanical baseline is 100 mm lift PLUS approximately 12 deg of
    tilt, and gravity feed needs both. See TiltState.
    """

    LOWERED = "LOWERED"
    RAISING = "RAISING"
    RAISED = "RAISED"
    LOWERING = "LOWERING"
    UNKNOWN = "UNKNOWN"
    FAULT = "FAULT"


class TiltState(str, Enum):
    """State of the basket TILT axis.

    No tilt joint is modelled: the tilt mechanism is not mechanically validated
    and the required angle is itself still an open question (gate L4), so
    inventing an actuator would fabricate confidence. The axis is represented
    anyway so the throwing pose is already expressed as
    ``lift_confirmed AND tilt_confirmed`` — when a validated tilt mechanism
    arrives it reports CONFIRMED/FAULT here and nothing in the session state
    machine changes.
    """

    MECHANICAL_VALIDATION_PENDING = "MECHANICAL_VALIDATION_PENDING"
    CONFIRMED = "CONFIRMED"
    FAULT = "FAULT"


class FlywheelState(str, Enum):
    UNAVAILABLE = "NOT_AVAILABLE"
    IDLE = "IDLE"
    SPINNING_UP = "SPINNING_UP"
    READY = "READY"
    FAULT = "FAULT"


class SessionState(str, Enum):
    IDLE = "IDLE"
    POSITIONING = "POSITIONING"
    RAISING_BASKET = "RAISING_BASKET"
    ARMING = "ARMING"
    READY = "READY"
    THROWING = "THROWING"
    PAUSED = "PAUSED"
    STOPPING = "STOPPING"
    COMPLETED = "COMPLETED"
    FAULT = "FAULT"


@dataclass(frozen=True)
class TrainingProgram:
    program_id: ProgramId
    name: str
    target_zones: tuple[TargetZone, ...]
    throw_speed_mps: float
    throw_angle_deg: float
    interval_s: float
    ball_count: int
    placement_strategy: str

    def target_for_index(self, throw_index: int) -> TargetZone:
        if throw_index < 0:
            raise ThrowingError("throw index cannot be negative")
        return self.target_zones[throw_index % len(self.target_zones)]


PROGRAMS: dict[ProgramId, TrainingProgram] = {
    ProgramId.FOREHAND: TrainingProgram(
        ProgramId.FOREHAND, "Forehand Training", (TargetZone.FOREHAND,),
        18.0, 20.0, 4.0, 12, "fixed_forehand_zone",
    ),
    ProgramId.BACKHAND: TrainingProgram(
        ProgramId.BACKHAND, "Backhand Training", (TargetZone.BACKHAND,),
        18.0, 20.0, 4.0, 12, "fixed_backhand_zone",
    ),
    ProgramId.MIXED: TrainingProgram(
        ProgramId.MIXED, "Mixed Training",
        (TargetZone.FOREHAND, TargetZone.BACKHAND),
        18.0, 20.0, 4.0, 12, "deterministic_alternating",
    ),
}


@dataclass(frozen=True)
class ThrowConfiguration:
    program_id: ProgramId
    throw_speed_mps: float
    throw_angle_deg: float
    interval_s: float
    ball_count: int

    def __post_init__(self) -> None:
        values = (self.throw_speed_mps, self.throw_angle_deg, self.interval_s)
        if not all(isinstance(value, (int, float)) and math.isfinite(value) for value in values):
            raise ThrowingError("throw parameters must be finite")
        if not 1.0 <= self.throw_speed_mps <= 35.0:
            raise ThrowingError("throw speed must be between 1 and 35 m/s")
        if not 0.0 <= self.throw_angle_deg <= 60.0:
            raise ThrowingError("throw angle must be between 0 and 60 degrees")
        if not 0.25 <= self.interval_s <= 60.0:
            raise ThrowingError("throw interval must be between 0.25 and 60 seconds")
        if not 1 <= self.ball_count <= 200:
            raise ThrowingError("ball count must be between 1 and 200")

    @classmethod
    def defaults(cls, program_id: ProgramId) -> "ThrowConfiguration":
        program = PROGRAMS[program_id]
        return cls(program_id, program.throw_speed_mps, program.throw_angle_deg,
                   program.interval_s, program.ball_count)


@dataclass(frozen=True)
class RobotReadiness:
    positioned: bool = False
    stationary: bool = False
    orientation_ok: bool = False
    basket_state: BasketState = BasketState.UNKNOWN
    basket_tilt_state: TiltState = TiltState.MECHANICAL_VALIDATION_PENDING
    flywheel_state: FlywheelState = FlywheelState.UNAVAILABLE
    motor_power_available: bool = False
    estop_clear: bool = False
    blocking_fault: str | None = None
    hopper_ball_count: int | None = None
    hopper_count_source: str = "UNKNOWN"

    @property
    def lift_confirmed(self) -> bool:
        return self.basket_state == BasketState.RAISED

    @property
    def tilt_confirmed(self) -> bool:
        """True while no tilt axis exists to confirm.

        This is deliberately not a silent pass: with the tilt mechanism
        unvalidated the simulated throwing pose IS the lift endpoint, and the
        status surface reports MECHANICAL_VALIDATION_PENDING so nobody reads it
        as a confirmed physical pose. Once a tilt axis is modelled, only
        CONFIRMED satisfies this and throwing_pose_confirmed tightens on its own.
        """
        if self.basket_tilt_state is TiltState.MECHANICAL_VALIDATION_PENDING:
            return True
        return self.basket_tilt_state is TiltState.CONFIRMED

    @property
    def throwing_pose_confirmed(self) -> bool:
        return self.lift_confirmed and self.tilt_confirmed

    @property
    def throw_blockers(self) -> tuple[str, ...]:
        blockers: list[str] = []
        if not self.positioned: blockers.append("robot_not_positioned")
        if not self.stationary: blockers.append("robot_not_stationary")
        if not self.orientation_ok: blockers.append("orientation_not_confirmed")
        if not self.lift_confirmed: blockers.append("basket_not_raised")
        if not self.tilt_confirmed: blockers.append("basket_tilt_not_confirmed")
        if self.flywheel_state != FlywheelState.READY: blockers.append("flywheel_not_ready")
        if not self.motor_power_available: blockers.append("motor_power_unavailable")
        if not self.estop_clear: blockers.append("estop_not_confirmed_clear")
        if self.blocking_fault: blockers.append(f"blocking_fault:{self.blocking_fault}")
        if self.hopper_ball_count == 0: blockers.append("hopper_empty")
        return tuple(blockers)


@dataclass(frozen=True)
class PerceptionEvent:
    event_type: str
    throw_id: str
    world_distance_m: float | None = None
    outcome: str | None = None


@dataclass
class SessionStatistics:
    balls_thrown: int = 0
    balls_hit: int | None = None
    balls_in: int | None = None
    balls_out: int | None = None
    average_distance_m: float | None = None

    @property
    def hit_rate(self) -> float | None:
        if self.balls_hit is None or self.balls_thrown <= 0:
            return None
        return self.balls_hit / self.balls_thrown


@dataclass
class ThrowingSession:
    configuration: ThrowConfiguration
    session_id: str = field(default_factory=lambda: str(uuid.uuid4()))
    state: SessionState = SessionState.IDLE
    started_at: float | None = None
    ended_at: float | None = None
    successful_throw_ids: list[str] = field(default_factory=list)
    perception_events: list[PerceptionEvent] = field(default_factory=list)
    statistics: SessionStatistics = field(default_factory=SessionStatistics)
    fault_reason: str | None = None
    paused_from: SessionState | None = None

    @property
    def program(self) -> TrainingProgram:
        return PROGRAMS[self.configuration.program_id]

    @property
    def next_target(self) -> TargetZone:
        return self.program.target_for_index(self.statistics.balls_thrown)

    def start(self, readiness: RobotReadiness, now: float | None = None) -> None:
        if self.state not in {SessionState.IDLE, SessionState.COMPLETED}:
            raise ThrowingError(f"cannot start from {self.state.value}")
        if readiness.blocking_fault:
            raise ThrowingError("blocking hardware fault")
        if not readiness.estop_clear:
            raise ThrowingError("E-stop clear state is not confirmed")
        if readiness.hopper_ball_count == 0:
            raise ThrowingError("hopper is known empty")
        self.started_at = now if now is not None else time.time()
        self.ended_at = None
        self.successful_throw_ids.clear()
        self.perception_events.clear()
        self.statistics = SessionStatistics()
        self.fault_reason = None
        self.state = SessionState.POSITIONING

    def navigation_confirmed(self, *, reached: bool, stationary: bool, orientation_ok: bool,
                             basket_state: BasketState) -> None:
        if self.state != SessionState.POSITIONING:
            raise ThrowingError("navigation confirmation is only valid while positioning")
        if not reached or not stationary or not orientation_ok:
            raise ThrowingError("position, stationary state, and orientation must all be confirmed")
        self.state = (SessionState.ARMING if basket_state == BasketState.RAISED
                      else SessionState.RAISING_BASKET)

    def basket_confirmed(self, state: BasketState) -> None:
        if self.state != SessionState.RAISING_BASKET:
            raise ThrowingError("basket confirmation is not expected")
        if state != BasketState.RAISED:
            raise ThrowingError("basket is not confirmed raised")
        self.state = SessionState.ARMING

    def flywheel_confirmed_ready(self, readiness: RobotReadiness) -> None:
        if self.state != SessionState.ARMING:
            raise ThrowingError("flywheel readiness is only valid while arming")
        blockers = readiness.throw_blockers
        if blockers:
            raise ThrowingError("not ready: " + ", ".join(blockers))
        self.state = SessionState.READY

    def record_successful_throw(self, readiness: RobotReadiness) -> tuple[str, TargetZone]:
        throw_id, target = self.prepare_throw(readiness)
        return self.confirm_successful_throw(readiness, throw_id, target)

    def prepare_throw(self, readiness: RobotReadiness) -> tuple[str, TargetZone]:
        """Reserve correlation metadata without counting an unaccepted feed."""
        if self.state not in {SessionState.READY, SessionState.THROWING}:
            raise ThrowingError(f"cannot throw from {self.state.value}")
        blockers = readiness.throw_blockers
        if blockers:
            raise ThrowingError("throw blocked: " + ", ".join(blockers))
        return str(uuid.uuid4()), self.next_target

    def confirm_successful_throw(self, readiness: RobotReadiness, throw_id: str,
                                 target: TargetZone) -> tuple[str, TargetZone]:
        """Count a prepared throw only after its feed request is accepted.

        PAUSED is accepted here but NOT in prepare_throw. Pausing is what stops
        the next throw from starting; a throw whose feed request was already
        emitted has physically happened, so refusing to count it would lose the
        throw and leave an uncorrelated feed event on the wire. A pause landing
        during the feed request therefore keeps the session paused and still
        records the throw that was already in flight.
        """
        resumable = self.state is SessionState.PAUSED
        if self.state not in {SessionState.READY, SessionState.THROWING,
                              SessionState.PAUSED}:
            raise ThrowingError(f"cannot throw from {self.state.value}")
        blockers = readiness.throw_blockers
        if blockers:
            raise ThrowingError("throw blocked: " + ", ".join(blockers))
        if not throw_id or throw_id in self.successful_throw_ids:
            raise ThrowingError("throw_id must be unique")
        if target != self.next_target:
            raise ThrowingError("prepared target no longer matches the session sequence")
        if resumable:
            # Stay paused; resume() must still return to a throwing state.
            self.paused_from = SessionState.THROWING
        else:
            self.state = SessionState.THROWING
        self.successful_throw_ids.append(throw_id)
        self.statistics.balls_thrown += 1
        return throw_id, target

    def add_perception_event(self, event: PerceptionEvent) -> bool:
        if event.throw_id not in self.successful_throw_ids:
            return False
        self.perception_events.append(event)
        # These counters remain unavailable until at least one correlated
        # source event supplies that class of result.
        if event.event_type == "player_contact":
            self.statistics.balls_hit = (self.statistics.balls_hit or 0) + 1
        if event.event_type == "ball_landed" and event.outcome in {"in", "out"}:
            if event.outcome == "in": self.statistics.balls_in = (self.statistics.balls_in or 0) + 1
            else: self.statistics.balls_out = (self.statistics.balls_out or 0) + 1
        distances = [item.world_distance_m for item in self.perception_events
                     if item.world_distance_m is not None]
        if distances:
            self.statistics.average_distance_m = sum(distances) / len(distances)
        return True

    def pause(self) -> None:
        if self.state not in {SessionState.READY, SessionState.THROWING}:
            raise ThrowingError("only a ready or throwing session can be paused")
        self.paused_from = self.state
        self.state = SessionState.PAUSED

    def resume(self) -> None:
        if self.state != SessionState.PAUSED:
            raise ThrowingError("only a paused session can be resumed")
        self.state = self.paused_from or SessionState.READY
        self.paused_from = None

    def complete(self, now: float | None = None) -> None:
        if self.state in {SessionState.IDLE, SessionState.COMPLETED}:
            raise ThrowingError(f"cannot stop from {self.state.value}")
        self.state = SessionState.STOPPING
        self.ended_at = now if now is not None else time.time()
        self.state = SessionState.COMPLETED

    def fault(self, reason: str) -> None:
        self.fault_reason = reason
        self.ended_at = time.time()
        self.state = SessionState.FAULT


def session_to_mapping(session: ThrowingSession) -> dict[str, object]:
    stats = session.statistics
    return {
        "session_id": session.session_id,
        "state": session.state.value,
        "program": session.configuration.program_id.value,
        "parameters": {
            "throw_speed_mps": session.configuration.throw_speed_mps,
            "throw_angle_deg": session.configuration.throw_angle_deg,
            "interval_s": session.configuration.interval_s,
            "ball_count": session.configuration.ball_count,
        },
        "started_at": session.started_at,
        "ended_at": session.ended_at,
        "successful_throws": len(session.successful_throw_ids),
        "throw_ids": list(session.successful_throw_ids),
        "next_target": session.next_target.value,
        "fault_reason": session.fault_reason,
        "statistics": {
            "balls_thrown": stats.balls_thrown,
            "balls_hit": stats.balls_hit,
            "hit_rate": stats.hit_rate,
            "balls_in": stats.balls_in,
            "balls_out": stats.balls_out,
            "average_distance_m": stats.average_distance_m,
        },
    }
