"""collect_one mission: pick up one ball then return to start pose.

Pure FSM — no Webots dependency. The controller passes a live
ConceptACollectorBehavior instance on each update() call so the mission
shares the same behavior object without owning it.
"""

from __future__ import annotations

import math
import time as _time
from typing import TYPE_CHECKING

from tennis_robot.collector import (
    BaseCommand,
    BallObservationInput,
    CollectorCommand,
    CollectorState,
    ConceptACommand,
    ConceptACollectorBehavior,
)

if TYPE_CHECKING:
    pass


# ── Constants (no env-var overrides needed) ────────────────────────────────────

_RETURN_POSITION_TOLERANCE_M = 0.12
_BASKET_SETTLE_HOLD_S = 2.0
_BASKET_SETTLE_INTAKE_S = 0.25
# Max distance between a fresh sighting and the current lock for the lock to
# be refreshed (same physical ball) instead of ignored (different ball).
_RELOCK_GATE_M = 0.6
_RETURN_YAW_TOLERANCE_RAD = math.radians(6.0)
_RETURN_LINEAR_GAIN = 0.55
_RETURN_ANGULAR_GAIN = 1.8
_RETURN_MAX_SPEED_M_S = 0.28
_RETURN_MAX_TURN_RAD_S = 1.0
_SCAN_STEP_RAD = math.radians(30.0)
_SCAN_STEP_TOLERANCE_RAD = math.radians(2.0)
_SCAN_TURN_SPEED_RAD_S = 0.65
_SCAN_SETTLE_S = 0.20

_IDLE_CMD = ConceptACommand(
    state=CollectorState.IDLE,
    base=BaseCommand(0.0, 0.0),
    collector=CollectorCommand(0.0, False),
)
_SCAN_CMD = ConceptACommand(
    state=CollectorState.SCAN,
    base=BaseCommand(0.0, 0.0),
    collector=CollectorCommand(0.0, False),
)


def _angle_delta(a: float, b: float) -> float:
    return (a - b + math.pi) % (2 * math.pi) - math.pi


def _world_to_robot_obs(
    world_x: float,
    world_y: float,
    robot_x: float,
    robot_y: float,
    robot_yaw: float,
) -> BallObservationInput | None:
    dx = world_x - robot_x
    dy = world_y - robot_y
    local_x = math.cos(-robot_yaw) * dx - math.sin(-robot_yaw) * dy
    local_y = math.sin(-robot_yaw) * dx + math.cos(-robot_yaw) * dy
    if local_x <= -0.1:
        return None
    return BallObservationInput(
        visible=True,
        bearing_rad=math.atan2(local_y, local_x),
        distance_m=math.hypot(local_x, local_y),
        confidence=0.8,
        source="collect_one_locked",
        robot_x_m=local_x,
        robot_y_m=local_y,
        world_x_m=world_x,
        world_y_m=world_y,
    )


# ── Mission ────────────────────────────────────────────────────────────────────

class CollectOneMission:
    """Drives: scan in place → align/approach/capture → return to start.

    The controller owns the ConceptACollectorBehavior and passes it to update()
    so the behavior is reset correctly when modes switch.
    """

    def __init__(self) -> None:
        self.phase: str = "idle"          # idle | collect | settle | return | done
        self._start_pose: tuple[float, float, float] | None = None
        self._locked_world: tuple[float, float] | None = None
        self._scan_target_yaw: float | None = None
        self._scan_settle_until_s: float = 0.0
        self._scan_steps_taken: int = 0
        self._settle_remaining_s: float = 0.0
        self._complete_reported: bool = False

    def start(self, robot_pose: tuple[float, float, float]) -> None:
        """Initialize mission at the given start pose (x, y, yaw)."""
        self.phase = "collect"
        self._start_pose = robot_pose
        self._locked_world = None
        self._scan_target_yaw = None
        self._scan_settle_until_s = 0.0
        self._scan_steps_taken = 0
        self._settle_remaining_s = 0.0
        self._complete_reported = False

    def reset(self) -> None:
        self.__init__()

    @property
    def is_done(self) -> bool:
        return self.phase == "done"

    def update(
        self,
        observation: BallObservationInput,
        collection_confirmed: bool,
        dt_s: float,
        robot_pose: tuple[float, float, float],
        behavior: ConceptACollectorBehavior,
    ) -> ConceptACommand:
        """Compute next command. Called every timestep while mode == 'collect_one'."""
        if self._start_pose is None:
            self.start(robot_pose)

        if self.phase == "collect":
            return self._collect_phase(observation, collection_confirmed, dt_s, robot_pose, behavior)

        if self.phase == "settle":
            self._settle_remaining_s = max(0.0, self._settle_remaining_s - dt_s)
            if self._settle_remaining_s > 0.0:
                intake_enabled = self._settle_remaining_s > (
                    _BASKET_SETTLE_HOLD_S - _BASKET_SETTLE_INTAKE_S
                )
                return ConceptACommand(
                    state=CollectorState.COLLECTED,
                    base=BaseCommand(0.0, 0.0),
                    collector=CollectorCommand(
                        behavior.config.lift_wheel_speed if intake_enabled else 0.0,
                        intake_enabled,
                    ),
                )
            self.phase = "return"

        if self.phase == "return":
            cmd = self._return_phase(robot_pose)
            if cmd is not None:
                return cmd
            self.phase = "done"

        return _IDLE_CMD

    # ── Phases ─────────────────────────────────────────────────────────────────

    def _collect_phase(
        self,
        observation: BallObservationInput,
        collection_confirmed: bool,
        dt_s: float,
        robot_pose: tuple[float, float, float],
        behavior: ConceptACollectorBehavior,
    ) -> ConceptACommand:
        if collection_confirmed:
            behavior.reset()
            self.phase = "settle"
            self._settle_remaining_s = _BASKET_SETTLE_HOLD_S
            return ConceptACommand(
                state=CollectorState.COLLECTED,
                base=BaseCommand(0.0, 0.0),
                collector=CollectorCommand(behavior.config.lift_wheel_speed, True),
            )

        robot_x, robot_y, robot_yaw = robot_pose

        if behavior.state == CollectorState.SCAN and observation.visible:
            self._scan_target_yaw = None
            self._scan_settle_until_s = 0.0
            behavior.start_tracking(observation)
            if observation.world_x_m is not None and observation.world_y_m is not None:
                self._locked_world = (observation.world_x_m, observation.world_y_m)
            return behavior.update(observation, dt_s, collection_confirmed=False)

        if behavior.state == CollectorState.SCAN and not observation.visible:
            self._locked_world = None
            return self._scan_step(robot_yaw)

        # Keep the lock FRESH while the ball is still visible. The first lock
        # is taken from the scan sighting (possibly 3-5 m out, right after a
        # rotation) where perception + odom-yaw error is largest; live ground
        # truth showed blind legs missing by 0.7 m on that stale lock
        # (debug-log #44). Re-locking on each nearer sighting shrinks the
        # frozen error to the last-visible accuracy (~7 cm at ~1 m) before the
        # camera's near-field blind zone takes over. The gate keeps a stray
        # detection of a DIFFERENT ball from stealing the lock mid-approach.
        if (
            observation.visible
            and observation.world_x_m is not None
            and observation.world_y_m is not None
            and self._locked_world is not None
            and math.hypot(
                observation.world_x_m - self._locked_world[0],
                observation.world_y_m - self._locked_world[1],
            )
            <= _RELOCK_GATE_M
        ):
            self._locked_world = (observation.world_x_m, observation.world_y_m)

        locked_obs = (
            _world_to_robot_obs(*self._locked_world, robot_x, robot_y, robot_yaw)
            if self._locked_world is not None
            else None
        )
        tracking_obs = locked_obs if locked_obs is not None else observation
        return behavior.update(tracking_obs, dt_s, collection_confirmed=False)

    def _return_phase(self, robot_pose: tuple[float, float, float]) -> ConceptACommand | None:
        if self._start_pose is None:
            return None
        start_x, start_y, start_yaw = self._start_pose
        robot_x, robot_y, robot_yaw = robot_pose
        dx = start_x - robot_x
        dy = start_y - robot_y
        dist = math.hypot(dx, dy)
        if dist > _RETURN_POSITION_TOLERANCE_M:
            heading_err = _angle_delta(math.atan2(dy, dx), robot_yaw)
            linear = min(_RETURN_MAX_SPEED_M_S, dist * _RETURN_LINEAR_GAIN)
            if abs(heading_err) > math.radians(35.0):
                linear = 0.0
            angular = max(-_RETURN_MAX_TURN_RAD_S, min(_RETURN_MAX_TURN_RAD_S, heading_err * _RETURN_ANGULAR_GAIN))
            return ConceptACommand(
                state=CollectorState.SURVEY,
                base=BaseCommand(linear, angular),
                collector=CollectorCommand(0.0, False),
            )
        yaw_err = _angle_delta(start_yaw, robot_yaw)
        if abs(yaw_err) > _RETURN_YAW_TOLERANCE_RAD:
            angular = max(-_RETURN_MAX_TURN_RAD_S, min(_RETURN_MAX_TURN_RAD_S, yaw_err * _RETURN_ANGULAR_GAIN))
            return ConceptACommand(
                state=CollectorState.SURVEY,
                base=BaseCommand(0.0, angular),
                collector=CollectorCommand(0.0, False),
            )
        return None

    def _scan_step(self, robot_yaw: float) -> ConceptACommand:
        now = _time.time()
        if self._scan_settle_until_s > now:
            return _SCAN_CMD

        max_steps = int(math.ceil(2 * math.pi / _SCAN_STEP_RAD))
        if self._scan_target_yaw is None:
            if self._scan_steps_taken >= max_steps:
                self.phase = "done"
                return _SCAN_CMD
            self._scan_target_yaw = robot_yaw + _SCAN_STEP_RAD

        yaw_err = _angle_delta(self._scan_target_yaw, robot_yaw)
        if abs(yaw_err) <= _SCAN_STEP_TOLERANCE_RAD:
            self._scan_target_yaw = None
            self._scan_steps_taken += 1
            self._scan_settle_until_s = now + _SCAN_SETTLE_S
            return _SCAN_CMD

        angular = max(-_SCAN_TURN_SPEED_RAD_S, min(_SCAN_TURN_SPEED_RAD_S, yaw_err * _RETURN_ANGULAR_GAIN))
        return ConceptACommand(
            state=CollectorState.SCAN,
            base=BaseCommand(0.0, angular),
            collector=CollectorCommand(0.0, False),
        )
