"""Collector behavior primitives for the Concept A funnel + intake roller MVP."""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum

from config_utils import _env_float


class CollectorState(str, Enum):
    IDLE = "idle"
    SURVEY = "survey"
    SCAN = "scan"
    ALIGN = "align"
    APPROACH = "approach"
    CAPTURE = "capture"
    REVERSE_CLEAR = "reverse_clear"
    COLLECTED = "collected"


@dataclass(frozen=True)
class BallObservationInput:
    visible: bool
    bearing_rad: float = 0.0
    distance_m: float = math.inf
    confidence: float = 0.0
    source: str = "unknown"
    robot_x_m: float | None = None
    robot_y_m: float | None = None
    world_x_m: float | None = None
    world_y_m: float | None = None


@dataclass(frozen=True)
class BaseCommand:
    linear_speed_m_s: float
    angular_speed_rad_s: float


@dataclass(frozen=True)
class CollectorCommand:
    lift_wheel_speed: float
    intake_enabled: bool


@dataclass(frozen=True)
class ConceptACommand:
    state: CollectorState
    base: BaseCommand
    collector: CollectorCommand


@dataclass(frozen=True)
class ConceptAConfig:
    align_tolerance_rad: float = math.radians(4.0)
    capture_distance_m: float = 0.34
    collected_hold_s: float = 0.7
    lost_target_timeout_s: float = 0.5
    capture_timeout_s: float = 2.8
    scan_angular_speed_rad_s: float = 0.75
    scan_full_turn_s: float = 8.4
    align_angular_gain: float = 2.7
    max_align_angular_speed_rad_s: float = 1.2
    approach_speed_m_s: float = 0.22
    capture_speed_m_s: float = 0.08
    capture_angular_gain: float = 1.2
    reverse_speed_m_s: float = -0.10
    lift_wheel_speed: float = 1.0

    @classmethod
    def from_env(cls) -> "ConceptAConfig":
        defaults = cls()
        return cls(
            align_tolerance_rad=math.radians(_env_float("COLLECTOR_ALIGN_TOLERANCE_DEG", math.degrees(defaults.align_tolerance_rad))),
            capture_distance_m=_env_float("COLLECTOR_CAPTURE_DISTANCE_M", defaults.capture_distance_m),
            collected_hold_s=_env_float("COLLECTOR_COLLECTED_HOLD_S", defaults.collected_hold_s),
            lost_target_timeout_s=_env_float("COLLECTOR_LOST_TARGET_TIMEOUT_S", defaults.lost_target_timeout_s),
            capture_timeout_s=_env_float("COLLECTOR_CAPTURE_TIMEOUT_S", defaults.capture_timeout_s),
            scan_angular_speed_rad_s=_env_float("COLLECTOR_SCAN_ANGULAR_SPEED_RAD_S", defaults.scan_angular_speed_rad_s),
            scan_full_turn_s=_env_float("COLLECTOR_SCAN_FULL_TURN_S", defaults.scan_full_turn_s),
            align_angular_gain=_env_float("COLLECTOR_ALIGN_ANGULAR_GAIN", defaults.align_angular_gain),
            max_align_angular_speed_rad_s=_env_float(
                "COLLECTOR_MAX_ALIGN_ANGULAR_SPEED_RAD_S",
                defaults.max_align_angular_speed_rad_s,
            ),
            approach_speed_m_s=_env_float("COLLECTOR_APPROACH_SPEED_M_S", defaults.approach_speed_m_s),
            capture_speed_m_s=_env_float("COLLECTOR_CAPTURE_SPEED_M_S", defaults.capture_speed_m_s),
            capture_angular_gain=_env_float("COLLECTOR_CAPTURE_ANGULAR_GAIN", defaults.capture_angular_gain),
            reverse_speed_m_s=-abs(_env_float("COLLECTOR_REVERSE_SPEED_M_S", abs(defaults.reverse_speed_m_s))),
            lift_wheel_speed=_env_float("COLLECTOR_LIFT_WHEEL_SPEED", defaults.lift_wheel_speed),
        )


class ConceptACollectorBehavior:
    """Small state machine that turns ball observations into base/collector commands."""

    def __init__(self, config: ConceptAConfig | None = None) -> None:
        self.config = config or ConceptAConfig()
        self.state = CollectorState.SCAN
        self._state_elapsed_s = 0.0
        self._lost_elapsed_s = 0.0
        self._last_visible_observation: BallObservationInput | None = None
        self._scan_best_observation: BallObservationInput | None = None

    def reset(self) -> None:
        self.state = CollectorState.SCAN
        self._state_elapsed_s = 0.0
        self._lost_elapsed_s = 0.0
        self._last_visible_observation = None
        self._scan_best_observation = None

    @property
    def state_elapsed_s(self) -> float:
        return self._state_elapsed_s

    @property
    def scan_best_observation(self) -> BallObservationInput | None:
        return self._scan_best_observation

    def start_tracking(self, observation: BallObservationInput) -> None:
        if not observation.visible:
            return
        self._lost_elapsed_s = 0.0
        self._last_visible_observation = observation
        self._scan_best_observation = observation
        self._transition(self._tracking_state(observation))

    def update(
        self,
        observation: BallObservationInput,
        dt_s: float,
        collection_confirmed: bool = False,
        jam_detected: bool = False,
    ) -> ConceptACommand:
        self._state_elapsed_s += max(0.0, dt_s)
        if observation.visible:
            self._lost_elapsed_s = 0.0
            self._last_visible_observation = observation
            if self.state == CollectorState.SCAN:
                self._remember_scan_observation(observation)
        else:
            self._lost_elapsed_s += max(0.0, dt_s)
        tracking_observation = self._tracking_observation(observation)

        if collection_confirmed:
            self._transition(CollectorState.COLLECTED)
        elif jam_detected:
            self._transition(CollectorState.REVERSE_CLEAR)
        elif self.state == CollectorState.SCAN:
            if self._state_elapsed_s >= self.config.scan_full_turn_s and self._scan_best_observation is not None:
                tracking_observation = self._scan_best_observation
                self._transition(self._tracking_state(tracking_observation))
        elif self.state in {CollectorState.ALIGN, CollectorState.APPROACH}:
            if self._lost_elapsed_s > self.config.lost_target_timeout_s:
                self._transition(CollectorState.SCAN)
            else:
                self._transition(self._tracking_state(tracking_observation))
        elif self.state == CollectorState.CAPTURE:
            if self._state_elapsed_s > self.config.capture_timeout_s:
                self._transition(CollectorState.REVERSE_CLEAR)
        elif self.state == CollectorState.REVERSE_CLEAR:
            if self._state_elapsed_s > 0.6:
                self._transition(CollectorState.SCAN)
        elif self.state == CollectorState.COLLECTED:
            if self._state_elapsed_s > self.config.collected_hold_s:
                self._transition(CollectorState.SCAN)

        return self._command_for_state(tracking_observation)

    def _remember_scan_observation(self, observation: BallObservationInput) -> None:
        current = self._scan_best_observation
        if current is None:
            self._scan_best_observation = observation
            return
        current_score = current.confidence / max(0.25, current.distance_m)
        new_score = observation.confidence / max(0.25, observation.distance_m)
        if new_score > current_score:
            self._scan_best_observation = observation

    def _tracking_observation(self, observation: BallObservationInput) -> BallObservationInput:
        if observation.visible:
            return observation
        if self.state in {CollectorState.ALIGN, CollectorState.APPROACH} and self._last_visible_observation is not None:
            if self._lost_elapsed_s <= self.config.lost_target_timeout_s:
                return self._last_visible_observation
        return observation

    def _tracking_state(self, observation: BallObservationInput) -> CollectorState:
        if not observation.visible:
            return CollectorState.SCAN
        if abs(observation.bearing_rad) > self.config.align_tolerance_rad:
            return CollectorState.ALIGN
        if observation.distance_m > self.config.capture_distance_m:
            return CollectorState.APPROACH
        return CollectorState.CAPTURE

    def _transition(self, next_state: CollectorState) -> None:
        if next_state != self.state:
            previous_state = self.state
            self.state = next_state
            self._state_elapsed_s = 0.0
            if next_state == CollectorState.SCAN:
                self._scan_best_observation = None
            elif previous_state == CollectorState.SCAN:
                self._lost_elapsed_s = 0.0

    def _clamped_turn(self, bearing_rad: float) -> float:
        cfg = self.config
        return max(
            -cfg.max_align_angular_speed_rad_s,
            min(cfg.max_align_angular_speed_rad_s, bearing_rad * cfg.align_angular_gain),
        )

    def _command_for_state(self, observation: BallObservationInput) -> ConceptACommand:
        cfg = self.config
        if self.state == CollectorState.SCAN:
            base = BaseCommand(0.0, cfg.scan_angular_speed_rad_s)
            collector = CollectorCommand(0.0, False)
        elif self.state == CollectorState.ALIGN:
            base = BaseCommand(0.0, self._clamped_turn(observation.bearing_rad))
            collector = CollectorCommand(0.0, False)
        elif self.state == CollectorState.APPROACH:
            base = BaseCommand(cfg.approach_speed_m_s, self._clamped_turn(observation.bearing_rad))
            collector = CollectorCommand(cfg.lift_wheel_speed, True)
        elif self.state == CollectorState.CAPTURE:
            base = BaseCommand(cfg.capture_speed_m_s, observation.bearing_rad * cfg.capture_angular_gain)
            collector = CollectorCommand(cfg.lift_wheel_speed, True)
        elif self.state == CollectorState.REVERSE_CLEAR:
            base = BaseCommand(cfg.reverse_speed_m_s, 0.0)
            collector = CollectorCommand(-cfg.lift_wheel_speed, True)
        else:
            base = BaseCommand(0.0, 0.0)
            collector = CollectorCommand(0.0, False)

        return ConceptACommand(state=self.state, base=base, collector=collector)
