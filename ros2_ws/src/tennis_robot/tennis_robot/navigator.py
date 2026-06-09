"""Robot navigation primitives.

Stateless movement API: direction, speed, turning, stop.
Consumed by survey, collector, and any other behavior — nothing here
knows about FSM states or sensor semantics.
"""

from __future__ import annotations

import math

from tennis_robot.collector import BaseCommand


class Navigator:
    """Thin command factory with clamping and safety override."""

    def __init__(
        self,
        max_linear_m_s: float = 1.0,
        max_angular_rad_s: float = 2.0,
        stop_range_m: float = 0.32,
        slow_range_m: float = 0.65,
        slow_factor: float = 0.35,
    ) -> None:
        self.max_linear_m_s = max_linear_m_s
        self.max_angular_rad_s = max_angular_rad_s
        self.stop_range_m = stop_range_m
        self.slow_range_m = slow_range_m
        self.slow_factor = slow_factor

    # ------------------------------------------------------------------
    # Core primitives
    # ------------------------------------------------------------------

    def stop(self) -> BaseCommand:
        return BaseCommand(0.0, 0.0)

    def forward(self, speed_m_s: float) -> BaseCommand:
        return self.drive(speed_m_s, 0.0)

    def turn(self, angular_rad_s: float) -> BaseCommand:
        return self.drive(0.0, angular_rad_s)

    def drive(self, linear_m_s: float, angular_rad_s: float = 0.0) -> BaseCommand:
        return BaseCommand(
            _clamp(linear_m_s, -self.max_linear_m_s, self.max_linear_m_s),
            _clamp(angular_rad_s, -self.max_angular_rad_s, self.max_angular_rad_s),
        )

    # ------------------------------------------------------------------
    # Safety override (call last before issuing any command)
    # ------------------------------------------------------------------

    def apply_safety(self, command: BaseCommand, front_range_m: float) -> BaseCommand:
        if command.linear_speed_m_s <= 0.0:
            return command
        if front_range_m <= self.stop_range_m:
            return self.stop()
        if front_range_m <= self.slow_range_m:
            return BaseCommand(
                command.linear_speed_m_s * self.slow_factor,
                command.angular_speed_rad_s,
            )
        return command

    # ------------------------------------------------------------------
    # Derived helpers
    # ------------------------------------------------------------------

    def clamp_turn(self, command: BaseCommand, max_rad_s: float) -> BaseCommand:
        """Limit angular component without touching linear."""
        clamped = _clamp(command.angular_speed_rad_s, -max_rad_s, max_rad_s)
        return BaseCommand(command.linear_speed_m_s, clamped)

    def scale_linear(self, command: BaseCommand, factor: float) -> BaseCommand:
        """Multiply linear component by factor, leave angular unchanged."""
        return BaseCommand(command.linear_speed_m_s * factor, command.angular_speed_rad_s)


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def navigator_from_survey_config(config: object) -> Navigator:
    """Build a Navigator wired to a SurveyConfig instance."""
    return Navigator(
        max_linear_m_s=config.drive_speed_m_s,
        max_angular_rad_s=config.turn_speed_rad_s,
        stop_range_m=config.safety_stop_range_m,
        slow_range_m=config.safety_slow_range_m,
        slow_factor=0.35,
    )
