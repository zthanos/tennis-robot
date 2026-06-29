"""Collector application interface shared by automatic and manual control."""

from __future__ import annotations

from dataclasses import dataclass

from tennis_robot.collector_driver import CollectorDriver


@dataclass(frozen=True)
class CollectorStatus:
    running: bool
    speed_rad_s: float
    manual_override: bool


class CollectorInterface:
    def __init__(self, driver: CollectorDriver, *, default_speed: float = 10.0, max_speed: float = 40.0):
        self._driver = driver
        self.default_speed = default_speed
        self.max_speed = max_speed
        self._speed = default_speed
        self._running = False
        self.manual_override = False

    def start(self) -> None:
        self.manual_override = True
        self._running = True
        self._driver.set_speed(self._speed)

    def stop(self) -> None:
        self.manual_override = True
        self._running = False
        self._driver.stop()

    def adjust_speed(self, delta: float) -> None:
        self.manual_override = True
        self._speed = max(-self.max_speed, min(self.max_speed, self._speed + delta))
        if self._running:
            self._driver.set_speed(self._speed)

    def release_manual(self) -> None:
        self.manual_override = False

    def apply_automatic(self, speed: float) -> None:
        if self.manual_override:
            return
        self._running = abs(speed) > 1e-6
        self._speed = max(-self.max_speed, min(self.max_speed, speed))
        self._driver.set_speed(self._speed)

    @property
    def status(self) -> CollectorStatus:
        return CollectorStatus(self._running, self._speed, self.manual_override)
