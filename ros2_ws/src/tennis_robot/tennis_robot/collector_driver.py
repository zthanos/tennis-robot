"""Collector hardware drivers. Only this layer knows Gazebo or serial details."""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class CollectorSensorStatus:
    entry_beam_broken: bool
    exit_beam_broken: bool
    collection_cycle_state: Optional[str] = None


class CollectorDriver(ABC):
    @abstractmethod
    def set_speed(self, speed_rad_s: float) -> None: ...

    def stop(self) -> None:
        self.set_speed(0.0)

    def sensor_status(self) -> Optional[CollectorSensorStatus]:
        return None


class GazeboCollectorDriver(CollectorDriver):
    def __init__(self, publish):
        self._publish = publish

    def set_speed(self, speed_rad_s: float) -> None:
        self._publish(float(speed_rad_s))


class SerialProtocol:
    @staticmethod
    def direction(speed: float) -> bytes:
        return b"f" if speed > 0 else b"r" if speed < 0 else b"s"

    @staticmethod
    def speed_step(up: bool) -> bytes:
        return b"+" if up else b"-"

    @staticmethod
    def parse_ir_status(line: str) -> Optional[CollectorSensorStatus]:
        if not line.startswith("ir:"):
            return None
        fields = line[3:].split(",")
        if (
            len(fields) not in (2, 3)
            or any(field not in ("0", "1") for field in fields[:2])
        ):
            return None
        return CollectorSensorStatus(
            entry_beam_broken=fields[0] == "1",
            exit_beam_broken=fields[1] == "1",
            collection_cycle_state=fields[2] if len(fields) == 3 else None,
        )


class SerialCollectorDriver(CollectorDriver):
    def __init__(self, port: str, baud: int = 9600):
        try:
            import serial
        except ImportError as exc:
            raise RuntimeError("Serial collector backend requires pyserial") from exc
        self._serial = serial.Serial(port, baud, timeout=0.1)
        # Hardware-validated 06_motor_driver_wiring_check starts at PWM=255
        # and accepts f/r/s/+/- single-byte commands with no line ending.
        self._pwm = 255
        self._sensor_status: Optional[CollectorSensorStatus] = None
        self._rx_buffer = bytearray()

    def set_speed(self, speed: float) -> None:
        if abs(speed) < 1e-6:
            self._serial.write(SerialProtocol.direction(0))
            return
        target = max(0, min(255, round(abs(speed))))
        while self._pwm < target:
            self._serial.write(SerialProtocol.speed_step(True))
            self._pwm = min(255, self._pwm + 10)
        while self._pwm > target:
            self._serial.write(SerialProtocol.speed_step(False))
            self._pwm = max(0, self._pwm - 10)
        self._serial.write(SerialProtocol.direction(speed))

    def sensor_status(self) -> Optional[CollectorSensorStatus]:
        waiting = self._serial.in_waiting
        if waiting:
            self._rx_buffer.extend(self._serial.read(waiting))
        while b"\n" in self._rx_buffer:
            raw_line, _, remainder = self._rx_buffer.partition(b"\n")
            self._rx_buffer = bytearray(remainder)
            line = raw_line.decode("ascii", errors="ignore").strip()
            parsed = SerialProtocol.parse_ir_status(line)
            if parsed is not None:
                self._sensor_status = parsed
        return self._sensor_status
