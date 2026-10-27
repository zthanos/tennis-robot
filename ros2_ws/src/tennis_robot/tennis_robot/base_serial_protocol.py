"""Hardware-independent helpers for the P2-64 Mega serial protocol."""

from __future__ import annotations

from dataclasses import dataclass


FIELDS = (
    "ms", "state", "mode", "cmd_v", "cmd_w", "ramp_v", "ramp_w",
    "duty_l", "duty_r", "lf", "lr", "rf", "rr", "w_lf", "w_lr",
    "w_rf", "w_rr", "gz_rad_s", "yaw_rad", "x_m", "y_m", "estop",
    "max_loop_gap_us", "max_telem_write_us",
)


@dataclass(frozen=True)
class DriveTelemetry:
    """One decoded 50 Hz telemetry sample from the drive-test sketch."""

    ms: int
    state: int
    mode: str
    cmd_v: float
    cmd_w: float
    ramp_v: float
    ramp_w: float
    duty_l: float
    duty_r: float
    lf: int
    lr: int
    rf: int
    rr: int
    w_lf: float
    w_lr: float
    w_rf: float
    w_rr: float
    gz_rad_s: float
    yaw_rad: float
    x_m: float
    y_m: float
    estop: int
    max_loop_gap_us: int
    max_telem_write_us: int


def parse_telemetry(line: str) -> DriveTelemetry | None:
    """Return a sample for a valid ``T,...`` line, otherwise ``None``."""

    parts = line.strip().split(",")
    if len(parts) != len(FIELDS) + 1 or parts[0] != "T":
        return None

    try:
        values = [int(parts[1]), int(parts[2]), parts[3]]
        values.extend(float(value) for value in parts[4:10])
        values.extend(int(value) for value in parts[10:14])
        values.extend(float(value) for value in parts[14:22])
        values.extend(int(value) for value in parts[22:25])
        return DriveTelemetry(*values)
    except ValueError:
        return None


class BaseSerialSession:
    """Injectable command session shared by the ROS bridge and fake tests."""

    def __init__(self, serial_port):
        self.serial = serial_port

    def send(self, line: str) -> None:
        self.serial.write((line + "\n").encode("ascii"))
        self.serial.flush()

    def configure(
        self,
        counts_per_rev: float,
        radius_m: float,
        track_m: float,
        pwm_ceiling: float,
        no_load_speed_rad_s: float,
        mode: str,
    ) -> None:
        commands = (
            f"CFG CPR {counts_per_rev}",
            f"CFG RADIUS {radius_m}",
            f"CFG TRACK {track_m}",
            f"CFG PWM {pwm_ceiling}",
            f"CFG NOLOAD {no_load_speed_rad_s}",
            f"MODE {mode.upper()}",
            "ZERO",
            "ZERO_GYRO",
            "ARM",
        )
        for command in commands:
            self.send(command)

    def stop(self) -> None:
        self.send("STOP")

    def command(self, linear_m_s: float, angular_rad_s: float) -> None:
        self.send(f"CMD {linear_m_s:.6f} {angular_rad_s:.6f}")

    def read(self) -> DriveTelemetry | None:
        line = self.serial.readline().decode("ascii", "replace")
        return parse_telemetry(line)
