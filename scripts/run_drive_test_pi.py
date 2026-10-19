#!/usr/bin/env python3
"""Run one frozen P2-62 scenario on the Pi, or exercise it with --dry-run."""

from __future__ import annotations

import argparse
import csv
import fcntl
import json
import math
import select
import signal
import sys
import termios
import time
import tty
from datetime import datetime, timezone
from pathlib import Path


HEADER = (
    "ms", "state", "mode", "cmd_v", "cmd_w", "ramp_v", "ramp_w",
    "duty_l", "duty_r", "lf", "lr", "rf", "rr", "w_lf", "w_lr",
    "w_rf", "w_rr", "gz_rad_s", "yaw_rad", "x_m", "y_m", "estop",
    "max_loop_gap_us", "max_telem_write_us",
)


def scenario(name: str):
    """Return (linear speed, yaw rate, duration) phases for a frozen test."""

    scenarios = {
        "straight_0p3": [(0.3, 0.0, 10.0)],
        "straight_0p6": [(0.6, 0.0, 5.0)],
        "spin_90": [(0.0, 0.5, math.pi)],
        "spin_180": [(0.0, 0.5, 2.0 * math.pi)],
        "spin_360": [(0.0, 0.5, 4.0 * math.pi)],
        "arc_R0p5": [(0.3, 0.6, math.pi * 0.5 / 0.3)],
        "arc_R1p0": [(0.3, 0.3, math.pi / 0.3)],
        "accel_brake": [(0.6, 0.0, 3.0), (0.0, 0.0, 2.0)],
    }
    return scenarios[name]


class Port:
    """Exclusive Mega port; runner and ROS bridge may never own it together."""

    def __init__(self, path: str, baud: int, dry_run: bool):
        self.dry_run = dry_run
        self.serial = None
        self.lock = open("/tmp/tennis_robot_ttyACM0.lock", "w", encoding="ascii")
        fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)

        if not dry_run:
            import serial

            self.serial = serial.Serial(
                path,
                baud,
                timeout=0.005,
                exclusive=True,
            )
            time.sleep(2.0)

    def send(self, command: str) -> None:
        if self.serial is not None:
            self.serial.write((command + "\n").encode("ascii"))
            self.serial.flush()

    def line(self) -> str:
        if self.serial is None:
            return ""
        return self.serial.readline().decode("ascii", "replace").strip()

    def close(self) -> None:
        if self.serial is not None:
            self.serial.close()
        fcntl.flock(self.lock, fcntl.LOCK_UN)
        self.lock.close()


def parse_arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "scenario",
        choices=(
            "straight_0p3", "straight_0p6", "spin_90", "spin_180",
            "spin_360", "arc_R0p5", "arc_R1p0", "accel_brake",
        ),
    )
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=500000)
    parser.add_argument("--mode", choices=("open", "gyro"), default="gyro")
    parser.add_argument("--counts-per-rev", type=float, required=True)
    parser.add_argument("--radius-mm", type=float, default=85.0)
    parser.add_argument("--track-m", type=float, default=0.585)
    parser.add_argument("--pwm-ceiling", type=float, default=1.0)
    parser.add_argument(
        "--no-load-rad-s",
        type=float,
        default=122.0 * 2.0 * math.pi / 60.0,
    )
    parser.add_argument("--accel", type=float, default=0.35)
    parser.add_argument("--yaw-accel", type=float, default=0.8)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path.home() / "tennis-robot-logs",
    )
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def main():
    args = parse_arguments()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    stem = f"{timestamp}_{args.scenario}_{args.mode}_r{args.radius_mm:g}"
    csv_path = args.output_dir / f"{stem}.csv"
    metadata_path = args.output_dir / f"{stem}.json"

    phases = scenario(args.scenario)
    maximum_time_s = sum(phase[2] for phase in phases) + 2.0
    if "straight" in args.scenario or "arc" in args.scenario:
        maximum_distance_m = 4.0
    else:
        maximum_distance_m = 1.0

    metadata = vars(args).copy()
    metadata.update({
        "output_dir": str(args.output_dir),
        "created_utc": timestamp,
        "phases": phases,
        "max_time_s": maximum_time_s,
        "max_distance_m": maximum_distance_m,
        "hardware": not args.dry_run,
        "serial_budget": {
            "baud": args.baud,
            "assumed_max_telemetry_chars": 220,
            "wire_time_ms": 220.0 * 10.0 / args.baud * 1000.0,
            "stop_requirement_ms": 20.0,
        },
    })

    port = Port(args.port, args.baud, args.dry_run)
    stopping = False
    terminal_state = None
    maximum_host_loop_gap_s = 0.0

    def emergency_stop(*_):
        nonlocal stopping
        stopping = True
        port.send("STOP")

    for handled_signal in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(handled_signal, emergency_stop)

    try:
        if sys.stdin.isatty():
            terminal_state = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
            print("SPACE/q/Esc = immediate STOP", file=sys.stderr)

        commands = (
            f"CFG CPR {args.counts_per_rev}",
            f"CFG RADIUS {args.radius_mm / 1000.0}",
            f"CFG TRACK {args.track_m}",
            f"CFG PWM {args.pwm_ceiling}",
            f"CFG NOLOAD {args.no_load_rad_s}",
            f"CFG ACCEL {args.accel} {args.yaw_accel}",
            f"LIMIT {maximum_time_s} {maximum_distance_m}",
            f"MODE {args.mode.upper()}",
            "ZERO",
            "ZERO_GYRO",
            "ARM",
        )
        for command in commands:
            port.send(command)
            time.sleep(0.03)

        with csv_path.open("w", newline="", encoding="utf-8") as csv_file:
            output = csv.writer(csv_file)
            output.writerow(HEADER)
            start_time = time.monotonic()
            previous_iteration = start_time
            last_ping_time = 0.0
            last_synthetic_time = 0.0
            synthetic_counts = 0

            for linear, angular, duration in phases:
                phase_start = time.monotonic()
                port.send(f"CMD {linear} {angular}")

                while time.monotonic() - phase_start < duration and not stopping:
                    now = time.monotonic()
                    maximum_host_loop_gap_s = max(
                        maximum_host_loop_gap_s,
                        now - previous_iteration,
                    )
                    previous_iteration = now

                    if now - last_ping_time > 0.1:
                        port.send("PING")
                        last_ping_time = now

                    if sys.stdin.isatty() and select.select([sys.stdin], [], [], 0)[0]:
                        if sys.stdin.read(1) in (" ", "q", "Q", "\x1b"):
                            emergency_stop()

                    line = port.line()
                    if line.startswith("T,"):
                        fields = line.split(",")[1:]
                        if len(fields) == len(HEADER):
                            output.writerow(fields)
                            csv_file.flush()
                    elif args.dry_run and now - last_synthetic_time >= 0.02:
                        last_synthetic_time = now
                        elapsed = now - start_time
                        synthetic_counts += int(
                            linear * args.counts_per_rev * 0.02
                            / (2.0 * math.pi * (args.radius_mm / 1000.0))
                        )
                        wheel_speed = linear / (args.radius_mm / 1000.0)
                        output.writerow([
                            int(elapsed * 1000.0), 1, args.mode.upper(),
                            linear, angular, linear, angular, 0.1, 0.1,
                            synthetic_counts, synthetic_counts,
                            synthetic_counts, synthetic_counts,
                            wheel_speed, wheel_speed, wheel_speed, wheel_speed,
                            angular, angular * elapsed, linear * elapsed, 0.0,
                            0, 10000, 4400,
                        ])
                        csv_file.flush()
                    if args.dry_run:
                        time.sleep(0.005)

            port.send("STOP")
            time.sleep(0.3)
    finally:
        if terminal_state is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, terminal_state)
        port.send("STOP")
        port.send("DISARM")
        port.close()

    metadata["dry_run_host_max_loop_gap_ms"] = maximum_host_loop_gap_s * 1000.0
    metadata["stop_latency_verification"] = (
        "Analytical serial bound only; verify max_loop_gap_us and STOP on Mega "
        "during lifted-wheel bring-up."
    )
    metadata_path.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")

    print(csv_path)
    print(metadata_path)
    return 130 if stopping else 0


if __name__ == "__main__":
    raise SystemExit(main())
