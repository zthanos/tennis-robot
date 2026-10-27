#!/usr/bin/env python3
"""Run automatic dual-intake ball cycles through the Arduino Mega.

The Mega owns the time-critical safety path: entry IR starts both motors and
exit IR (or the configured timeout) stops them. This Pi-side script arms and
re-arms that cycle over USB serial and always sends STOP + DISARM on exit.

The Mega must run arduino/collector/07_dual_intake_mega_bench.
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from dataclasses import dataclass
from typing import Protocol


MAX_PWM = 90
MAX_TIMEOUT_S = 4.0
EXPECTED_FEATURE = "FEATURE,AUTO_TIMEOUT_MS,4000,STOP_ON_EXIT_BROKEN"
EXPECTED_BENCH_FEATURE = "FEATURE,AUTO_BENCH_NO_ENCODER,HARD_TIMEOUT_MS,4000"


class SerialPort(Protocol):
    def write(self, data: bytes) -> int: ...
    def readline(self) -> bytes: ...
    def close(self) -> None: ...


@dataclass(frozen=True)
class IntakeTelemetry:
    mega_ms: int
    state: int
    left_pwm: int
    right_pwm: int
    left_rpm: float
    right_rpm: float
    entry_broken: bool
    exit_broken: bool


def parse_data_line(line: str) -> IntakeTelemetry | None:
    """Parse the Mega DATA record; return None for non-telemetry lines."""
    fields = line.split(",")
    if len(fields) != 11 or fields[0] != "DATA":
        return None
    try:
        return IntakeTelemetry(
            mega_ms=int(fields[1]),
            state=int(fields[2]),
            left_pwm=int(fields[3]),
            right_pwm=int(fields[4]),
            left_rpm=float(fields[7]),
            right_rpm=float(fields[8]),
            entry_broken=bool(int(fields[9])),
            exit_broken=bool(int(fields[10])),
        )
    except ValueError:
        return None


def send_line(port: SerialPort, line: str) -> None:
    port.write((line + "\n").encode("ascii"))


def open_serial(port_name: str, baud: int) -> SerialPort:
    try:
        import serial  # type: ignore[import-not-found]
    except ImportError as exc:
        raise RuntimeError(
            "pyserial is missing; install it on the Pi with "
            "'sudo apt install python3-serial'"
        ) from exc
    return serial.Serial(port_name, baud, timeout=0.1)


def run(args: argparse.Namespace) -> int:
    timeout_ms = round(args.timeout * 1000)
    port: SerialPort | None = None
    completed_cycles = 0

    try:
        print(f"Connecting to Mega on {args.port} @ {args.baud} ...")
        port = open_serial(args.port, args.baud)
        time.sleep(args.startup_delay)  # Mega resets when USB serial opens.

        # STOP is intentionally first. AUTO is not sent until the updated
        # firmware feature banner has been observed and both beams are clear.
        send_line(port, "STOP")
        send_line(port, "HELP")
        send_line(port, "ARM")

        feature_seen = False
        bench_feature_seen = False
        armed = False
        cycle_armed = False
        feature_deadline = time.monotonic() + 3.0
        last_rx = time.monotonic()
        last_feedback = 0.0
        previous_beams: tuple[bool, bool] | None = None

        print(
            f"Waiting: entry IR starts both motors at PWM {args.pwm}; "
            f"exit IR or {args.timeout:.3g} s stops them. Ctrl-C stops/disarms."
        )

        while True:
            raw = port.readline()
            now = time.monotonic()

            if raw:
                last_rx = now
                line = raw.decode("ascii", "replace").strip()
                if not line:
                    continue

                telemetry = parse_data_line(line)
                if args.raw and telemetry is not None:
                    print(f"[raw] {line}")

                if line == EXPECTED_FEATURE:
                    feature_seen = True
                    print("[ready] Mega firmware: exit-stop + 4 s timeout verified")
                elif line == EXPECTED_BENCH_FEATURE:
                    bench_feature_seen = True
                    print("[ready] Bench mode available: encoder check disabled")
                elif line.startswith("OK,ARMED"):
                    armed = True
                    print("[safety] Mega ARMED")
                elif line.startswith("OK,DISARMED"):
                    armed = False
                    print("[safety] Mega DISARMED")
                elif line.startswith("OK,WAITING_FOR_BALL"):
                    cycle_armed = True
                    print("[waiting] Both motors OFF; waiting for front IR")
                elif line.startswith("ERR,BEAMS_NOT_CLEAR"):
                    # Wait for a DATA record showing both beams clear, then
                    # re-arm. This also prevents retriggering on the same ball.
                    cycle_armed = False
                    print("[waiting] IR beam still blocked; waiting until both clear")
                elif line.startswith("FAULT,"):
                    raise RuntimeError(f"Mega safety fault: {line}")
                elif line.startswith("ERR,"):
                    raise RuntimeError(f"Mega rejected command: {line}")
                elif line.startswith("BALL_ENTRY,"):
                    print(f"[event] Front IR BROKEN -> motors starting at PWM {args.pwm}")
                elif line.endswith(",RUN_STARTED"):
                    print("[motors] ON")
                elif line.startswith("BALL_EXIT_REACHED,"):
                    fields = line.split(",")
                    transit_ms = fields[2] if len(fields) > 2 else "?"
                    print(f"[event] Rear IR BROKEN after {transit_ms} ms")
                elif ",RUN_STOPPED," in line:
                    completed_cycles += 1
                    cycle_armed = False
                    reason = line.rsplit(",", 1)[-1]
                    if reason == "BALL_EXIT":
                        print("[motors] OFF: ball reached rear IR")
                    elif reason == "BALL_TIMEOUT":
                        print(f"[motors] OFF: safety timeout at {args.timeout:.3g} s")
                    else:
                        print(f"[motors] OFF: {reason}")
                    if args.once:
                        print(f"Completed {completed_cycles} ball cycle.")
                        return 0
                elif telemetry is None and not line.startswith(
                    ("DATA,", "HEADER,", "CMD,", "LIMIT,")
                ):
                    print(f"[mega] {line}")

                if telemetry is not None:
                    if telemetry.state == 4:
                        raise RuntimeError("Mega entered FAULTED state")
                    beams = (telemetry.entry_broken, telemetry.exit_broken)
                    if beams != previous_beams:
                        print(
                            "[ir] entry="
                            f"{'BROKEN' if beams[0] else 'CLEAR'} "
                            "exit="
                            f"{'BROKEN' if beams[1] else 'CLEAR'}"
                        )
                        previous_beams = beams
                    if now - last_feedback >= args.feedback_period:
                        state_name = {
                            0: "DISARMED",
                            1: "ARMED_IDLE",
                            2: "RUNNING",
                            3: "WAITING_FOR_BALL",
                            4: "FAULTED",
                        }.get(telemetry.state, str(telemetry.state))
                        print(
                            f"[status] {state_name} "
                            f"pwm(L/R)={telemetry.left_pwm}/{telemetry.right_pwm} "
                            f"rpm(L/R)={telemetry.left_rpm:.1f}/"
                            f"{telemetry.right_rpm:.1f} cycles={completed_cycles}"
                        )
                        last_feedback = now
                    beams_clear = not (
                        telemetry.entry_broken or telemetry.exit_broken
                    )
                    if (
                        feature_seen
                        and armed
                        and telemetry.state == 1  # ARMED_IDLE
                        and beams_clear
                        and not cycle_armed
                    ):
                        command = "AUTO_BENCH" if args.ignore_encoders else "AUTO"
                        if args.ignore_encoders and not bench_feature_seen:
                            raise RuntimeError(
                                "Mega firmware does not advertise safe bench mode"
                            )
                        send_line(port, f"{command} {args.pwm} {timeout_ms}")
                        cycle_armed = True

            if not feature_seen and now >= feature_deadline:
                raise RuntimeError(
                    "Mega firmware does not advertise 4 s AUTO/exit-stop support; "
                    "flash 07_dual_intake_mega_bench.ino from this checkout"
                )
            if now - last_rx > args.serial_watchdog:
                raise RuntimeError(
                    f"no Mega telemetry for {args.serial_watchdog:.1f} s"
                )

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
        return 0
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    finally:
        if port is not None:
            try:
                send_line(port, "STOP")
                time.sleep(0.05)
                send_line(port, "DISARM")
                time.sleep(0.05)
            except Exception:
                pass
            port.close()
            print("Mega STOP + DISARM sent; serial port closed.")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Start intake motors on entry IR; stop on exit IR or timeout."
    )
    parser.add_argument(
        "--port",
        default=os.getenv("COLLECTOR_SERIAL_PORT", "/dev/ttyACM0"),
        help="Mega serial device (default: COLLECTOR_SERIAL_PORT or /dev/ttyACM0)",
    )
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--pwm", type=int, default=60)
    parser.add_argument("--timeout", type=float, default=MAX_TIMEOUT_S)
    parser.add_argument("--once", action="store_true", help="exit after one cycle")
    parser.add_argument(
        "--ignore-encoders",
        action="store_true",
        help="bench only: keep the hard timeout but disable encoder jam faults",
    )
    parser.add_argument("--startup-delay", type=float, default=2.0)
    parser.add_argument("--serial-watchdog", type=float, default=2.0)
    parser.add_argument(
        "--feedback-period",
        type=float,
        default=1.0,
        help="seconds between human-readable PWM/RPM status lines",
    )
    parser.add_argument(
        "--raw", action="store_true", help="also print every raw DATA record"
    )
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    if not 1 <= args.pwm <= MAX_PWM:
        parser.error(f"--pwm must be in 1..{MAX_PWM}")
    if not 0.1 <= args.timeout <= MAX_TIMEOUT_S:
        parser.error(f"--timeout must be in 0.1..{MAX_TIMEOUT_S} seconds")
    if args.startup_delay < 0:
        parser.error("--startup-delay cannot be negative")
    if args.serial_watchdog < 0.5:
        parser.error("--serial-watchdog must be at least 0.5 seconds")
    if args.feedback_period <= 0:
        parser.error("--feedback-period must be positive")
    return run(args)


if __name__ == "__main__":
    raise SystemExit(main())
