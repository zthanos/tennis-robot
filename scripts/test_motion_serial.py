#!/usr/bin/env python3
"""Bench test for the Mega drive motion firmware (arduino/motion/motion_mega).

Talks to the Mega over USB serial (115200) with the same line protocol the Pi
will use later. Streams a heartbeat so the firmware's command timeout never trips
while you hold a duty, reads telemetry in the background, and ALWAYS stops +
disarms on exit (Ctrl-C, quit, or crash).

SAFETY: wheels OFF the ground for first runs. Start with low duty. One side at a
time to confirm direction + encoder sign before driving both.

Usage:
    uv run python scripts/test_motion_serial.py --port /dev/ttyACM0
    uv run python scripts/test_motion_serial.py --port COM5         # Windows

REPL commands:
    arm                 arm the drivers
    disarm              disarm + stop
    m <left> <right>    per-side duty, floats in [-1,1]   e.g.  m 0.3 0.3
    f [d]               forward both at d (default 0.25)
    b [d]               backward both at d
    l [d] / r [d]       spin left / right in place at d
    stop                duty 0 (stays armed)
    s                   alias for stop
    z                   zero encoders display baseline
    t                   print latest telemetry once
    quiet / loud        hide / show live telemetry
    quit / q            stop, disarm, exit
"""

from __future__ import annotations

import argparse
import sys
import threading
import time

try:
    import serial  # pyserial
except ImportError:
    sys.exit("pyserial not installed.  pip install pyserial  (or: uv add pyserial)")


class MotionLink:
    def __init__(self, port: str, baud: int = 115200) -> None:
        self._ser = serial.Serial(port, baud, timeout=0.1)
        self._target = (0.0, 0.0)
        self._armed = False
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._show_telem = True
        self._last_telem = ""
        self._enc_base = (0, 0, 0, 0)
        time.sleep(2.0)  # Mega resets on serial open; wait for boot
        self._ser.reset_input_buffer()
        self._rx = threading.Thread(target=self._rx_loop, daemon=True)
        self._tx = threading.Thread(target=self._heartbeat_loop, daemon=True)
        self._rx.start()
        self._tx.start()

    # -- low level --
    def _send(self, line: str) -> None:
        with self._lock:
            self._ser.write((line + "\n").encode("ascii"))

    def _rx_loop(self) -> None:
        while not self._stop.is_set():
            try:
                raw = self._ser.readline().decode("ascii", "replace").strip()
            except Exception:
                break
            if not raw:
                continue
            if raw.startswith("T "):
                self._last_telem = raw
                if self._show_telem:
                    self._print_telem(raw)
            else:
                print(f"\n[mega] {raw}")
                self._reprompt()

    def _heartbeat_loop(self) -> None:
        # Stream the current target at 20 Hz so CMD_TIMEOUT_MS never trips while
        # armed; send PING when disarmed to keep the link warm.
        while not self._stop.is_set():
            if self._armed:
                l, r = self._target
                self._send(f"M {l:.3f} {r:.3f}")
            else:
                self._send("PING")
            time.sleep(0.05)

    # -- telemetry display --
    def _print_telem(self, raw: str) -> None:
        p = raw.split()
        if len(p) != 9:
            return
        state = {"0": "DISARMED", "1": "ARMED", "2": "ESTOP"}.get(p[1], p[1])
        b = self._enc_base
        enc = [int(p[4]) - b[0], int(p[5]) - b[1], int(p[6]) - b[2], int(p[7]) - b[3]]
        estop = "ESTOP!" if p[8] == "1" else "ok"
        sys.stdout.write(
            f"\r{state:8} L={float(p[2]):+.2f} R={float(p[3]):+.2f} "
            f"enc[LF,LR,RF,RR]={enc} estop={estop}   "
        )
        sys.stdout.flush()

    def _reprompt(self) -> None:
        sys.stdout.write("> ")
        sys.stdout.flush()

    # -- commands --
    def arm(self) -> None:
        self._send("ARM")
        self._armed = True

    def disarm(self) -> None:
        self._armed = False
        self._target = (0.0, 0.0)
        self._send("DISARM")

    def set_duty(self, l: float, r: float) -> None:
        l = max(-1.0, min(1.0, l))
        r = max(-1.0, min(1.0, r))
        self._target = (l, r)

    def stop_motion(self) -> None:
        self._target = (0.0, 0.0)

    def zero_encoders(self) -> None:
        p = self._last_telem.split()
        if len(p) == 9:
            self._enc_base = (int(p[4]), int(p[5]), int(p[6]), int(p[7]))
            print("\nEncoder baseline zeroed.")

    def close(self) -> None:
        # belt-and-suspenders shutdown
        try:
            self._target = (0.0, 0.0)
            self._send("STOP")
            time.sleep(0.1)
            self._send("DISARM")
            time.sleep(0.1)
        finally:
            self._stop.set()
            time.sleep(0.15)
            self._ser.close()


def _f(args: list[str], default: float) -> float:
    try:
        return float(args[0]) if args else default
    except ValueError:
        return default


def main() -> None:
    ap = argparse.ArgumentParser(description="Bench test the Mega drive firmware.")
    ap.add_argument("--port", required=True, help="serial port (e.g. /dev/ttyACM0, COM5)")
    ap.add_argument("--baud", type=int, default=115200)
    args = ap.parse_args()

    print(f"Connecting to {args.port} @ {args.baud} ...")
    link = MotionLink(args.port, args.baud)
    print(__doc__.split("REPL commands:")[1])
    print("Wheels OFF the ground for first runs. Type 'arm' then e.g. 'f 0.2'.\n")

    try:
        while True:
            try:
                line = input("> ").strip()
            except EOFError:
                break
            if not line:
                continue
            parts = line.split()
            cmd, rest = parts[0].lower(), parts[1:]

            if cmd in ("quit", "q"):
                break
            elif cmd == "arm":
                link.arm()
            elif cmd == "disarm":
                link.disarm()
            elif cmd == "m" and len(rest) == 2:
                link.set_duty(float(rest[0]), float(rest[1]))
            elif cmd == "f":
                d = _f(rest, 0.25); link.set_duty(d, d)
            elif cmd == "b":
                d = _f(rest, 0.25); link.set_duty(-d, -d)
            elif cmd == "l":
                d = _f(rest, 0.25); link.set_duty(-d, d)
            elif cmd == "r":
                d = _f(rest, 0.25); link.set_duty(d, -d)
            elif cmd in ("stop", "s"):
                link.stop_motion()
            elif cmd == "z":
                link.zero_encoders()
            elif cmd == "t":
                print(link._last_telem or "(no telemetry yet)")
            elif cmd == "quiet":
                link._show_telem = False
            elif cmd == "loud":
                link._show_telem = True
            else:
                print("?  commands: arm disarm m f b l r stop z t quiet loud quit")
    except KeyboardInterrupt:
        pass
    finally:
        print("\nStopping + disarming ...")
        link.close()
        print("Done.")


if __name__ == "__main__":
    main()
