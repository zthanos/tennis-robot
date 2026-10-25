#!/usr/bin/env python3
"""No-motor bench checks for drive_test_mega, run on the Pi.

The firmware faults after 300 ms without host traffic, so a plain serial
terminal cannot hold it ARMED.  This helper keeps the heartbeat alive and
prints a compact view of the telemetry.

  monitor  - never arms from the host; E-stop, encoders by hand, gyro, and
             the START button (with --cpr the Mega may arm itself on a
             press; PING keeps it armed, output stays zero: no CMD is sent)
  pwm      - arms with a low PWM ceiling and streams one CMD, then disarms;
             with no motors the duty rises to the ceiling, which is what you
             measure on RPWM/LPWM (logic) or M+/M- (BTS powered, no motor)
"""

from __future__ import annotations

import argparse
import sys
import time

STATES = {"0": "DISARMED", "1": "ARMED", "2": "ESTOPPED", "3": "FAULTED"}
# Field order of the "T," line in drive_test_mega.ino (after the "T" tag).
FIELDS = (
    "ms", "state", "mode", "cmd_v", "cmd_w", "ramp_v", "ramp_w",
    "duty_l", "duty_r", "lf", "lr", "rf", "rr", "w_lf", "w_lr",
    "w_rf", "w_rr", "gz_rad_s", "yaw_rad", "x_m", "y_m", "estop",
    "max_loop_gap_us", "max_telem_write_us",
)


def open_port(path: str, baud: int):
    import serial

    port = serial.Serial(path, baud, timeout=0.02, exclusive=True)
    time.sleep(2.0)  # the Mega resets when the port opens
    return port


def send(port, command: str) -> None:
    port.write((command + "\n").encode("ascii"))
    port.flush()


def parse(line: str):
    parts = line.split(",")
    if parts[0] != "T" or len(parts) - 1 < len(FIELDS):
        return None
    return dict(zip(FIELDS, parts[1:]))


def show(t: dict) -> str:
    state = STATES.get(t["state"], t["state"])
    estop = "OK" if t["estop"] == "0" else "OPEN"
    return (
        f"{state:9s} ESTOP={estop:4s} duty L={float(t['duty_l']):+.3f} "
        f"R={float(t['duty_r']):+.3f} | enc LF={t['lf']:>6} LR={t['lr']:>6} "
        f"RF={t['rf']:>6} RR={t['rr']:>6} | gz={float(t['gz_rad_s']):+.3f} rad/s "
        f"yaw={float(t['yaw_rad']):+.3f} rad | gap={t['max_loop_gap_us']} us"
    )


def pump(port, until: float, heartbeat: str, period: float = 0.05,
         print_every: float = 0.5) -> dict | None:
    """Read lines until `until`, sending `heartbeat` every `period` s."""
    last_sent = 0.0
    last_print = 0.0
    last = None
    while time.monotonic() < until:
        now = time.monotonic()
        if now - last_sent >= period:
            send(port, heartbeat)
            last_sent = now
        raw = port.readline().decode("ascii", "replace").strip()
        if not raw:
            continue
        telemetry = parse(raw)
        if telemetry is None:
            if raw not in ("PONG", "OK,CMD"):
                print(f"  << {raw}")
            continue
        last = telemetry
        if now - last_print >= print_every:
            print("  " + show(telemetry))
            last_print = now
    return last


def monitor(port, args) -> int:
    if args.zero:
        send(port, "ZERO")
        send(port, "ZERO_GYRO")
    if args.cpr:
        # Needed only for the START button test: the Mega refuses to arm
        # without CPR.  No CMD is ever sent, so outputs stay at zero.
        send(port, f"CFG CPR {args.cpr}")
        send(port, "CFG PWM 0")
    print("monitor: το Pi δεν οπλίζει. Ctrl+C για έξοδο (DISARM στο τέλος).")
    try:
        pump(port, time.monotonic() + args.seconds, "PING")
    except KeyboardInterrupt:
        pass
    finally:
        send(port, "DISARM")
    return 0


def pwm(port, args) -> int:
    if args.pwm_ceiling > 0.3:
        print("Άρνηση: --pwm-ceiling > 0.3 δεν χρειάζεται για bench χωρίς μοτέρ.")
        return 2
    for command in (
        f"CFG CPR {args.cpr}",
        f"CFG PWM {args.pwm_ceiling}",
        f"LIMIT {args.seconds + 1.0} 0.5",
        "MODE OPEN",
        "ZERO",
        "ARM",
    ):
        send(port, command)
        time.sleep(0.05)
    print(f"pwm: CMD {args.v} {args.w}, οροφή PWM {args.pwm_ceiling}, "
          f"{args.seconds} s. Μέτρα τώρα.")
    try:
        last = pump(port, time.monotonic() + args.seconds,
                    f"CMD {args.v} {args.w}")
    except KeyboardInterrupt:
        last = None
    finally:
        send(port, "STOP")
        send(port, "DISARM")
        time.sleep(0.2)
    if last is not None:
        print("τελευταία τιμή: " + show(last))
    print("STOP + DISARM στάλθηκαν.")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=500000)
    sub = parser.add_subparsers(dest="action", required=True)

    mon = sub.add_parser("monitor")
    mon.add_argument("--seconds", type=float, default=600.0)
    mon.add_argument("--zero", action="store_true",
                     help="μηδενισμός encoders και gyro στην αρχή")
    mon.add_argument("--cpr", type=float, default=0.0,
                     help="για τη δοκιμή START: επιτρέπει όπλιση από το κουμπί (PWM 0)")

    test = sub.add_parser("pwm")
    test.add_argument("--cpr", type=float, required=True,
                      help="counts ανά περιστροφή τροχού (μετρημένο στο monitor)")
    test.add_argument("--v", type=float, default=0.1, help="m/s (+ μπροστά)")
    test.add_argument("--w", type=float, default=0.0, help="rad/s (+ αριστερά)")
    test.add_argument("--pwm-ceiling", type=float, default=0.2)
    test.add_argument("--seconds", type=float, default=3.0)

    args = parser.parse_args()
    port = open_port(args.port, args.baud)
    try:
        return monitor(port, args) if args.action == "monitor" else pwm(port, args)
    finally:
        port.close()


if __name__ == "__main__":
    sys.exit(main())
