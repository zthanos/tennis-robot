#!/usr/bin/env python3
"""Live collect-mission tracer: logs CHANGES instead of snapshots.

Polls runtime/robot_status.json and prints a timestamped line whenever
something meaningful changes: collector state, mode, new collection events,
collection count, or ball distance moving by more than a step. Tracks the
minimum ball distance per collect attempt — the key number for diagnosing
"pushes the ball but never captures" (lip contact is at 0.6925 m from base
centre; the intake zone upper bound is 0.72 m).

Usage:  python3 scripts/watch_collect.py [--status runtime/robot_status.json]
Stop with Ctrl-C (prints a summary).
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

DIST_STEP_M = 0.05
LIP_CONTACT_M = 0.6925


def read_status(path: Path) -> dict | None:
    for _ in range(10):
        try:
            return json.loads(path.read_text())
        except (json.JSONDecodeError, OSError):
            time.sleep(0.03)
    return None


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--status", default="runtime/robot_status.json")
    ap.add_argument("--interval", type=float, default=0.2)
    args = ap.parse_args()
    path = Path(args.status)

    last_state = last_mode = None
    last_dist_bucket: int | None = None
    last_count = None
    seen_events: set[tuple] = set()
    attempt_min_dist: float | None = None
    attempt_started: float | None = None
    mins: list[float] = []

    def log(msg: str) -> None:
        print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)

    log(f"watching {path} (dist step {DIST_STEP_M} m, lip contact {LIP_CONTACT_M} m)")
    try:
        while True:
            d = read_status(path)
            if d is None:
                time.sleep(args.interval)
                continue

            state = d.get("collector_state")
            mode = d.get("mode")
            dist = d.get("ball_distance_m")
            count = d.get("collection_count")

            if mode != last_mode:
                log(f"mode: {last_mode} -> {mode}")
                last_mode = mode
            if state != last_state:
                extra = f" (dist {dist:.3f})" if isinstance(dist, float) else ""
                log(f"state: {last_state} -> {state}{extra}")
                last_state = state

            if isinstance(dist, float):
                if attempt_min_dist is None or dist < attempt_min_dist:
                    attempt_min_dist = dist
                bucket = int(dist / DIST_STEP_M)
                if bucket != last_dist_bucket:
                    marker = "  <-- INSIDE capture zone" if dist <= 0.72 else (
                        "  <-- at lip" if dist <= 0.78 else "")
                    log(f"dist: {dist:.3f}{marker}")
                    last_dist_bucket = bucket

            for e in d.get("collection_events", []):
                key = (e.get("t_s"), e.get("type"))
                if key in seen_events:
                    continue
                seen_events.add(key)
                detail = {k: v for k, v in e.items()
                          if k in ("reason", "ball_distance_m", "detail", "seeded", "elapsed_s")}
                log(f"EVENT {e.get('t_s')} {e.get('type')} {detail if detail else ''}")
                if e.get("type", "").endswith(("collect_start",)):
                    attempt_min_dist = None
                    attempt_started = e.get("t_s")
                elif e.get("type", "").endswith(("timeout", "gave_up", "abort", "confirmed")):
                    if attempt_min_dist is not None:
                        mins.append(attempt_min_dist)
                        verdict = ("reached capture zone" if attempt_min_dist <= 0.72
                                   else "stalled at/BEFORE the lip")
                        log(f"  attempt (t={attempt_started}) min dist {attempt_min_dist:.3f} -> {verdict}")
                    attempt_min_dist = None

            if count != last_count:
                log(f"collected: {last_count} -> {count}")
                last_count = count

            time.sleep(args.interval)
    except KeyboardInterrupt:
        if mins:
            log(f"summary: {len(mins)} attempts, min dists: {[round(m,3) for m in mins]}")


if __name__ == "__main__":
    main()
