#!/usr/bin/env python3
"""Console PoC runner for the perimeter court survey.

The script is intentionally file-bus only: it does not import ROS 2 or tennis_robot
modules. Run it inside the Gazebo container while the normal sim stack is up; the
command bridge will pick up runtime/robot_command.json and the controller will
publish runtime/robot_status.json.
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path
from typing import Any


DEFAULT_ROOT = Path("/workspace")
DEFAULT_COMMAND_FILE = DEFAULT_ROOT / "runtime" / "robot_command.json"
DEFAULT_STATUS_FILE = DEFAULT_ROOT / "runtime" / "robot_status.json"
DEFAULT_SUMMARY_FILE = DEFAULT_ROOT / "runtime" / "perimeter_survey_poc_summary.json"
DEFAULT_BOUNDS_FILE = DEFAULT_ROOT / "runtime" / "court_boundary.json"


def _read_json(path: Path) -> dict[str, Any] | None:
    try:
        data = json.loads(path.read_text(encoding="utf-8-sig"))
    except (FileNotFoundError, OSError, json.JSONDecodeError):
        return None
    return data if isinstance(data, dict) else None


def _write_json_atomic(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(f"{path.name}.{time.time_ns()}.tmp")
    tmp.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    tmp.replace(path)


def _write_command(path: Path, mode: str, source: str) -> dict[str, Any]:
    current = _read_json(path) or {}
    command = {
        "mode": mode,
        "sequence": int(current.get("sequence", 0)) + 1,
        "source": source,
        "updated_at": time.time(),
    }
    _write_json_atomic(path, command)
    return command


def _survey_payload(status: dict[str, Any]) -> dict[str, Any]:
    survey = status.get("survey")
    return survey if isinstance(survey, dict) else {}


def _navigation_payload(status: dict[str, Any]) -> dict[str, Any]:
    navigation = _survey_payload(status).get("navigation")
    return navigation if isinstance(navigation, dict) else {}


def _bounds_payload(status: dict[str, Any]) -> dict[str, Any] | None:
    bounds = _survey_payload(status).get("bounds")
    return bounds if isinstance(bounds, dict) else None


def _status_age_s(status: dict[str, Any], now: float) -> float | None:
    try:
        return max(0.0, now - float(status["updated_at"]))
    except (KeyError, TypeError, ValueError):
        return None


def _print_progress(status: dict[str, Any], started_at: float) -> None:
    nav = _navigation_payload(status)
    state = _survey_payload(status).get("state") or nav.get("state") or "unknown"
    event = nav.get("last_event") or "none"
    points = nav.get("survey_navigation_point_count")
    distance = nav.get("distance_traveled_m")
    front = nav.get("front_lidar_range_m")
    elapsed = int(time.time() - started_at)
    print(
        f"[{elapsed:04d}s] state={state} event={event} "
        f"points={points if points is not None else '-'} "
        f"distance_m={distance if distance is not None else '-'} "
        f"front_m={front if front is not None else '-'}",
        flush=True,
    )


def _build_summary(
    *,
    outcome: str,
    started_at: float,
    command_file: Path,
    status_file: Path,
    bounds_file: Path,
    last_status: dict[str, Any] | None,
) -> dict[str, Any]:
    now = time.time()
    nav = _navigation_payload(last_status or {})
    bounds = _bounds_payload(last_status or {}) or {}
    return {
        "outcome": outcome,
        "started_at": started_at,
        "finished_at": now,
        "elapsed_s": round(now - started_at, 1),
        "command_file": str(command_file),
        "status_file": str(status_file),
        "bounds_file": str(bounds_file),
        "survey_state": (_survey_payload(last_status or {}).get("state") or nav.get("state")),
        "survey_event": nav.get("last_event"),
        "survey_complete": bool(bounds.get("survey_complete")),
        "survey_status": bounds.get("status"),
        "failure_reason": bounds.get("failure_reason"),
        "navigation_point_count": nav.get("survey_navigation_point_count"),
        "distance_traveled_m": nav.get("distance_traveled_m"),
        "bounds": bounds or None,
    }


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Start and monitor the perimeter court survey PoC via runtime JSON files."
    )
    parser.add_argument("--command-file", type=Path, default=DEFAULT_COMMAND_FILE)
    parser.add_argument("--status-file", type=Path, default=DEFAULT_STATUS_FILE)
    parser.add_argument("--summary-file", type=Path, default=DEFAULT_SUMMARY_FILE)
    parser.add_argument("--bounds-file", type=Path, default=DEFAULT_BOUNDS_FILE)
    parser.add_argument("--timeout-s", type=float, default=600.0)
    parser.add_argument("--poll-s", type=float, default=1.0)
    parser.add_argument("--stale-after-s", type=float, default=8.0)
    parser.add_argument("--missing-status-grace-s", type=float, default=12.0)
    parser.add_argument("--startup-grace-s", type=float, default=20.0)
    parser.add_argument("--no-stop-on-exit", action="store_true")
    parser.add_argument("--source", default="perimeter-survey-poc")
    args = parser.parse_args()

    started_at = time.time()
    print("Starting perimeter survey PoC: mode=map_court", flush=True)
    _write_command(args.command_file, "map_court", args.source)

    last_status: dict[str, Any] | None = None
    last_status_seen_at: float | None = None
    last_progress_key: tuple[Any, Any, Any] | None = None
    outcome = "timeout"
    saw_survey_active = False

    try:
        while time.time() - started_at <= args.timeout_s:
            now = time.time()
            status = _read_json(args.status_file)
            if status is None:
                if last_status is not None and last_status_seen_at is not None:
                    if now - last_status_seen_at <= args.missing_status_grace_s:
                        time.sleep(max(0.1, args.poll_s))
                        continue
                    outcome = "status_unavailable"
                    break
                if now - started_at > args.startup_grace_s:
                    outcome = "no_status"
                    break
                time.sleep(max(0.1, args.poll_s))
                continue

            last_status = status
            last_status_seen_at = now
            age = _status_age_s(status, now)
            if age is not None and age > args.stale_after_s and now - started_at > args.startup_grace_s:
                outcome = "stale_status"
                break

            nav = _navigation_payload(status)
            if status.get("requested_mode") == "map_court" or status.get("mode") == "map_court":
                saw_survey_active = True
            key = (
                _survey_payload(status).get("state") or nav.get("state"),
                nav.get("last_event"),
                nav.get("survey_navigation_point_count"),
            )
            if key != last_progress_key:
                _print_progress(status, started_at)
                last_progress_key = key

            bounds = _bounds_payload(status)
            if bounds and bounds.get("survey_complete") is True:
                outcome = "success"
                _write_json_atomic(args.bounds_file, bounds)
                break
            if bounds and bounds.get("status") == "PARTIAL":
                outcome = "partial"
                _write_json_atomic(args.bounds_file, bounds)
                break

            if status.get("requested_mode") == "idle" and status.get("mode") == "idle":
                if not saw_survey_active and now - started_at <= args.startup_grace_s:
                    time.sleep(max(0.1, args.poll_s))
                    continue
                survey_state = _survey_payload(status).get("state") or nav.get("state")
                if survey_state == "done":
                    outcome = "done_without_bounds"
                else:
                    outcome = "stopped"
                break

            time.sleep(max(0.1, args.poll_s))
    finally:
        if not args.no_stop_on_exit:
            _write_command(args.command_file, "idle", f"{args.source}-cleanup")

    summary = _build_summary(
        outcome=outcome,
        started_at=started_at,
        command_file=args.command_file,
        status_file=args.status_file,
        bounds_file=args.bounds_file,
        last_status=last_status,
    )
    _write_json_atomic(args.summary_file, summary)
    print(f"PoC outcome: {outcome}", flush=True)
    print(f"Summary: {args.summary_file}", flush=True)
    if summary["bounds"]:
        print(f"Bounds: {args.bounds_file}", flush=True)

    return 0 if outcome == "success" else 2


if __name__ == "__main__":
    raise SystemExit(main())
