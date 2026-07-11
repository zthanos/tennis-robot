#!/usr/bin/env python3
"""Evaluate dual-wheel intake transport criteria from contact + pose logs.

Criteria follow docs/intake-concept-decision-el.md (transport concept:
capture -> transport -> guide -> hopper). The --phase flag gates which
required criteria apply, matching the Concept Validation Plan:

  throat : both-wheel contact, capture through throat, inward transport,
           no stall/jam
  funnel : same as throat (lateral-offset cases come from the sweep config)
  ramp   : + ramp climb started, ramp crest crossing
  full   : same as ramp (hopper beams are confirmed by the controller path)

Repeatability (4/5 per condition) is judged across runs by the sweep
summary, not per-run here.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any


BALL_RADIUS_M = 0.033
DEFAULT_RELEASE_WINDOW_S = 0.20
DEFAULT_TRANSPORT_TARGET_M_S = 0.40
DEFAULT_MAX_CONTACT_DURATION_S = 0.50
DEFAULT_MIN_DIRECTIONAL_VELOCITY_M_S = 0.01
DEFAULT_STALL_SPEED_M_S = 0.02
DEFAULT_STALL_LIMIT_S = 2.0
DEFAULT_RAMP_CLIMB_Z_M = 0.050
DEFAULT_RAMP_CREST_Z_M = 0.138

PHASES = ("throat", "funnel", "ramp", "full")


def _load_jsonl(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    if not path.exists():
        return rows
    for line in path.read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        try:
            rows.append(json.loads(line))
        except json.JSONDecodeError:
            continue
    return rows


def _yaw_from_quat(q: list[float] | None) -> float:
    if not q or len(q) != 4:
        return 0.0
    x, y, z, w = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _base_pose(robot: dict[str, Any], ball: dict[str, Any]) -> tuple[float, float, float]:
    yaw = _yaw_from_quat(robot.get("q"))
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    dx = float(ball["x"]) - float(robot["x"])
    dy = float(ball["y"]) - float(robot["y"])
    return (
        cos_yaw * dx + sin_yaw * dy,
        -sin_yaw * dx + cos_yaw * dy,
        float(ball["z"]) - float(robot["z"]),
    )


def _pose_samples(pose_rows: list[dict[str, Any]], ball_name: str) -> list[dict[str, Any]]:
    latest_robot: dict[str, Any] | None = None
    latest_ball: dict[str, Any] | None = None
    samples: list[dict[str, Any]] = []
    for row in pose_rows:
        for pose in row.get("poses", []):
            name = pose.get("n")
            if name == "tennis_robot":
                latest_robot = pose
            elif name == ball_name:
                latest_ball = pose
        if latest_robot is None or latest_ball is None:
            continue
        bx, by, bz = _base_pose(latest_robot, latest_ball)
        samples.append(
            {
                "t_wall": float(row.get("t_wall", 0.0)),
                "t_sim": row.get("t_sim"),
                "base_xyz_m": [bx, by, bz],
                "world_xyz_m": [
                    float(latest_ball["x"]),
                    float(latest_ball["y"]),
                    float(latest_ball["z"]),
                ],
            }
        )
    velocities: list[dict[str, Any]] = []
    for prev, curr in zip(samples, samples[1:]):
        dt = curr["t_wall"] - prev["t_wall"]
        if dt <= 1e-6:
            continue
        prev_xyz = prev["base_xyz_m"]
        curr_xyz = curr["base_xyz_m"]
        vel = [(curr_xyz[i] - prev_xyz[i]) / dt for i in range(3)]
        velocities.append({**curr, "base_velocity_m_s": vel})
    return velocities


def _first_crossing(
    samples: list[dict[str, Any]],
    *,
    x_plane_m: float | None = None,
    z_plane_m: float | None = None,
    after_wall_s: float | None = None,
) -> dict[str, Any] | None:
    for sample in samples:
        if after_wall_s is not None and sample["t_wall"] < after_wall_s:
            continue
        x, _y, z = sample["base_xyz_m"]
        if x_plane_m is not None and x > x_plane_m:
            continue
        if z_plane_m is not None and z < z_plane_m:
            continue
        return sample
    return None


def _nearest_sample(samples: list[dict[str, Any]], t_wall: float) -> dict[str, Any] | None:
    if not samples:
        return None
    return min(samples, key=lambda sample: abs(sample["t_wall"] - t_wall))


def _percentile(values: list[float], pct: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    index = (len(ordered) - 1) * pct
    lower = math.floor(index)
    upper = math.ceil(index)
    if lower == upper:
        return ordered[int(index)]
    weight = index - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def _longest_stall_s(
    samples: list[dict[str, Any]],
    *,
    after_wall_s: float,
    zone_min_x_m: float,
    zone_max_x_m: float,
    stall_speed_m_s: float,
) -> tuple[float, dict[str, Any] | None]:
    """Longest continuous dwell inside the intake zone below stall speed."""
    longest = 0.0
    longest_at: dict[str, Any] | None = None
    run_start: float | None = None
    run_sample: dict[str, Any] | None = None
    for sample in samples:
        if sample["t_wall"] < after_wall_s:
            continue
        x = sample["base_xyz_m"][0]
        vel = sample.get("base_velocity_m_s")
        speed = math.sqrt(sum(v * v for v in vel)) if vel else None
        stalled = (
            zone_min_x_m <= x <= zone_max_x_m
            and speed is not None
            and speed < stall_speed_m_s
        )
        if stalled:
            if run_start is None:
                run_start = sample["t_wall"]
                run_sample = sample
            duration = sample["t_wall"] - run_start
            if duration > longest:
                longest = duration
                longest_at = run_sample
        else:
            run_start = None
            run_sample = None
    return longest, longest_at


def analyze(
    contact_jsonl: Path,
    pose_jsonl: Path,
    *,
    ball_name: str,
    phase: str,
    nip_x_m: float,
    wheel_radius_m: float,
    wheel_gap_m: float,
    ramp_climb_z_m: float,
    ramp_crest_z_m: float,
    release_window_s: float,
    preferred_contact_duration_s: float,
    transport_target_m_s: float,
    min_directional_velocity_m_s: float,
    stall_speed_m_s: float,
    stall_limit_s: float,
    force_p95_threshold_n: float | None,
) -> dict[str, Any]:
    all_contact_rows = _load_jsonl(contact_jsonl)
    contact_rows = [
        row
        for row in all_contact_rows
        if row.get("type") == "roller_contact_sample" and row.get("ball") == ball_name
    ]
    left_rows = [row for row in contact_rows if row.get("wheel") == "left"]
    right_rows = [row for row in contact_rows if row.get("wheel") == "right"]
    ramp_guide_rows = [
        row
        for row in all_contact_rows
        if row.get("type") in ("lip_contact_sample", "ramp_guide_contact_sample")
        and row.get("ball") == ball_name
    ]
    pose_rows = _load_jsonl(pose_jsonl)
    pose_samples = _pose_samples(pose_rows, ball_name)

    # Throat geometry: a centred ball first touches the wheels bite_dx ahead
    # of the nip plane and is fully through at nip - bite_dx.
    wheel_y_m = wheel_gap_m / 2.0 + wheel_radius_m
    reach = wheel_radius_m + BALL_RADIUS_M
    bite_dx_m = math.sqrt(max(0.0, reach**2 - wheel_y_m**2))
    throat_exit_x_m = nip_x_m - bite_dx_m
    first_touch_x_m = nip_x_m + bite_dx_m

    contact_wall_times = [
        float(row["t_wall"]) for row in contact_rows if row.get("t_wall") is not None
    ]
    contact_elapsed_times = [
        float(row["t_s"]) for row in contact_rows if row.get("t_s") is not None
    ]
    forces = [
        float(row["max_force_n"])
        for row in contact_rows
        if row.get("max_force_n") is not None
    ]
    force_p95 = _percentile(forces, 0.95)
    contact_duration_s = (
        max(contact_elapsed_times) - min(contact_elapsed_times)
        if contact_elapsed_times
        else 0.0
    )
    first_contact_wall_s = min(contact_wall_times) if contact_wall_times else None
    release_wall_s = max(contact_wall_times) if contact_wall_times else None

    # Capture: ball centre fully through the throat.
    capture = _first_crossing(
        pose_samples,
        x_plane_m=throat_exit_x_m,
        after_wall_s=first_contact_wall_s,
    )

    # Transport: peak inward (-base_vx) velocity between first contact and
    # capture (or end of log if the ball never got through).
    window_end_wall_s = capture["t_wall"] if capture else None
    transport_peak_inward_m_s: float | None = None
    if first_contact_wall_s is not None:
        for sample in pose_samples:
            if sample["t_wall"] < first_contact_wall_s:
                continue
            if window_end_wall_s is not None and sample["t_wall"] > window_end_wall_s + 0.5:
                break
            inward = -sample["base_velocity_m_s"][0]
            if transport_peak_inward_m_s is None or inward > transport_peak_inward_m_s:
                transport_peak_inward_m_s = inward
    capture_inward_m_s = -capture["base_velocity_m_s"][0] if capture else None

    # Stall/jam: continuous low-speed dwell inside the intake zone.
    stall_s = 0.0
    stall_at: dict[str, Any] | None = None
    if first_contact_wall_s is not None:
        stall_s, stall_at = _longest_stall_s(
            pose_samples,
            after_wall_s=first_contact_wall_s,
            zone_min_x_m=throat_exit_x_m - 0.10,
            zone_max_x_m=first_touch_x_m + 0.02,
            stall_speed_m_s=stall_speed_m_s,
        )

    # Ramp criteria (phases ramp/full): climb started + crest crossing.
    ramp_climb = _first_crossing(
        pose_samples,
        z_plane_m=ramp_climb_z_m,
        after_wall_s=first_contact_wall_s,
    )
    ramp_crest = _first_crossing(
        pose_samples,
        z_plane_m=ramp_crest_z_m,
        after_wall_s=first_contact_wall_s,
    )

    # Release diagnostics (kept for debugging; not required criteria).
    release_sample = _nearest_sample(pose_samples, release_wall_s) if release_wall_s else None
    release_velocity = release_sample.get("base_velocity_m_s") if release_sample else None
    release_speed_m_s = (
        math.sqrt(sum(v * v for v in release_velocity)) if release_velocity else None
    )

    required: dict[str, Any] = {
        "confirmed_contact_with_both_rollers": bool(left_rows) and bool(right_rows),
        "capture_through_wheel_throat": capture is not None,
        "positive_inward_transport": (
            capture_inward_m_s is not None
            and capture_inward_m_s >= min_directional_velocity_m_s
        ),
        "no_stall_or_jam": stall_s <= stall_limit_s,
    }
    if phase in ("ramp", "full"):
        required["ramp_climb_started"] = ramp_climb is not None
        required["hopper_entry_or_ramp_crest_crossing"] = ramp_crest is not None

    preferred = {
        "transport_speed_gte_target": (
            transport_peak_inward_m_s is not None
            and transport_peak_inward_m_s >= transport_target_m_s
        ),
        "contact_duration_lt_limit": contact_duration_s < preferred_contact_duration_s,
        "force_p95_below_threshold": (
            None
            if force_p95_threshold_n is None or force_p95 is None
            else force_p95 < force_p95_threshold_n
        ),
    }

    required_pass = sum(1 for value in required.values() if value is True)

    return {
        "contact_log": str(contact_jsonl),
        "pose_log": str(pose_jsonl),
        "ball_name": ball_name,
        "phase": phase,
        "parameters": {
            "nip_x_m": nip_x_m,
            "wheel_radius_m": wheel_radius_m,
            "wheel_gap_m": wheel_gap_m,
            "bite_dx_m": round(bite_dx_m, 5),
            "throat_exit_x_m": round(throat_exit_x_m, 5),
            "first_touch_x_m": round(first_touch_x_m, 5),
            "ramp_climb_z_m": ramp_climb_z_m,
            "ramp_crest_z_m": ramp_crest_z_m,
            "release_window_s": release_window_s,
            "preferred_contact_duration_s": preferred_contact_duration_s,
            "transport_target_m_s": transport_target_m_s,
            "min_directional_velocity_m_s": min_directional_velocity_m_s,
            "stall_speed_m_s": stall_speed_m_s,
            "stall_limit_s": stall_limit_s,
            "force_p95_threshold_n": force_p95_threshold_n,
        },
        "measurements": {
            "contact_samples": len(contact_rows),
            "wheel_left_contact_samples": len(left_rows),
            "wheel_right_contact_samples": len(right_rows),
            "ramp_guide_contact_samples": len(ramp_guide_rows),
            "contact_duration_s": round(contact_duration_s, 4),
            "first_contact_t_s": round(min(contact_elapsed_times), 4)
            if contact_elapsed_times
            else None,
            "last_contact_t_s": round(max(contact_elapsed_times), 4)
            if contact_elapsed_times
            else None,
            "force_p95_n": round(force_p95, 4) if force_p95 is not None else None,
            "force_max_n": round(max(forces), 4) if forces else None,
            "capture_crossing": capture,
            "capture_inward_velocity_m_s": capture_inward_m_s,
            "transport_peak_inward_m_s": transport_peak_inward_m_s,
            "longest_stall_s": round(stall_s, 4),
            "longest_stall_at": stall_at,
            "ramp_climb_crossing": ramp_climb,
            "ramp_crest_crossing": ramp_crest,
            "release_wall_s": round(release_wall_s, 6) if release_wall_s else None,
            "release_base_xyz_m": release_sample.get("base_xyz_m") if release_sample else None,
            "release_base_velocity_m_s": release_velocity,
            "release_speed_m_s": release_speed_m_s,
        },
        "required": required,
        "required_pass": f"{required_pass}/{len(required)}",
        "preferred": preferred,
        "notes": {
            "inward_sign": "positive inward speed is computed as -base_vx because inward is toward smaller base_x",
            "capture": "capture = ball centre crosses the throat exit plane (nip - bite_dx) after first wheel contact",
            "stall": "stall = longest continuous dwell below stall speed inside the intake zone after first contact",
            "vertical_at_release": "NOT a criterion: it belonged to the old launch concept; elevation is the ramp's job",
            "repeatability": "4/5-per-condition repeatability is evaluated across runs by the sweep summary",
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("contact_jsonl", type=Path)
    parser.add_argument("pose_jsonl", type=Path)
    parser.add_argument("--ball-name", default="ball_02")
    parser.add_argument("--phase", choices=PHASES, default="throat")
    parser.add_argument("--nip-x-m", type=float, default=0.590)
    parser.add_argument("--wheel-radius-m", type=float, default=0.060)
    parser.add_argument("--wheel-gap-m", type=float, default=0.060)
    parser.add_argument("--ramp-climb-z-m", type=float, default=DEFAULT_RAMP_CLIMB_Z_M)
    parser.add_argument("--ramp-crest-z-m", type=float, default=DEFAULT_RAMP_CREST_Z_M)
    parser.add_argument("--release-window-s", type=float, default=DEFAULT_RELEASE_WINDOW_S)
    parser.add_argument(
        "--preferred-contact-duration-s",
        type=float,
        default=DEFAULT_MAX_CONTACT_DURATION_S,
    )
    parser.add_argument(
        "--transport-target-m-s", type=float, default=DEFAULT_TRANSPORT_TARGET_M_S
    )
    parser.add_argument(
        "--min-directional-velocity-m-s",
        type=float,
        default=DEFAULT_MIN_DIRECTIONAL_VELOCITY_M_S,
    )
    parser.add_argument("--stall-speed-m-s", type=float, default=DEFAULT_STALL_SPEED_M_S)
    parser.add_argument("--stall-limit-s", type=float, default=DEFAULT_STALL_LIMIT_S)
    parser.add_argument("--force-p95-threshold-n", type=float, default=None)
    parser.add_argument("--json-out", type=Path)
    args = parser.parse_args()

    result = analyze(
        args.contact_jsonl,
        args.pose_jsonl,
        ball_name=args.ball_name,
        phase=args.phase,
        nip_x_m=args.nip_x_m,
        wheel_radius_m=args.wheel_radius_m,
        wheel_gap_m=args.wheel_gap_m,
        ramp_climb_z_m=args.ramp_climb_z_m,
        ramp_crest_z_m=args.ramp_crest_z_m,
        release_window_s=args.release_window_s,
        preferred_contact_duration_s=args.preferred_contact_duration_s,
        transport_target_m_s=args.transport_target_m_s,
        min_directional_velocity_m_s=args.min_directional_velocity_m_s,
        stall_speed_m_s=args.stall_speed_m_s,
        stall_limit_s=args.stall_limit_s,
        force_p95_threshold_n=args.force_p95_threshold_n,
    )
    text = json.dumps(result, indent=2, sort_keys=True)
    if args.json_out:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(text + "\n", encoding="utf-8")
    print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
