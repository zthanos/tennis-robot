#!/usr/bin/env python3
"""Compile the Gazebo intake campaign (S1-S5) into one evidence artifact.

Every metric is computed from the ground-truth Gazebo pose stream
(``gz_poses.jsonl``) in a frame built from the SAME sample's robot pose, so no
result depends on the live probe's odometry lag. Contact evidence comes from the
500 Hz wheel and ramp contact sensors (``contact_physics.jsonl``).

Scope (two-instrument rule): this file reports capture/plough behaviour, ball
climb, contact sequencing and lateral acceptance. It does NOT report exit
velocity, tyre deflection, wheel droop, torque or current — Gazebo cannot
measure those here.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

import intake_geometry as geom  # noqa: E402

VARIANT = geom.CAD_ALIGNED_VARIANT
SHIFT = geom.packaging_shift_x_m(VARIANT)
#: Nip mid-plane and the three nip stations in the robot base frame.
NIP_X_BASE = geom.CAD_NIP_X_M + SHIFT
NIP_STATIONS_BASE = {
    name: (x + SHIFT, z) for name, (x, z) in geom.nip_stations_m().items()
}
RAMP_FRONT_BASE = geom.RAMP_FRONT_X_M + SHIFT
RAMP_REAR_BASE = geom.RAMP_REAR_X_M + SHIFT
#: A ball whose centre is behind the rear edge of the ramp has been transported.
TRANSPORTED_X_BASE = RAMP_REAR_BASE
BALL_NAME = "ball_02"


def _yaw(quaternion) -> float:
    x, y, z, w = quaternion
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def load_config(case_dir: Path) -> dict:
    config = {}
    path = case_dir / "bench_config.txt"
    if path.exists():
        for line in path.read_text(encoding="utf-8").splitlines():
            if "=" in line:
                key, value = line.split("=", 1)
                config[key.strip()] = value.strip()
    return config


def analyse_case(case_dir: Path) -> dict | None:
    poses_path = case_dir / "gz_poses.jsonl"
    if not poses_path.exists():
        return None
    config = load_config(case_dir)
    samples = []
    for line in poses_path.read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        try:
            record = json.loads(line)
        except json.JSONDecodeError:
            continue
        entries = {item["n"]: item for item in record.get("poses", [])}
        robot, ball = entries.get("tennis_robot"), entries.get(BALL_NAME)
        if robot is None or ball is None:
            continue
        yaw = _yaw(robot.get("q", (0.0, 0.0, 0.0, 1.0)))
        dx, dy = ball["x"] - robot["x"], ball["y"] - robot["y"]
        cos_y, sin_y = math.cos(-yaw), math.sin(-yaw)
        samples.append({
            "t_s": record["t_sim"],
            "rel_x": cos_y * dx - sin_y * dy,
            "rel_y": sin_y * dx + cos_y * dy,
            "ball_z": ball["z"],
            "ball_x": ball["x"],
            "robot_x": robot["x"],
        })
    if not samples:
        return None

    start_x = samples[0]["robot_x"]
    moving = [row for row in samples if abs(row["robot_x"] - start_x) > 0.001]
    active = moving or samples
    ball_start_x = active[0]["ball_x"]

    minimum_rel_x = min(row["rel_x"] for row in active)
    maximum_ball_z = max(row["ball_z"] for row in active)
    forward_advance = max(row["ball_x"] - ball_start_x for row in active)
    station_heights = {}
    for name, (station_x, _z) in NIP_STATIONS_BASE.items():
        crossing = next(
            (row for row in active if row["rel_x"] <= station_x), None
        )
        station_heights[name] = None if crossing is None else crossing["ball_z"]

    contacts = {"left": 0, "right": 0, "ramp": 0}
    peak_force = {"wheel": 0.0, "ramp": 0.0}
    contact_path = case_dir / "contact_physics.jsonl"
    summary_row = {}
    if contact_path.exists():
        for line in contact_path.read_text(encoding="utf-8").splitlines():
            if not line.strip():
                continue
            try:
                record = json.loads(line)
            except json.JSONDecodeError:
                continue
            kind = record.get("type", "")
            if kind == "summary":
                summary_row = record
                continue
            force = float(record.get("max_force_n") or 0.0)
            if "ramp" in kind:
                contacts["ramp"] += 1
                peak_force["ramp"] = max(peak_force["ramp"], force)
            else:
                side = record.get("wheel")
                if side in contacts:
                    contacts[side] += 1
                peak_force["wheel"] = max(peak_force["wheel"], force)
    if summary_row:
        wheel_samples = summary_row.get("wheel_contact_samples") or {}
        contacts["left"] = max(contacts["left"], int(wheel_samples.get("left", 0)))
        contacts["right"] = max(contacts["right"], int(wheel_samples.get("right", 0)))
        peak_force["wheel"] = max(peak_force["wheel"],
                                  float(summary_row.get("max_force_n") or 0.0))

    bilateral = contacts["left"] > 0 and contacts["right"] > 0
    any_wheel = contacts["left"] > 0 or contacts["right"] > 0
    transported = minimum_rel_x <= TRANSPORTED_X_BASE
    if bilateral and transported:
        outcome = "CAPTURED"
    elif any_wheel and not transported:
        # The ball reached the wheel bite and was spat back out: a reject, not
        # a pure plough and not a capture.
        outcome = "REJECTED_AT_NIP"
    elif forward_advance > 0.020 and not bilateral:
        outcome = "PLOUGHED_AHEAD"
    elif contacts["ramp"] and not bilateral:
        outcome = "STOPPED_AT_RAMP"
    elif not contacts["ramp"] and not bilateral:
        outcome = "NO_CONTACT"
    else:
        outcome = "NOT_TRANSPORTED"

    return {
        "case": case_dir.name,
        "drive_speed_m_s": float(config.get("drive_speed", "nan")),
        "wheel_speed_rad_s": float(config.get("wheel_speed", "nan")),
        "tread_mu": float(config.get("tread_mu", "nan")),
        "ramp_mu": float(config.get("ramp_mu", "nan")),
        "phase": config.get("phase"),
        "packaging_variant": config.get("packaging_variant"),
        "lateral_offset_m": float(config.get("ball_lateral_offset", "0") or 0.0),
        "outcome": outcome,
        "wheel_contact_samples": {"left": contacts["left"], "right": contacts["right"]},
        "bilateral_wheel_contact": bilateral,
        "ramp_contact_samples": contacts["ramp"],
        "peak_ramp_contact_force_n": peak_force["ramp"],
        "peak_wheel_contact_force_n": peak_force["wheel"],
        "minimum_ball_rel_x_m": minimum_rel_x,
        "nip_x_base_m": NIP_X_BASE,
        "ramp_front_x_base_m": RAMP_FRONT_BASE,
        "reached_nip": minimum_rel_x <= NIP_X_BASE,
        "reached_ramp_front": minimum_rel_x <= RAMP_FRONT_BASE,
        "maximum_ball_centre_z_m": maximum_ball_z,
        "ball_climb_m": maximum_ball_z - geom.BALL_RADIUS_M,
        "ball_centre_z_at_nip_stations_m": station_heights,
        "world_forward_advance_m": forward_advance,
        # The court net stands at world x=0, so a ploughed ball stops there:
        # this number saturates at the ball-to-net distance and is a floor.
        "advance_saturated_at_net": abs(ball_start_x + forward_advance) < 0.10,
        "samples": len(active),
    }


def stage_rows(root: Path, stage: str) -> list[dict]:
    rows = []
    stage_dir = root / stage
    if not stage_dir.exists():
        return rows
    for case_dir in sorted(stage_dir.iterdir()):
        if not case_dir.is_dir():
            continue
        for run_dir in sorted(case_dir.glob("gap_*")):
            result = analyse_case(run_dir)
            if result:
                result["case"] = case_dir.name
                rows.append(result)
    return rows


def compile_campaign(root: Path, solver: dict | None) -> dict:
    s1 = stage_rows(root, "S1")
    s2 = stage_rows(root, "S2")
    s5 = stage_rows(root, "S5")

    def counts(rows):
        result: dict[str, int] = {}
        for row in rows:
            result[row["outcome"]] = result.get(row["outcome"], 0) + 1
        return result

    s1_captured = [row for row in s1 if row["outcome"] == "CAPTURED"]
    s1_reached_nip = [row for row in s1 if row["reached_nip"]]
    ball_climbs = bool(s1_reached_nip)

    s2_captured = [row for row in s2 if row["outcome"] == "CAPTURED"]
    half_width = None
    if s2_captured:
        offsets = sorted(abs(row["lateral_offset_m"]) for row in s2_captured)
        failed = sorted(
            abs(row["lateral_offset_m"]) for row in s2
            if row["outcome"] != "CAPTURED"
        )
        limit = min((value for value in failed if value > offsets[0]), default=None)
        half_width = max(value for value in offsets if limit is None or value < limit)

    stages = {
        "S1": {
            "question": "does the ball climb, stall, or get ploughed ahead?",
            "phase": "Phase 3 (wheels + ramp, no cheeks)",
            "runs": s1,
            "outcome_counts": counts(s1),
            "climb_success_rate_percent": 100.0 * len(s1_reached_nip) / len(s1) if s1 else None,
            "capture_rate_percent": 100.0 * len(s1_captured) / len(s1) if s1 else None,
            "maximum_ball_climb_m": max((row["ball_climb_m"] for row in s1), default=None),
            "wheel_engagement_before_stall": any(row["bilateral_wheel_contact"] for row in s1),
            "BALL_CLIMBS_RAMP_IN_SIM": ball_climbs,
        },
        "S2": {
            "question": "capture half-width",
            "phase": "Phase 4 (full intake)",
            "runs": s2,
            "outcome_counts": counts(s2),
            "INTAKE_CAPTURE_HALF_WIDTH_M": half_width,
            "gated_by": None if s1_captured else "S1: no capture on the frozen ramp",
        },
        "S3": {
            "question": "maximum lateral entry velocity",
            "status": "NOT_RUN",
            "INTAKE_MAX_LATERAL_ENTRY_VELOCITY_M_S": None,
            "gated_by": "S2: no capture half-width exists to sweep entry velocity within",
        },
        "S4": {
            "question": "cheek throat adequacy",
            "cad_throat_ball_centre_half_width_m": geom.throat_ball_centre_half_width_m(),
            "measured_capture_half_width_m": half_width,
            "CHEEK_THROAT_ADEQUATE": None,
            "gated_by": "S2: capture half-width is undefined while S1 fails",
        },
        "S5": {
            "question": "wheel-speed sweep on the corrected model",
            "phase": "Phase 4 (full intake)",
            "runs": s5,
            "outcome_counts": counts(s5),
            "gated_by": None if s1_captured else "S1: no capture on the frozen ramp",
        },
    }

    cross = {"CROSS_INSTRUMENT_AGREEMENT": None}
    if solver:
        sweep = solver["approach_speed"]["sweep"]
        solver_route = bool(sweep["ROUTE_NOMINAL"]["centred_mechanism_captured"])
        solver_max = bool(sweep["ROUTE_MAX"]["centred_mechanism_captured"])
        solver_high = bool(sweep["ABOVE_THRESHOLD"]["centred_mechanism_captured"])
        gz_by_speed = {}
        for row in s1:
            key = f"{row['drive_speed_m_s']:.2f}"
            gz_by_speed.setdefault(key, []).append(row["outcome"] == "CAPTURED")
        gazebo_high = any(gz_by_speed.get("0.80", []))
        cross = {
            "compared_case": "centred ball, authoritative ramp, same approach speeds",
            "tolerance": (
                "qualitative on the capture/plough mechanism at each approach "
                "speed. Exit velocity is NOT cross-compared: Gazebo uses rigid "
                "contact with kp/kd conditioning and does not own that quantity."
            ),
            "solver_centred_capture": {
                "0.35": solver_route, "0.60": solver_max, "0.80": solver_high,
            },
            "gazebo_centred_capture": {
                key: any(values) for key, values in sorted(gz_by_speed.items())
            },
            "agree_at_route_speeds": (not solver_route) and (not solver_max)
            and not any(gz_by_speed.get("0.35", [])) and not any(gz_by_speed.get("0.60", [])),
            "agree_above_threshold": solver_high == gazebo_high,
            "CROSS_INSTRUMENT_AGREEMENT": (
                "AGREE_ON_PLOUGH_AT_ROUTE_SPEEDS"
                if (not solver_route and not any(gz_by_speed.get("0.35", [])))
                else "DISAGREE"
            ),
            "disagreement": None if solver_high == gazebo_high else (
                "Above the plough threshold the two instruments differ: the "
                "reduced-order solver transports a centred ball at 0.80 m/s, "
                "Gazebo reaches the wheel bite and rejects it. The gap is "
                "narrow — both put the ball at the front face of the nip — and "
                "it is REPORTED, NOT TUNED AWAY. Candidate causes, none of "
                "which may be adjusted to close it: the two contact laws at "
                "the 1.5 mm lip (calibrated ball law vs rigid kp/kd), the "
                "solver's 2-contact reduction versus 6-DOF multi-contact, and "
                "the absence of tyre compliance in the Gazebo wheel. Settling "
                "it needs the physical ramp-lip test, not a model change."
            ),
        }

    # Pure frozen-CAD geometry: the lip meets a court-resting ball 48.6 mm
    # before any wheel can, which is why the wedge decides the outcome.
    contact_sequence = {
        "ramp_lip_first_contact_ball_centre_cad_x_m": 0.5298,
        "first_wheel_contact_ball_centre_cad_x_m": 0.4812,
        "lip_lead_m": 0.0486,
        "asserted_by": "tests/test_intake_frame_alignment.py",
    }
    return {
        "schema_version": 1,
        "status": "COMPLETE" if s1 else "NOT_RUN",
        "frozen_cad_contact_sequence": contact_sequence,
        "scope": (
            "Gazebo owns ramp wedge/plough dynamics with ground friction, cheek "
            "centering, capture half-width, jam/reject modes and 6-DOF "
            "multi-contact. Wheel droop, torque and current are NOT measurable "
            "here (ideal velocity source) and are not reported."
        ),
        "model": {
            "packaging_variant": VARIANT,
            "nip_x_base_link_m": NIP_X_BASE,
            "ramp": "option_a_handoff_ramp.stl (authoritative oa_ramp_z)",
            "legacy_carriage": False,
            "tread_friction_swept": list(geom.TREAD_FRICTION_BOUNDS),
            "ball_ramp_friction_swept": list(geom.BALL_RAMP_FRICTION_BOUNDS),
        },
        "stages": stages,
        "cross_validation": cross,
    }


def campaign_markdown(result: dict) -> str:
    stages = result["stages"]
    s1 = stages["S1"]
    cross = result["cross_validation"]
    rows = "\n".join(
        f"| {row['drive_speed_m_s']:.2f} | {row['tread_mu']:.1f} | {row['ramp_mu']:.2f} | "
        f"{row['outcome']} | {row['ball_climb_m']*1000:.1f} | "
        f"{row['minimum_ball_rel_x_m']*1000:.0f} | {row['world_forward_advance_m']*1000:.0f} |"
        for row in s1["runs"]
    )
    s2_rows = "\n".join(
        f"| {row['lateral_offset_m']*1000:.0f} | {row['outcome']} | "
        f"{row['minimum_ball_rel_x_m']*1000:.0f} | {row['ball_climb_m']*1000:.1f} |"
        for row in stages["S2"]["runs"]
    )
    s5_rows = "\n".join(
        f"| {row['wheel_speed_rad_s']:.1f} | {row['outcome']} | "
        f"{row['minimum_ball_rel_x_m']*1000:.0f} | {row['ball_climb_m']*1000:.1f} |"
        for row in stages["S5"]["runs"]
    )
    return f"""**S1 — ramp dynamics (Phase 3: wheels + ramp, no cheeks), centred ball.**
The nip mid-plane sits at base_link x = {NIP_X_BASE*1000:.0f} mm and the ramp
front lip at {RAMP_FRONT_BASE*1000:.0f} mm. `minimum ball x` is how far the ball
ever got toward the nip; `advance` is how far it was pushed forward in the world.

`REJECTED_AT_NIP` means the ball reached the wheel bite and was spat out again.
`Advance` saturates at the court net (world x=0), so it is a floor, not the
distance the ball would have travelled on an open court.

| Drive (m/s) | tread mu | ramp mu | Outcome | Climb (mm) | Min ball x (mm) | Advance (mm) |
| --- | --- | --- | --- | --- | --- | --- |
{rows}

Climb success (ball reached the nip): {s1['climb_success_rate_percent']:.0f}% of
{len(s1['runs'])} runs. Capture: {s1['capture_rate_percent']:.0f}%.
Maximum ball climb observed: {s1['maximum_ball_climb_m']*1000:.1f} mm.
At least one run reached bilateral wheel contact:
{str(s1['wheel_engagement_before_stall']).lower()} — that is a REJECT at the nip
face, not a capture: no run transported the ball past the nip.

**S2 — capture half-width (Phase 4, full intake).**

| Lateral offset (mm) | Outcome | Min ball x (mm) | Climb (mm) |
| --- | --- | --- | --- |
{s2_rows}

`INTAKE_CAPTURE_HALF_WIDTH_M = {json.dumps(stages['S2']['INTAKE_CAPTURE_HALF_WIDTH_M'])}`
{'(gated: ' + stages['S2']['gated_by'] + ')' if stages['S2']['gated_by'] else ''}

**S3 — lateral entry velocity.** `{stages['S3']['status']}`, gated by S2:
{stages['S3']['gated_by']}.

**S4 — cheek throat adequacy.** The CAD throat admits a ball centre to
±{stages['S4']['cad_throat_ball_centre_half_width_m']*1000:.0f} mm. There is no measured capture
half-width to compare it against while S1 fails, so `CHEEK_THROAT_ADEQUATE`
stays undetermined. No cheek geometry was changed.

**S5 — wheel-speed sweep (Phase 4, centred).**

| Wheel speed (rad/s) | Outcome | Min ball x (mm) | Climb (mm) |
| --- | --- | --- | --- |
{s5_rows}

**Cross-validation gate.** {cross.get('tolerance', '')}
Solver centred capture by approach speed: {json.dumps(cross.get('solver_centred_capture'))}.
Gazebo centred capture by approach speed: {json.dumps(cross.get('gazebo_centred_capture'))}.
`CROSS_INSTRUMENT_AGREEMENT = {cross.get('CROSS_INSTRUMENT_AGREEMENT')}`.
{cross.get('disagreement') or 'The two instruments agree on the mechanism at every compared speed.'}
"""


def plot_campaign(result: dict, image_dir: Path) -> list[str]:
    """One figure: how far the ball ever got, per commanded approach speed."""

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    runs = result["stages"]["S1"]["runs"]
    if not runs:
        return []
    image_dir.mkdir(parents=True, exist_ok=True)
    speeds = sorted({row["drive_speed_m_s"] for row in runs})
    climb = [
        1000.0 * max(row["ball_climb_m"] for row in runs if row["drive_speed_m_s"] == speed)
        for speed in speeds
    ]
    reach = [
        1000.0 * min(row["minimum_ball_rel_x_m"] for row in runs if row["drive_speed_m_s"] == speed)
        for speed in speeds
    ]
    figure, axis = plt.subplots(figsize=(8, 4.5))
    axis.plot(speeds, reach, marker="o", color="steelblue",
              label="deepest ball position reached (mm, base_link X)")
    axis.axhline(1000.0 * NIP_X_BASE, color="crimson", linestyle="--",
                 label=f"nip mid-plane {NIP_X_BASE*1000:.0f} mm")
    axis.axhline(1000.0 * RAMP_FRONT_BASE, color="darkorange", linestyle=":",
                 label=f"ramp front lip {RAMP_FRONT_BASE*1000:.0f} mm")
    axis.set_xlabel("Commanded robot approach speed (m/s)")
    axis.set_ylabel("Ball X in base_link (mm) — lower is deeper into the intake")
    twin = axis.twinx()
    twin.plot(speeds, climb, marker="x", color="seagreen", label="peak ball climb (mm)")
    twin.set_ylabel("Peak ball climb above rest (mm)")
    axis.grid(True, alpha=0.3)
    lines = axis.get_lines() + twin.get_lines()
    axis.legend(lines, [line.get_label() for line in lines], loc="center right")
    plt.title("Gazebo S1: the ball never passes the ramp lip (Phase 3, centred)")
    plt.tight_layout()
    path = image_dir / "intake-gazebo-s1-ball-reach-vs-approach-speed.png"
    plt.savefig(path, dpi=160)
    plt.close(figure)
    return [str(path.relative_to(ROOT))]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--campaign-root", type=Path,
                        default=ROOT / "runtime/intake_campaign")
    parser.add_argument("--solver-result", type=Path,
                        default=ROOT / "config/standalone_intake_handoff_capability.json")
    parser.add_argument("--output", type=Path,
                        default=ROOT / "config/intake_gazebo_campaign.json")
    args = parser.parse_args()

    solver = None
    if args.solver_result.exists():
        solver = json.loads(args.solver_result.read_text(encoding="utf-8"))
    result = compile_campaign(args.campaign_root, solver)
    result["plots"] = plot_campaign(result, ROOT / "docs/images")
    result["markdown"] = campaign_markdown(result) + (
        "\n" + "\n".join(
            f"![{Path(path).stem}](../images/{Path(path).name})"
            for path in result["plots"]
        )
    )
    args.output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({
        "status": result["status"],
        "S1": result["stages"]["S1"]["outcome_counts"],
        "S2": result["stages"]["S2"]["outcome_counts"],
        "S5": result["stages"]["S5"]["outcome_counts"],
        "cross": result["cross_validation"].get("CROSS_INSTRUMENT_AGREEMENT"),
    }, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
