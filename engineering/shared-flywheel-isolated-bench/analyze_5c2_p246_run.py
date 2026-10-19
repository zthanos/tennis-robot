#!/usr/bin/env python3
"""Analyze one P2-46 delivery or motion-retention run."""
from __future__ import annotations

import argparse
import csv
import json
import math
from collections import defaultdict
from pathlib import Path


def read(path: Path) -> list[dict[str, str]]:
    with path.open(newline="") as stream:
        return [row for row in csv.DictReader(stream) if None not in row.values()]


def broad_contained(row: dict[str, str]) -> bool:
    return (-0.048 <= float(row["base_x_m"]) <= 0.311
            and abs(float(row["base_y_m"])) <= 0.207
            and 0.025 <= float(row["base_z_m"]) <= 0.32)


def settled_inside(row: dict[str, str], variant: str = "p246_open") -> bool:
    z_max = 0.315 if variant == "tall_open_drop" else 0.205
    return (0.018 <= float(row["base_x_m"]) <= 0.245
            and abs(float(row["base_y_m"])) <= 0.1405
            and 0.075 <= float(row["base_z_m"]) <= z_max)


def escaped(row: dict[str, str]) -> bool:
    return (float(row["base_x_m"]) < -0.048 or float(row["base_x_m"]) > 0.311
            or abs(float(row["base_y_m"])) > 0.207 or float(row["base_z_m"]) < 0.0)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=("delivery", "motion"), required=True)
    ap.add_argument("--observer", type=Path, required=True)
    ap.add_argument("--preload", type=int, required=True)
    ap.add_argument("--criteria", type=Path, required=True)
    ap.add_argument("--output", type=Path, required=True)
    ap.add_argument("--telemetry", type=Path)
    ap.add_argument("--ball-contacts", type=Path)
    ap.add_argument("--native-contacts", type=Path)
    ap.add_argument("--drive-telemetry", type=Path)
    ap.add_argument("--basket-variant", default="p246_open")
    args = ap.parse_args()
    criteria = json.loads(args.criteria.read_text())
    observer = read(args.observer)
    if not observer: raise SystemExit("empty observer telemetry")
    nonfinite = sum(not math.isfinite(float(value)) for row in observer for key, value in row.items() if key not in ("ball_name",) and value)
    last_t = max(float(row["time_s"]) for row in observer)
    by_name: dict[str, list[dict[str, str]]] = defaultdict(list)
    for row in observer: by_name[row["ball_name"]].append(row)

    if args.mode == "motion":
        drive = read(args.drive_telemetry)
        nonfinite += sum(not math.isfinite(float(value)) for row in drive for key, value in row.items() if key not in ("scenario", "case_name") and value)
        baseline = {name for name, rows in by_name.items() if any(5.0 <= float(row["time_s"]) <= 6.0 and broad_contained(row) for row in rows)}
        lost = {name for name, rows in by_name.items() if any(float(row["time_s"]) >= 6.0 and escaped(row) for row in rows)}
        final = {name for name, rows in by_name.items() if any(last_t-0.5 <= float(row["time_s"]) and broad_contained(row) for row in rows)}
        metrics = {"preload_requested": args.preload, "baseline_retained_count": len(baseline),
                   "motion_ball_loss_count": len(lost), "motion_lost_names": sorted(lost),
                   "final_retained_count": len(final), "nonfinite_count": nonfinite,
                   "max_abs_roll_deg": max(abs(math.degrees(float(row["roll_rad"]))) for row in drive),
                   "max_abs_pitch_deg": max(abs(math.degrees(float(row["pitch_rad"]))) for row in drive)}
        checks = {"initial_full_basket_retained": len(baseline) == args.preload,
                  "motion_ball_loss_count": not lost, "final_full_basket_retained": len(final) == args.preload,
                  "finite_state": nonfinite == 0}
        result = {"pass": all(checks.values()), "mode": args.mode, "checks": checks, "metrics": metrics,
                  "criteria_status": criteria["status"]}
    else:
        telemetry, ball_contacts, native = read(args.telemetry), read(args.ball_contacts), read(args.native_contacts)
        nonfinite += sum(not math.isfinite(float(value)) for row in telemetry for value in row.values() if value)
        active = [row for row in telemetry if float(row["time_s"]) >= 6.0]
        if args.basket_variant == "tall_open_drop":
            entry_x, entry_z_min, entry_z_max = 0.085, 0.205, 0.315
        elif args.basket_variant == "high_hood":
            entry_x, entry_z_min, entry_z_max = 0.278, 0.090, 0.190
        else:
            entry_x, entry_z_min, entry_z_max = 0.278, 0.075, 0.165
        entry = next((row for row in active if float(row["ball_base_x_m"]) <= entry_x
                      and abs(float(row["ball_base_y_m"])) <= 0.015
                      and entry_z_min <= float(row["ball_base_z_m"]) <= entry_z_max), None)
        nip = next((row for row in active if float(row["ball_launcher_x_m"]) <= -0.033), None)
        capture = by_name.get("capture_ball_link", [])
        entry_t = float(entry["time_s"]) if entry else None
        final_capture = [row for row in capture if float(row["time_s"]) >= last_t-1.0]
        retained = bool(final_capture) and sum(settled_inside(row, args.basket_variant) for row in final_capture) / len(final_capture) >= 0.8
        bounced_out = bool(entry_t is not None and any(float(row["time_s"]) >= entry_t and escaped(row) for row in capture))
        final_rows = [row for row in observer if float(row["time_s"]) >= last_t-1.0]
        times: dict[float, list[dict[str, str]]] = defaultdict(list)
        for row in final_rows: times[float(row["time_s"])].append(row)
        if args.basket_variant == "tall_open_drop":
            occupied = lambda row: (0.015 <= float(row["base_x_m"]) <= 0.090
                                    and abs(float(row["base_y_m"])) <= 0.055
                                    and 0.205 <= float(row["base_z_m"]) <= 0.315)
        else:
            occupied = lambda row: (0.235 <= float(row["base_x_m"]) <= 0.311
                                    and abs(float(row["base_y_m"])) <= 0.050
                                    and 0.050 <= float(row["base_z_m"]) <= 0.160)
        occupied_samples = sum(any(occupied(row) for row in rows)
                               for rows in times.values())
        blocked_fraction = occupied_samples / len(times) if times else 1.0
        entrance_blocked = blocked_fraction >= 0.8
        preload_names = {name for name in by_name if name.startswith("basket_ball_")}
        predrive_retained = sum(any(5.0 <= float(row["time_s"]) <= 5.8 and broad_contained(row) for row in by_name[name]) for name in preload_names)
        final_preload = sum(any(float(row["time_s"]) >= last_t-0.5 and broad_contained(row) for row in by_name[name]) for name in preload_names)
        initial_x, initial_y = float(telemetry[0]["ball_x_m"]), float(telemetry[0]["ball_y_m"])
        pre_nip = active if nip is None else [row for row in active if float(row["time_s"]) <= float(nip["time_s"])]
        forward_push = max(float(row["ball_x_m"])-initial_x for row in pre_nip)
        lateral_push = max(abs(float(row["ball_y_m"])-initial_y) for row in pre_nip)
        wheel_times = [float(row["time_s"]) for row in ball_contacts if int(float(row["contact_id"])) in (0,1) and float(row["normal_force_n"]) > 0.01]
        contact_rows = [] if not wheel_times else [row for row in telemetry if min(wheel_times) <= float(row["time_s"]) <= max(wheel_times)]
        def current_stats(rows: list[dict[str,str]]) -> dict:
            values = [abs(float(row[key])) for row in rows for key in ("left_current_a", "right_current_a")]
            return {"max_abs_a": max(values) if values else None, "mean_abs_a": sum(values)/len(values) if values else None,
                    "saturated_sample_count": sum(value >= 19.999 for value in values)}
        launcher = ("cad_upper_plate", "cad_yoke_", "cad_trunnion_", "d5065_motor_", "flywheel_left", "flywheel_right", "intake_cheek_")
        fixed = ("intake_fixed_ramp_", "fixture_arch_", "fixture_lidar_", "chassis_", "ground_plane")
        forbidden = [row for row in native if ((any(token in (row.get("collision_1") or "") for token in launcher) and any(token in (row.get("collision_2") or "") for token in fixed)) or (any(token in (row.get("collision_2") or "") for token in launcher) and any(token in (row.get("collision_1") or "") for token in fixed)))]
        path_contacts = sum(any(token in ((row.get("collision_1") or "")+(row.get("collision_2") or "")) for token in ("intake_cheek_", "basket_front_")) for row in native)
        max_current = max(abs(float(row[key])) for row in telemetry for key in ("left_current_a", "right_current_a"))
        collection_success = entry is not None and retained and not bounced_out
        checks = {"collection_success": collection_success, "bounce_out": not bounced_out,
                  "entrance_blocked": not entrance_blocked, "preload_retained_before_delivery": predrive_retained == args.preload,
                  "preload_retained_final": final_preload == args.preload,
                  "d5065_current": max_current <= 20.0+1e-9, "forbidden_launcher_contact_count": not forbidden,
                  "finite_state": nonfinite == 0}
        metrics = {"basket_variant": args.basket_variant, "preload_requested": args.preload, "preload_retained_before_delivery": predrive_retained,
                   "preload_retained_final": final_preload, "basket_entry_time_s": entry_t,
                   "nip_crossing_time_s": float(nip["time_s"]) if nip else None,
                   "new_ball_retained": retained, "bounce_out": bounced_out,
                   "entrance_blocked": entrance_blocked, "entrance_occupied_fraction_final_1s": blocked_fraction,
                   "pre_nip_forward_displacement_m": forward_push, "pre_nip_lateral_displacement_m": lateral_push,
                   "front_wall_or_cheek_contact_count": path_contacts, "spin_up_current": current_stats([row for row in telemetry if float(row["time_s"]) <= 2.0]),
                   "ball_contact_current": current_stats(contact_rows), "ball_contact_duration_s": max(wheel_times)-min(wheel_times) if wheel_times else None,
                   "max_current_a_whole_run": max_current, "forbidden_contact_count": len(forbidden), "nonfinite_count": nonfinite}
        result = {"pass": all(checks.values()), "mode": args.mode, "checks": checks, "metrics": metrics,
                  "criteria_status": criteria["status"], "forbidden_contact_examples": forbidden[:10]}
    args.output.write_text(json.dumps(result, indent=2)+"\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__": main()
