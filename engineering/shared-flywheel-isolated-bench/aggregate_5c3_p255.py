#!/usr/bin/env python3
from __future__ import annotations
import json
from pathlib import Path

root = Path("engineering/chassis-strength-bench/results/shared_flywheel_phase2/gazebo_5c3_p255")
case_names = ("series_gap_0", "series_gap_50", "series_gap_150", "series_gap_300",
              "side_by_side_y50", "side_by_side_y100")
matrix = []
for mu in ("0p6", "0p9"):
    for case in case_names:
        path = root / "matrix" / f"tm_{mu}" / case / "summary.json"
        if path.exists(): matrix.append(json.loads(path.read_text()))
diagnostics = []
for profile in ("stop_1s", "reverse_75mm"):
    for mu in ("0p6", "0p9"):
        path = root / "diagnostics" / profile / f"tm_{mu}" / "side_by_side_y100" / "summary.json"
        if path.exists(): diagnostics.append(json.loads(path.read_text()))

def replay_for(row: dict) -> Path:
    mu = str(row["tread_mu"]).replace(".", "p")
    if row["drive_profile"] == "continuous":
        return root / "matrix" / f"tm_{mu}" / row["case"] / "replay" / "state.tlog"
    return root / "diagnostics" / row["drive_profile"] / f"tm_{mu}" / row["case"] / "replay" / "state.tlog"

required_record_rows = ([row for row in matrix
                         if row["case"] in ("series_gap_0", "series_gap_50") or not row["pass"]]
                        + diagnostics)
second_speeds = [row["metrics"]["second_ball_exit_speed_m_s"] for row in matrix
                 if row["metrics"]["second_ball_exit_speed_m_s"] is not None]
replay_commands = []
for row in required_record_rows:
    args = f'{row["case"]} {row["tread_mu"]}'
    if row["drive_profile"] != "continuous":
        args += f' {row["drive_profile"]}'
    replay_commands.append(f"engineering/shared-flywheel-isolated-bench/replay_5c3_case.sh {args}")

matrix_metrics = []
for row in matrix:
    balls = list(row["metrics"]["per_ball"].values())
    matrix_metrics.append({
        "case": row["case"],
        "tread_mu": row["tread_mu"],
        "pass": row["pass"],
        "rpm_drop_per_ball": [ball["flywheel_response"]["maximum_rpm_drop"] for ball in balls],
        "recovery_to_95pct_s_per_ball": [ball["flywheel_response"]["recovery_to_95pct_s"] for ball in balls],
        "exit_speed_m_s_per_ball": [ball["takeoff_speed_m_s"] for ball in balls],
        "second_ball_exit_speed_m_s": row["metrics"]["second_ball_exit_speed_m_s"],
    })
report = {
    "status": "COMPLETE" if len(matrix) == 12 and len(diagnostics) == 4 else "INCOMPLETE",
    "headless_acceptance_only": True,
    "expected_matrix_conditions": 12,
    "completed_matrix_conditions": len(matrix),
    "passes": sum(row["pass"] for row in matrix),
    "failures": [{"case": row["case"], "tread_mu": row["tread_mu"]} for row in matrix if not row["pass"]],
    "minimum_second_ball_exit_speed_m_s": min(second_speeds) if second_speeds else None,
    "required_record_count": len(required_record_rows),
    "all_required_replays_recorded": all(replay_for(row).exists() for row in required_record_rows),
    "exact_replay_commands": replay_commands,
    "throat_assessment": {
        "governing_nip_free_width_mm": 58.0,
        "two_ball_reference_width_mm": 134.0,
        "width_deficit_mm": 76.0,
        "geometry_changed": False,
    },
    "matrix_metric_digest": matrix_metrics,
    "matrix": matrix,
    "jam_recovery_diagnostics": diagnostics,
    "jam_recovery_resolved": [{"profile": row["drive_profile"], "tread_mu": row["tread_mu"],
                                "resolved": row["pass"],
                                "actual_reverse_displacement_mm": row["metrics"]["actual_reverse_displacement_mm"]}
                               for row in diagnostics],
}
(root / "summary.json").write_text(json.dumps(report, indent=2) + "\n")
print(json.dumps({k: report[k] for k in ("status", "completed_matrix_conditions", "passes", "failures", "minimum_second_ball_exit_speed_m_s", "all_required_replays_recorded", "jam_recovery_resolved")}, indent=2))
