#!/usr/bin/env python3
"""Aggregate the complete 3x3x2 turn-only diagnostic campaign."""
from __future__ import annotations
import csv, json
from pathlib import Path

ROOT=Path("engineering/chassis-strength-bench/results/shared_flywheel_phase2/gazebo_5a_turn_diagnostic")
rows=[]; runs=[]
for variant in ("open_yaw","gyro_yaw"):
  for case in ("empty","full45","ballast"):
    for mu in ("0p5","0p7","0p9"):
      path=ROOT/variant/case/f"mu_{mu}"/"summary.json"; d=json.loads(path.read_text()); runs.append(d)
      for scenario,q in d["scenarios"].items():
        load=d["loads"][scenario]; screen=d["coulomb_screen"]
        rows.append({"variant":variant,"case":case,"mu":d["mu"],"scenario":scenario,"pass":q["pass"],
          "yaw_error":q["yaw_relative_error"],"linear_error":q["linear_relative_error"],"centre_translation_m":q["centre_translation_m"],
          "mean_slip":q["mean_slip"],"peak_slip":q["peak_slip"],"max_command_nm":q["max_command_nm"],
          "motor_saturation_fraction":q["motor_saturation_fraction"],"tyre_friction_limit_exceedance_fraction":q["tyre_friction_limit_exceedance_fraction"],
          "icr_body_x_median_m":q["instantaneous_icr_body_median_m"][0],"icr_body_y_median_m":q["instantaneous_icr_body_median_m"][1],
          "expected_icr_world_drift_m":q["expected_load_centre_icr_world_drift_m"],"geometric_base_chord_m":q["base_origin_geometric_chord_if_expected_icr_fixed_m"],
          "geometric_base_chord_about_actual_icr_m":q["base_origin_geometric_chord_about_median_actual_icr_m"],
          "instantaneous_icr_world_p95_spread_m":q["instantaneous_icr_world_p95_spread_m"],
          "instantaneous_icr_world_max_spread_m":q["instantaneous_icr_world_max_spread_m"],"icr_x_offset_from_load_centre_m":q["instantaneous_icr_x_offset_from_load_centre_m"],
          "coulomb_required_nm":screen["required_scrub_yaw_nm"],"coulomb_available_nm":min(screen["available_traction_yaw_nm"],screen["available_motor_yaw_nm"]),
          "coulomb_ratio":screen["available_to_required_ratio"],"tilt_peak_torque_nm":load["tilt_peak_torque_nm"],
          "tilt_actuator_peak_command_nm":load["tilt_actuator_peak_command_nm"],"tilt_position_mean_deg":load["tilt_position_mean_deg"]})
with (ROOT/"scenario_results.csv").open("w",newline="") as f:
  w=csv.DictWriter(f,fieldnames=list(rows[0])); w.writeheader(); w.writerows(rows)
summary={"run_count":len(runs),"scenario_count":len(rows),"overall_scenario_pass_count":sum(bool(r["pass"]) for r in rows),
 "by_variant":{},"global":{
  "coulomb_ratio_range":[min(r["coulomb_ratio"] for r in rows),max(r["coulomb_ratio"] for r in rows)],
  "max_hinge_torque_nm":max(r["tilt_peak_torque_nm"] for r in rows),"max_actuator_hold_command_nm":max(r["tilt_actuator_peak_command_nm"] for r in rows),
  "max_motor_command_nm":max(r["max_command_nm"] for r in rows),"max_motor_saturation_fraction":max(r["motor_saturation_fraction"] for r in rows)}}
for v in ("open_yaw","gyro_yaw"):
  q=[r for r in rows if r["variant"]==v]
  summary["by_variant"][v]={"scenario_pass_count":sum(bool(r["pass"]) for r in q),"scenario_count":len(q),
    "yaw_error_range":[min(r["yaw_error"] for r in q),max(r["yaw_error"] for r in q)],
    "spin_translation_range_m":[min(r["centre_translation_m"] for r in q if r["scenario"].startswith("spin")),max(r["centre_translation_m"] for r in q if r["scenario"].startswith("spin"))],
    "mean_slip_range":[min(r["mean_slip"] for r in q),max(r["mean_slip"] for r in q)]}
(ROOT/"campaign_summary.json").write_text(json.dumps(summary,indent=2)+"\n")
print(json.dumps(summary,indent=2))
