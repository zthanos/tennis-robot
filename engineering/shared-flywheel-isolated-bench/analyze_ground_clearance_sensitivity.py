#!/usr/bin/env python3
"""Aggregate the CAD-aligned angle sweep and R/h mounting sensitivity."""
from __future__ import annotations
import argparse, csv, json, math
from pathlib import Path

def main():
  p=argparse.ArgumentParser(); p.add_argument("sweep_dir",type=Path); p.add_argument("--output-json",type=Path,required=True); p.add_argument("--output-csv",type=Path,required=True); p.add_argument("--dynamic-allowance-mm",type=float,default=5.0); a=p.parse_args()
  sweeps=[json.loads(x.read_text()) for x in sorted(a.sweep_dir.glob("angle_*.json"))]
  # Sweep JSON is ground-normalized by the drive tyre.  It represents the
  # reference point R=85 mm, h=29 mm; R remains unmeasured, not nominal.
  nominal_other=min(x["minimum_nonwheel_nonflywheel_z_m"] for x in sweeps)*1000
  nominal_fly=min(x["minimum_flywheel_z_m"] for x in sweeps)*1000
  other_at=min(sweeps,key=lambda x:x["minimum_nonwheel_nonflywheel_z_m"])["launcher_angle_deg"]
  fly_at=min(sweeps,key=lambda x:x["minimum_flywheel_z_m"])["launcher_angle_deg"]
  # Intake contact-band audit.  The Ø200x50 wheels have their 50 mm shaft-axis
  # width along local W.  At theta=-10 deg its lower axial edge is
  # Znip-(width/2)*cos(theta).  The requested 8 mm inset treats the rounded
  # tyre edge as unusable; it is not a fitted tyre-profile model.
  intake_deg=-10.0; intake=math.radians(intake_deg)
  pivot_z_chassis=64.3; nip_u=45.0; wheel_width=50.0; wheel_radius=100.0
  ball_center_ground_z=33.0; edge_margin=8.0
  target_lower_edge=ball_center_ground_z-edge_margin
  phase=math.atan2(wheel_width/2.0,nip_u)
  amplitude=math.hypot(nip_u,wheel_width/2.0)
  rows=[]
  for radius in (82.0,85.0,90.0):
    for axle in (26.0,29.0,33.0):
      shift=(radius-85.0)-(axle-29.0)
      rail=radius-axle-30.0
      other=nominal_other+shift; fly=nominal_fly+shift
      pivot_ground=pivot_z_chassis+(radius-axle-52.0)
      nip_height=pivot_ground+nip_u*math.sin(intake)
      lower_edge=nip_height-(wheel_width/2.0)*math.cos(intake)
      ball_from_lower=ball_center_ground_z-lower_edge
      required_down=max(0.0,edge_margin-ball_from_lower)
      q=(target_lower_edge-pivot_ground)/amplitude
      angle_feasible=-1.0 <= q <= 1.0
      required_angle=math.degrees(phase+math.asin(q)) if angle_feasible else None
      angle_clearance=(target_lower_edge-wheel_radius*abs(math.sin(math.radians(required_angle)))) if angle_feasible else None
      rows.append({"drive_radius_mm":radius,"axle_above_foot_mm":axle,"vertical_shift_from_nominal_mm":shift,"rail_underside_mm":rail,"minimum_other_raw_mm":other,"minimum_other_after_dynamic_allowance_mm":other-a.dynamic_allowance_mm,"minimum_flywheel_raw_mm":fly,"minimum_flywheel_after_dynamic_allowance_mm":fly-a.dynamic_allowance_mm,"other_meets_12mm_after_allowance":other-a.dynamic_allowance_mm>=12.0,"intake_nip_height_ground_mm":nip_height,"contact_band_lower_edge_ground_mm":lower_edge,"ground_ball_center_from_lower_edge_mm":ball_from_lower,"ground_ball_meets_8mm_edge_margin":ball_from_lower>=edge_margin,"required_pivot_down_from_chassis_mm":required_down,"resulting_pivot_z_chassis_mm":pivot_z_chassis-required_down,"flywheel_raw_clearance_after_pivot_adjustment_mm":fly-required_down,"required_intake_angle_deg_if_pivot_unchanged":required_angle,"required_intake_angle_change_deg":required_angle-intake_deg if angle_feasible else None,"flywheel_raw_clearance_at_required_angle_mm":angle_clearance,"angle_solution_avoids_ground_intersection":angle_clearance is not None and angle_clearance>=0.0})
  def combined_adjustment(raw_clearance_target):
    # Once the lower band edge is fixed at Z=25 mm, wheel clearance is
    # 25-100*abs(sin(theta)); choose the shallowest admissible negative angle.
    angle=-math.degrees(math.asin((target_lower_edge-raw_clearance_target)/wheel_radius))
    term=nip_u*math.sin(math.radians(angle))-(wheel_width/2.0)*math.cos(math.radians(angle))
    downs=[max(0.0,(pivot_z_chassis+(r-h-52.0)+term)-target_lower_edge) for r in (82.0,85.0,90.0) for h in (26.0,29.0,33.0)]
    return {"intake_angle_deg":angle,"pivot_down_range_mm":[min(downs),max(downs)],"reference_r85_h29_pivot_down_mm":downs[4],"resulting_flywheel_raw_clearance_mm":raw_clearance_target}
  adjustment={"all_grid_cases_pass":all(r["ground_ball_meets_8mm_edge_margin"] for r in rows),"nip_height_ground_range_mm":[min(r["intake_nip_height_ground_mm"] for r in rows),max(r["intake_nip_height_ground_mm"] for r in rows)],"ball_center_from_lower_edge_range_mm":[min(r["ground_ball_center_from_lower_edge_mm"] for r in rows),max(r["ground_ball_center_from_lower_edge_mm"] for r in rows)],"pure_pivot_down_required_range_mm":[min(r["required_pivot_down_from_chassis_mm"] for r in rows),max(r["required_pivot_down_from_chassis_mm"] for r in rows)],"pure_pivot_resulting_flywheel_raw_clearance_mm":min(r["flywheel_raw_clearance_after_pivot_adjustment_mm"] for r in rows),"pure_angle_required_range_deg":[min(r["required_intake_angle_deg_if_pivot_unchanged"] for r in rows),max(r["required_intake_angle_deg_if_pivot_unchanged"] for r in rows)],"pure_angle_solution_avoids_ground_intersection":all(r["angle_solution_avoids_ground_intersection"] for r in rows),"combined_for_12mm_raw_flywheel_clearance":combined_adjustment(12.0),"combined_for_12mm_after_5mm_allowance":combined_adjustment(17.0)}
  result={"angle_sweep_deg":[min(x["launcher_angle_deg"] for x in sweeps),max(x["launcher_angle_deg"] for x in sweeps)],"angle_step_deg":2,"reference_case_not_radius_release":{"drive_radius_mm":85,"axle_above_foot_mm":29,"minimum_other_raw_mm":nominal_other,"minimum_other_angle_deg":other_at,"minimum_flywheel_raw_mm":nominal_fly,"minimum_flywheel_angle_deg":fly_at,"rail_underside_mm":26,"dynamic_pitch_and_uneven_ground_allowance_mm":a.dynamic_allowance_mm,"minimum_other_after_allowance_mm":nominal_other-a.dynamic_allowance_mm,"meets_12mm_other_requirement":nominal_other-a.dynamic_allowance_mm>=12},"intake_contact_band_contract":{"intake_angle_deg":intake_deg,"nip_local_u_mm":nip_u,"flywheel_diameter_mm":2*wheel_radius,"flywheel_axial_width_mm":wheel_width,"ground_ball_center_z_mm":ball_center_ground_z,"rounded_edge_unusable_inset_mm":edge_margin,"criterion":"ball centre must be at least 8 mm above the projected lower axial edge; 8 mm is a conservative edge-rounding reserve, not a measured profile","lower_edge_formula":"Znip - 25*cos(theta)","warning":"pivot-Z-only compliance leaves 7.635 mm raw flywheel-ground clearance at -10 deg; angle-only compliance intersects the ground throughout this R/h grid"},"adjustment_envelope":adjustment,"sensitivity_assumption":"all chassis-attached Z clearances shift by (R-85)-(h-29) mm after tyre settling; tyre loaded radius and foam-insert deformation are not measured","withdrawn_h_estimate_mm":[21,25],"h_measurement_mm":{"nominal":29,"tape_uncertainty":3,"sweep":[26,33]},"loaded_radius_unmeasured_sweep_mm":[82,90],"rows":rows}
  a.output_json.write_text(json.dumps(result,indent=2)+"\n")
  with a.output_csv.open("w",newline="") as f: w=csv.DictWriter(f,fieldnames=list(rows[0])); w.writeheader(); w.writerows(rows)
  print(json.dumps(result,indent=2))
if __name__=="__main__":main()
