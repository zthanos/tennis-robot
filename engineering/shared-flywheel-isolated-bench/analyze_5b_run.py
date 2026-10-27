#!/usr/bin/env python3
"""Analyze one P2-42 Gazebo 5b telemetry/contact pair."""
from __future__ import annotations
import argparse, csv, json, math
from pathlib import Path

def rows(path: Path) -> list[dict[str,str]]:
    with path.open(newline="") as f: return list(csv.DictReader(f))

def main() -> None:
    ap=argparse.ArgumentParser(); ap.add_argument("telemetry",type=Path); ap.add_argument("contacts",type=Path)
    ap.add_argument("--criteria",type=Path,required=True); ap.add_argument("--case",required=True,choices=("empty","full45","ballast")); ap.add_argument("--output",type=Path,required=True)
    a=ap.parse_args(); c=json.loads(a.criteria.read_text()); data=rows(a.telemetry); contact=rows(a.contacts)
    if not data: raise SystemExit("empty telemetry")
    nums=("time_s","command_rad","measured_rad","velocity_rad_s","command_nm","base_x_m","base_y_m","base_z_m","base_roll_rad","base_pitch_rad","base_yaw_rad","module_x_m","module_y_m","module_z_m","module_roll_rad","module_pitch_rad","module_yaw_rad")
    finite_errors=[]
    for i,r in enumerate(data):
        for k in nums:
            try: value=float(r[k])
            except (ValueError,KeyError): finite_errors.append(f"row {i} {k} parse"); continue
            if not math.isfinite(value): finite_errors.append(f"row {i} {k} nonfinite")
    deg=180/math.pi
    dwell=[r for r in data if r["phase"].startswith("dwell_")]
    # The last second of each dwell is the accepted post-dwell measurement window.
    grouped={}
    for r in dwell: grouped.setdefault(r["phase"],[]).append(r)
    settled=[]
    for phase, group in grouped.items():
        end=max(float(r["time_s"]) for r in group); window=[r for r in group if float(r["time_s"])>=end-1.0]
        settled.append({"phase":phase,"max_abs_error_deg":max(abs(float(r["command_rad"])-float(r["measured_rad"]))*deg for r in window)})
    max_error=max((x["max_abs_error_deg"] for x in settled),default=0.0)
    # DART's constrained stop settles at ~1e-6 rad outside the nominal value;
    # 1e-5 rad is a numerical comparison tolerance, not extra physical travel.
    low,high=map(math.radians,(-3,20)); violations=sum(float(r["measured_rad"])<low-1e-5 or float(r["measured_rad"])>high+1e-5 for r in data)
    initial_z=float(data[0]["base_z_m"])
    settle_rows=[r for r in data if 3.5 <= float(r["time_s"]) < 4.0]
    if not settle_rows: raise SystemExit("telemetry does not contain the frozen 3.5..4.0 s settled-datum window")
    base_z0=sum(float(r["base_z_m"]) for r in settle_rows)/len(settle_rows)
    max_roll=max(abs(float(r["base_roll_rad"]))*deg for r in data); max_pitch=max(abs(float(r["base_pitch_rad"]))*deg for r in data)
    post_settle=[r for r in data if float(r["time_s"]) >= 4.0]
    rise=max(float(r["base_z_m"])-base_z0 for r in post_settle)
    spawn_correction=base_z0-initial_z
    max_torque=max(abs(float(r["command_nm"])) for r in data)
    launcher_tokens=c["launcher_owner_tokens"]; env_tokens=c["forbidden_environment_owner_tokens"]
    def forbidden(r):
        one,two=r["collision_1"],r["collision_2"]
        return (any(x in one for x in launcher_tokens) and any(x in two for x in env_tokens)) or (any(x in two for x in launcher_tokens) and any(x in one for x in env_tokens))
    forbidden_rows=[r for r in contact if forbidden(r)]
    max_force=max((float(r["max_force_n"]) for r in contact),default=0.0)
    metrics={"dwell_joint_target_error_deg":max_error,"forbidden_launcher_contact_count":len(forbidden_rows),"nan_or_inf_state_count":len(finite_errors),"joint_limit_violation":violations,"absolute_roll_deg":max_roll,"absolute_pitch_deg":max_pitch,"base_rise_from_settled_m":rise,"spawn_to_settled_z_correction_m_diagnostic":spawn_correction,"settled_datum_window_s":[3.5,4.0],"tilt_command_torque_nm":max_torque,"contact_force_max_n_diagnostic":max_force,"chassis_horizontal_displacement_m_diagnostic":math.hypot(float(data[-1]["base_x_m"])-float(data[0]["base_x_m"]),float(data[-1]["base_y_m"])-float(data[0]["base_y_m"]))}
    limits={x["name"]:x for x in c["criteria"] if a.case in x["cases"] and x["hard_fail"]}
    values={"dwell_joint_target_error":max_error,"forbidden_launcher_contact_count":len(forbidden_rows),"nan_or_inf_state_count":len(finite_errors),"joint_limit_violation":violations,"absolute_roll":max_roll,"absolute_pitch":max_pitch,"base_rise_from_settled":rise,"tilt_command_torque":max_torque}
    checks=[]
    for name,rule in limits.items():
        v=values[name]; t=rule["threshold"]; ok=(v<=t+1e-12 if rule["comparison"] in ("max_le","abs_max_le") else v==t)
        checks.append({"name":name,"value":v,"threshold":t,"unit":rule["unit"],"pass":ok})
    result={"case":a.case,"pass":all(x["pass"] for x in checks),"scope":"dynamic_tilt" if a.case!="ballast" else "spawn_stability_only","metrics":metrics,"checks":checks,"settled_windows":settled,"forbidden_contact_examples":forbidden_rows[:20],"contact_row_count":len(contact),"limitations":["P16 force/current/linkage stress/singularities excluded","terminal impact excluded; joint limits are stops","contact force is diagnostic only"]}
    a.output.write_text(json.dumps(result,indent=2)+"\n"); print(json.dumps(result,indent=2))
    if not result["pass"]: raise SystemExit(1)
if __name__=="__main__": main()
