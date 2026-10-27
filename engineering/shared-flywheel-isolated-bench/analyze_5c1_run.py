#!/usr/bin/env python3
"""Analyze one P2-43 one-ball capture repetition."""
from __future__ import annotations
import argparse,csv,json,math
from pathlib import Path
def read(p):
 with p.open(newline="") as f:
  # Gazebo is terminated after the controller writes telemetry.done.  A
  # contact stream can therefore end with one incomplete, unflushed CSV row;
  # it carries no usable sample and must not invalidate the completed run.
  return [row for row in csv.DictReader(f) if None not in row.values()]
def main():
 ap=argparse.ArgumentParser();ap.add_argument("telemetry",type=Path);ap.add_argument("ball_contacts",type=Path);ap.add_argument("native_contacts",type=Path);ap.add_argument("--criteria",type=Path,required=True);ap.add_argument("--output",type=Path,required=True);a=ap.parse_args()
 c=json.loads(a.criteria.read_text());r=read(a.telemetry);bc=read(a.ball_contacts);nc=read(a.native_contacts)
 if not r:raise SystemExit("empty telemetry")
 keys=[k for k in r[0] if k not in ("",)]
 nonfinite=sum(not math.isfinite(float(x[k])) for x in r for k in keys)
 def crossing(threshold):return next((x for x in r if float(x["time_s"])>=2 and float(x["ball_launcher_x_m"])<=threshold),None)
 nip=crossing(-.033);guide=crossing(-.105);nip_speed=-float(nip["ball_launcher_vx_m_s"]) if nip else None
 initial_x=float(r[0]["ball_x_m"]); pre=r if nip is None else [x for x in r if float(x["time_s"])<=float(nip["time_s"])]
 plough=max(float(x["ball_x_m"])-initial_x for x in pre)
 wheel_times=[float(x["time_s"]) for x in bc if int(float(x["contact_id"])) in (0,1) and float(x["normal_force_n"])>0]
 first_contact=min(wheel_times) if wheel_times else None
 longest=run_start=None
 longest=0.0
 for x in r:
  t=float(x["time_s"]);px=float(x["ball_launcher_x_m"]);speed=math.sqrt(sum(float(x[k])**2 for k in ("ball_launcher_vx_m_s","ball_launcher_vy_m_s","ball_launcher_vz_m_s")))
  stalled=first_contact is not None and t>=first_contact and -.105<=px<=.033 and speed<.05
  if stalled and run_start is None:run_start=t
  if not stalled and run_start is not None:longest=max(longest,t-run_start);run_start=None
 if run_start is not None:longest=max(longest,float(r[-1]["time_s"])-run_start)
 max_current=max(abs(float(x[k])) for x in r for k in ("left_current_a","right_current_a"));max_torque=max(abs(float(x[k])) for x in r for k in ("left_torque_nm","right_torque_nm"))
 launcher=("cad_upper_plate","cad_yoke_","cad_trunnion_","d5065_motor_","flywheel_left","flywheel_right",
           "intake_cheek_","intake_moving_guide_")
 env=("fixture_arch_","fixture_lidar_","chassis_","ground_plane")
 forbidden=[]
 for x in nc:
  p,q=x["collision_1"],x["collision_2"]
  if (any(v in p for v in launcher) and any(v in q for v in env)) or (any(v in q for v in launcher) and any(v in p for v in env)):forbidden.append(x)
 checks={"fully_through_nip":nip is not None,"guide_entry_reached":guide is not None,"nip_exit_speed":nip_speed is not None and nip_speed>=1.7,"plough":plough<=.1,"jam":longest<=.5,"d5065_current":max_current<=20+1e-9,"d5065_torque":max_torque<=.62+1e-9,"forbidden_launcher_contact_count":not forbidden,"finite_state":nonfinite==0}
 out={"pass":all(checks.values()),"checks":checks,"metrics":{"nip_crossing_time_s":float(nip["time_s"]) if nip else None,"guide_entry_time_s":float(guide["time_s"]) if guide else None,"nip_exit_rearward_speed_m_s":nip_speed,"meets_preferred_1p9_m_s":nip_speed is not None and nip_speed>=1.9,"pre_nip_forward_displacement_m":plough,"longest_jam_s":longest,"first_wheel_contact_time_s":first_contact,"max_current_a":max_current,"max_torque_nm":max_torque,"forbidden_contact_count":len(forbidden),"nonfinite_count":nonfinite},"forbidden_contact_examples":forbidden[:10]}
 a.output.write_text(json.dumps(out,indent=2)+"\n");print(json.dumps(out,indent=2))
if __name__=="__main__":main()
