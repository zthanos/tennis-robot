#!/usr/bin/env python3
"""Fetch Pi P2-63 logs with scp, then compare odometry to frozen P2-62."""
from __future__ import annotations
import argparse,csv,json,math,subprocess
from pathlib import Path

def main():
    ap=argparse.ArgumentParser();ap.add_argument("--remote",help="e.g. tennisserver@10.42.0.1:~/tennis-robot-logs/");ap.add_argument("--input-dir",type=Path);ap.add_argument("--measurements",type=Path,help="optional JSON mapping run stem to tape x_m/y_m/yaw_deg and PSU current");ap.add_argument("--output-dir",type=Path,default=Path("results/physical_drive_p263"));ap.add_argument("--predictions",type=Path,default=Path("engineering/chassis-strength-bench/results/shared_flywheel_phase2/physical_drive_p262/drive_predictions.json"));args=ap.parse_args()
    args.output_dir.mkdir(parents=True,exist_ok=True)
    if args.remote:subprocess.run(["scp","-p",f"{args.remote.rstrip('/')}/*.csv",f"{args.remote.rstrip('/')}/*.json",str(args.output_dir)],check=True)
    source=args.input_dir or args.output_dir;pred=json.loads(args.predictions.read_text());reports=[];measurements=json.loads(args.measurements.read_text()) if args.measurements else {}
    for meta_path in sorted(source.glob("*_r*.json")):
        meta=json.loads(meta_path.read_text());csv_path=meta_path.with_suffix(".csv")
        if not csv_path.exists():continue
        rows=list(csv.DictReader(csv_path.open()));
        if not rows:continue
        last=rows[-1];name=meta["scenario"]
        if name.startswith("straight_"):pname=f"straight_3m_{name.split('_',1)[1]}"
        elif name.startswith("spin_"):pname=f"{name}_{meta['mode']}_yaw"
        elif name.startswith("arc_"):pname=f"{name}_180deg_gyro"
        else:pname="accel_brake_0_to_0p6_to_0"
        candidates=[p for p in pred["predictions"] if p["scenario"]==pname and p["loaded_radius_mm"]==round(meta["radius_mm"])]
        observed={"x_m":float(last["x_m"]),"y_m":float(last["y_m"]),"yaw_deg":math.degrees(float(last["yaw_rad"]))}
        physical=measurements.get(meta_path.stem)
        errors=None
        if physical:errors={"encoder_minus_tape_x_m":observed["x_m"]-physical["x_m"],"encoder_minus_tape_y_m":observed["y_m"]-physical["y_m"],"encoder_minus_tape_yaw_deg":observed["yaw_deg"]-physical["yaw_deg"]}
        reports.append({"run":meta_path.stem,"scenario":pname,"observed_encoder_odom":observed,"physical_measurement":physical,"encoder_minus_physical":errors,"frozen_by_mu":candidates,"note":"without --measurements, encoder odom alone cannot validate slip"})
    out=args.output_dir/"comparison_summary.json";out.write_text(json.dumps({"runs":reports},indent=2)+"\n");print(out)
if __name__=="__main__":main()
