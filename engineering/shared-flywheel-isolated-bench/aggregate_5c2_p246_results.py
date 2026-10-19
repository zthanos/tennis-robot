#!/usr/bin/env python3
"""Aggregate frozen P2-46 delivery/capacity/motion results."""
from __future__ import annotations

import argparse
import csv
import json
from collections import defaultdict
from pathlib import Path


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", type=Path, required=True); ap.add_argument("--matrix", type=Path, required=True)
    ap.add_argument("--output", type=Path, required=True); ap.add_argument("--csv", type=Path, required=True)
    args = ap.parse_args(); groups=defaultdict(list); missing=[]
    for line in args.matrix.read_text().splitlines():
        mode, preload, repeat = line.split("\t"); path=args.root/mode/f"preload_{preload}"/f"r{repeat}"/"summary.json"
        if path.exists(): groups[(mode,int(preload))].append(json.loads(path.read_text()))
        else: missing.append(str(path))
    rows=[]
    for (mode,preload), runs in sorted(groups.items()):
        successes=sum(run["pass"] for run in runs)
        row={"mode":mode,"preload_count":preload,"resulting_total":preload+(mode=="delivery"),"successes":successes,"repeats":len(runs),"condition_pass":successes==len(runs)}
        if mode=="delivery":
            row.update({"collection_successes":sum(run["checks"]["collection_success"] for run in runs),
                        "bounce_out_count":sum(not run["checks"]["bounce_out"] for run in runs),
                        "entrance_blocked_count":sum(not run["checks"]["entrance_blocked"] for run in runs),
                        "preload_spill_runs":sum(not run["checks"]["preload_retained_before_delivery"] for run in runs),
                        "contact_current_peak_a":max((run["metrics"]["ball_contact_current"]["max_abs_a"] or 0) for run in runs),
                        "contact_current_mean_a":sum((run["metrics"]["ball_contact_current"]["mean_abs_a"] or 0) for run in runs)/len(runs),
                        "forward_push_max_m":max(run["metrics"]["pre_nip_forward_displacement_m"] for run in runs),
                        "lateral_push_max_m":max(run["metrics"]["pre_nip_lateral_displacement_m"] for run in runs)})
        else:
            row.update({"motion_ball_losses":sum(run["metrics"]["motion_ball_loss_count"] for run in runs),
                        "minimum_final_retained":min(run["metrics"]["final_retained_count"] for run in runs),
                        "max_abs_roll_deg":max(run["metrics"]["max_abs_roll_deg"] for run in runs),
                        "max_abs_pitch_deg":max(run["metrics"]["max_abs_pitch_deg"] for run in runs)})
        rows.append(row)
    fields=[]
    for row in rows:
        for key in row:
            if key not in fields: fields.append(key)
    with args.csv.open("w",newline="") as stream:
        writer=csv.DictWriter(stream,fieldnames=fields);writer.writeheader();writer.writerows(rows)
    delivery={row["preload_count"]:row for row in rows if row["mode"]=="delivery"}
    core_pass=all(delivery.get(n,{}).get("successes")==5 for n in (0,10,25,40))
    capacity=[row for row in rows if row["mode"]=="delivery" and row["preload_count"] in (44,49,54,59)]
    passing_totals=[row["resulting_total"] for row in capacity if row["condition_pass"]]
    failing_totals=[row["resulting_total"] for row in capacity if not row["condition_pass"]]
    motion=next((row for row in rows if row["mode"]=="motion"),None)
    result={"status":"COMPLETE" if not missing else "INCOMPLETE","stage":"P2-46 Gazebo 5c-2",
            "runs":sum(len(v) for v in groups.values()),"core_delivery_pass":core_pass,
            "target_45_pass":45 in passing_totals,"largest_probed_practical_total":max(passing_totals,default=None),
            "first_probed_closed_or_failed_total":min(failing_totals,default=None),"capacity_resolution_balls":5,
            "motion_retention_pass":bool(motion and motion["condition_pass"] and motion["motion_ball_losses"]==0),
            "overall_pass":bool(core_pass and 45 in passing_totals and motion and motion["condition_pass"] and motion["motion_ball_losses"]==0),
            "missing":missing,"criteria_status":"FROZEN_BEFORE_FIRST_P2_46_GAZEBO_RUN","rows":rows}
    args.output.write_text(json.dumps(result,indent=2)+"\n");print(json.dumps({k:v for k,v in result.items() if k!="rows"},indent=2))
    if missing: raise SystemExit(1)


if __name__=="__main__": main()
