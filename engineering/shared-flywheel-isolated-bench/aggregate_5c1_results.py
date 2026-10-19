#!/usr/bin/env python3
"""Aggregate five repeats into the frozen P2-43 condition matrix."""
from __future__ import annotations
import argparse,csv,json
from collections import defaultdict
from pathlib import Path
def tag(v,y,ch,mu,s,r):
 a=f"v_{v.replace('.','p')}/y_{y.replace('-','m')}";a=a.replace('.', 'p',1);return f"{a}/{ch}/mu_{mu.replace('.','p')}/surf_{s.replace('.','p')}/r{r}"
def main():
 ap=argparse.ArgumentParser();ap.add_argument("--root",type=Path,required=True);ap.add_argument("--matrix",type=Path,required=True);ap.add_argument("--criteria",type=Path,required=True);ap.add_argument("--output",type=Path,required=True);ap.add_argument("--csv",type=Path,required=True);a=ap.parse_args();criteria=json.loads(a.criteria.read_text());groups=defaultdict(list);missing=[]
 for line in a.matrix.read_text().splitlines():
  v,y,ch,mu,s,r=line.split('\t');p=a.root/tag(v,y,ch,mu,s,r)/"summary.json"
  if not p.exists():missing.append(str(p));continue
  groups[(v,y,ch,mu,s)].append(json.loads(p.read_text()))
 rows=[]
 for key,runs in sorted(groups.items()):
  success=sum(x["pass"] for x in runs);speeds=[x["metrics"]["nip_exit_rearward_speed_m_s"] for x in runs if x["metrics"]["nip_exit_rearward_speed_m_s"] is not None]
  rows.append({"approach_m_s":key[0],"lateral_m":key[1],"cheek":key[2],"tread_mu":key[3],"surface_speed_m_s":key[4],"successes":success,"repeats":len(runs),"condition_pass":success>=4,"nip_speed_min_m_s":min(speeds) if speeds else None,"nip_speed_max_m_s":max(speeds) if speeds else None,"preferred_speed_repeats":sum(x["metrics"]["meets_preferred_1p9_m_s"] for x in runs),"max_current_a":max(x["metrics"]["max_current_a"] for x in runs),"max_torque_nm":max(x["metrics"]["max_torque_nm"] for x in runs),"plough_failures":sum(not x["checks"]["plough"] for x in runs),"jam_failures":sum(not x["checks"]["jam"] for x in runs),"nip_failures":sum(not x["checks"]["fully_through_nip"] for x in runs),"guide_failures":sum(not x["checks"]["guide_entry_reached"] for x in runs),"current_failures":sum(not x["checks"]["d5065_current"] for x in runs),"torque_failures":sum(not x["checks"]["d5065_torque"] for x in runs),"finite_failures":sum(not x["checks"]["finite_state"] for x in runs),"forbidden_contacts":sum(x["metrics"]["forbidden_contact_count"] for x in runs)})
 fields=list(rows[0]) if rows else []
 with a.csv.open('w',newline='') as f:w=csv.DictWriter(f,fieldnames=fields);w.writeheader();w.writerows(rows)
 out={"status":"COMPLETE" if not missing else "INCOMPLETE","stage":"5c-1","conditions":len(rows),"runs":sum(len(x) for x in groups.values()),"passing_conditions":sum(x["condition_pass"] for x in rows),"failing_conditions":sum(not x["condition_pass"] for x in rows),"forbidden_contact_total":sum(x["forbidden_contacts"] for x in rows),"missing":missing,"criteria_status":criteria["status"],"rows":rows}
 a.output.write_text(json.dumps(out,indent=2)+"\n");print(json.dumps({k:v for k,v in out.items() if k!='rows'},indent=2))
 if missing:raise SystemExit(1)
if __name__=="__main__":main()
