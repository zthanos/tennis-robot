#!/usr/bin/env python3
from __future__ import annotations
import argparse, json
from pathlib import Path
ap=argparse.ArgumentParser(); ap.add_argument('--root',type=Path,default=Path('engineering/chassis-strength-bench/results/shared_flywheel_phase2/gazebo_5c_p251_calibration')); a=ap.parse_args()
root=a.root
rows=[];missing=[]
def tag(x):return str(x).replace('-','m').replace('.','p')
for speed in (6.0,10.0,14.0):
 for tm in (.6,.9):
  for rm in (.15,.25,.4):
   for y in (-.1,-.05,0.0,.05,.1):
    p=root/f'a_35/v_{tag(speed)}/tm_{tag(tm)}/rm_{tag(rm)}/y_{tag(y)}/summary.json'
    if not p.exists():missing.append(str(p));continue
    d=json.loads(p.read_text());m=d['metrics'];rows.append({**d['condition'],'pass':d['pass'],'delivery_success':d['delivery_success'],'takeoff_speed_m_s':m['takeoff_speed_m_s'],'takeoff_angle_deg':m['takeoff_angle_deg'],'ball_top_z_m':m['highest_ball_top_z_m'],'lidar_margin_m':m['lidar_margin_m'],'retained':d['checks']['retained_empty_basket'],'current_max_a':m['contact_max_current_a']})
out={'status':'INCOMPLETE' if missing else 'COMPLETE','conditions':len(rows),'passes':sum(r['pass'] for r in rows),'deliveries':sum(r['delivery_success'] for r in rows),'rows':rows,'missing':missing}
(root/'summary.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(out,indent=2))
