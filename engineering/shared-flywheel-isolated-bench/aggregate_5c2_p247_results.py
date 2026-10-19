#!/usr/bin/env python3
"""Aggregate the frozen P2-47 relative basket-entry comparison."""
import argparse, csv, json
from collections import defaultdict
from pathlib import Path

ap=argparse.ArgumentParser(); ap.add_argument('--root',type=Path,required=True); ap.add_argument('--matrix',type=Path,required=True); ap.add_argument('--output',type=Path,required=True); ap.add_argument('--csv',type=Path,required=True); a=ap.parse_args()
groups=defaultdict(list); missing=[]
for line in a.matrix.read_text().splitlines():
    variant,mode,preload,repeat=line.split('\t'); path=a.root/variant/mode/f'preload_{preload}'/f'r{repeat}'/'summary.json'
    (groups[(variant,mode,int(preload))].append(json.loads(path.read_text())) if path.exists() else missing.append(str(path)))
rows=[]
for (variant,mode,preload),runs in sorted(groups.items()):
    row={'variant':variant,'mode':mode,'preload_count':preload,'resulting_total':preload+(mode=='delivery'),'successes':sum(r['pass'] for r in runs),'repeats':len(runs)}
    row['condition_pass']=row['successes']==row['repeats']
    if mode=='delivery':
        row.update(collection_successes=sum(r['checks']['collection_success'] for r in runs),
                   entrance_blocked_runs=sum(not r['checks']['entrance_blocked'] for r in runs),
                   preload_spill_runs=sum(not r['checks']['preload_retained_before_delivery'] for r in runs),
                   bounce_out_runs=sum(not r['checks']['bounce_out'] for r in runs),
                   contact_current_peak_a=max((r['metrics']['ball_contact_current']['max_abs_a'] or 0) for r in runs))
    else:
        row.update(motion_ball_losses=sum(r['metrics']['motion_ball_loss_count'] for r in runs),
                   min_baseline_retained=min(r['metrics']['baseline_retained_count'] for r in runs),
                   min_final_retained=min(r['metrics']['final_retained_count'] for r in runs))
    rows.append(row)
fields=[]
for row in rows:
    for key in row:
        if key not in fields: fields.append(key)
with a.csv.open('w',newline='') as f: w=csv.DictWriter(f,fieldnames=fields); w.writeheader(); w.writerows(rows)
rank=[]
for variant in ('high_hood','one_way_flap','lip_cover'):
    vr=[r for r in rows if r['variant']==variant]; dr=[r for r in vr if r['mode']=='delivery']; mr=next(r for r in vr if r['mode']=='motion')
    rank.append({'variant':variant,'fully_passing_conditions':sum(r['condition_pass'] for r in vr),'delivery_successes':sum(r['successes'] for r in dr),
                 'preload_spill_runs':sum(r['preload_spill_runs'] for r in dr),'entrance_blocked_runs':sum(r['entrance_blocked_runs'] for r in dr),
                 'motion_ball_losses':mr['motion_ball_losses'],'motion_condition_pass':mr['condition_pass']})
rank.sort(key=lambda r:(-r['fully_passing_conditions'],-r['delivery_successes'],r['preload_spill_runs'],r['entrance_blocked_runs'],r['motion_ball_losses']))
result={'status':'COMPLETE' if not missing else 'INCOMPLETE','stage':'P2-47 revision-3 Gazebo comparative basket-entry redesign','runs':sum(map(len,groups.values())),'criteria_status':'FROZEN_BEFORE_FIRST_P2_47_R3_GAZEBO_RUN','absolute_capacity_claim':False,'ranking':rank,'rows':rows,'missing':missing}
a.output.write_text(json.dumps(result,indent=2)+'\n'); print(json.dumps({k:v for k,v in result.items() if k!='rows'},indent=2)); raise SystemExit(bool(missing))
