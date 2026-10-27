#!/usr/bin/env python3
import json
from pathlib import Path
base=Path('engineering/chassis-strength-bench/results/shared_flywheel_phase2');rows=[]
for root in ('gazebo_5c3_p261_p255','gazebo_5c3_p261_p258'):
    for p in sorted((base/root).glob('matrix/**/summary.json')):
        d=json.loads(p.read_text());m=d['metrics']
        rows.append({'source':root,'case':d['case'],'mu':d['tread_mu'],'pass':d.get('pass',False),'balls_in_basket':m.get('balls_in_basket'),'jammed':m.get('jammed'),'d5065_triggered':m.get('d5065_detector_triggered') or m.get('jam_detector',{}).get('detected'),'path':str(p.parent)})
o={'phase':'P2-61','cases':len(rows),'series_passes':sum(x['pass'] for x in rows if 'series_' in x['case']),'series_total':sum('series_' in x['case'] for x in rows),'rows':rows}
p=base/'gazebo_5c3_p261_summary.json';p.write_text(json.dumps(o,indent=2)+'\n');print(json.dumps({'cases':o['cases'],'series':f"{o['series_passes']}/{o['series_total']}"},indent=2))
