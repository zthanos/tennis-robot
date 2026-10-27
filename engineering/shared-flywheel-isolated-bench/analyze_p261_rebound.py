#!/usr/bin/env python3
import argparse,csv,json
from pathlib import Path
a=argparse.ArgumentParser();a.add_argument('--telemetry',type=Path,required=True);a.add_argument('--dt',type=float,required=True);a.add_argument('--output',type=Path,required=True);x=a.parse_args()
r=[{k:float(v) for k,v in q.items()} for q in csv.DictReader(x.telemetry.open())]
impact=next(i for i,q in enumerate(r) if q['z_m']<=.034)
apex=max(q['z_m'] for q in r[impact:])-.033
o={'phase':'P2-61','timestep_s':x.dt,'rebound_ball_bottom_apex_m':apex,'pass':1.35<=apex<=1.47};x.output.write_text(json.dumps(o,indent=2)+'\n');print(json.dumps(o,indent=2))
