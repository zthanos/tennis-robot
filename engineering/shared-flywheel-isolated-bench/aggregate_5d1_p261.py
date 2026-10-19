#!/usr/bin/env python3
import json
from pathlib import Path
r=Path('engineering/chassis-strength-bench/results/shared_flywheel_phase2/gazebo_5d1_p261');rows=[]
for p in sorted(r.glob('cases/tm_*/surface_*/feed_*/summary.json')):
 d=json.loads(p.read_text());rows.append({'mu':d['tread_mu'],'surface_m_s':d['surface_speed_m_s'],'feed_m_s':d['feed_speed_m_s'],'exit_m_s':d['metrics']['exit_speed_m_s'],'elevation_deg':d['metrics']['exit_elevation_deg'],'spin_rad_s':d['metrics']['spin_magnitude_rad_s'],'rpm_drop':d['metrics']['rpm_drop'],'recovery_s':d['metrics']['recovery_to_95pct_s'],'peak_current_a':d['metrics']['contact_peak_current_a'],'pass':d['pass'],'path':str(p)})
o={'phase':'P2-61','cases':len(rows),'passes':sum(x['pass'] for x in rows),'rows':rows,'target_cells':[x for x in rows if x['exit_m_s'] is not None and 12<=x['exit_m_s']<=14]};(r/'summary.json').write_text(json.dumps(o,indent=2)+'\n');print(json.dumps(o,indent=2))
