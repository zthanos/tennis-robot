#!/usr/bin/env python3
"""Read-only independent mesh probes, all-angle gaps and gauge metrology."""
import json
from pathlib import Path
import numpy as np
import trimesh
import manifold3d as m

HERE=Path(__file__).resolve().parent
OUT=HERE.parent/'independent-check/out/scene'
r=json.loads((OUT/'report.json').read_text())
def solid(path):
 t=trimesh.load(path,force='mesh')
 return m.Manifold(m.Mesh(np.asarray(t.vertices,np.float32),np.asarray(t.faces,np.uint32)))
def body(n,a=0):return solid(OUT/r['files'][f'{n}@{a}'])
probes=[]
for side in [-1,1]:
 for name in [f'front_terminal_v3[side={side}]',f'clamp_sleeve_x[side={side}xx=200]']:
  s=body(name)
  for y in [207.5,220,232.5]:
   c=m.Manifold.cylinder(30,1,1,48).translate([230,side*y,22])
   probes.append({'body':name,'centre_xy_mm':[230,side*y],
                  'diameter2_through_probe_blocked_mm3':(s^c).volume()})
gaps={}
for a,names in r['bodies'].items():
 u=m.Manifold()
 for n in names:
  if n.startswith(('p293d_','arch_crossbar_center','joint_sleeve_x')):u=u+body(n,a)
 gaps[a]=u.min_gap(body('drive_package_envelopes',a),30)
gauges=[]
for name,side in [('p293e_foot_template.stl',1),('p293e_foot_template_R.stl',-1)]:
 p=HERE/'stl-candidate-p293d'/name;t=trimesh.load(p,force='mesh');s=solid(p)
 bores=[]
 for x,y in [(240,150),(240,160),(230,220)]:
  xy=[x-192,y-133 if side>0 else 242-y]
  c=m.Manifold.cylinder(4,2.2,2.2,64).translate(xy+[-1])
  bores.append({'installed_xy_mm':[x,side*y],'probe_diameter_mm':4.4,'blocked_mm3':(s^c).volume()})
 gauges.append({'stl':name,'side':'L' if side>0 else 'R','bodies':len(t.split()),
 'watertight':t.is_watertight,'bounds_mm':t.bounds.tolist(),'bores':bores})
ok=all(v>=8.9999 for v in gaps.values()) and all(g['bodies']==1 and g['watertight'] and all(b['blocked_mm3']<1e-6 for b in g['bores']) for g in gauges)
print(json.dumps({'status':'PASS' if ok else 'FAIL','drive_gaps_mm_by_angle':gaps,'existing_hole_probes':probes,'gauges':gauges},indent=2))
raise SystemExit(0 if ok else 1)
