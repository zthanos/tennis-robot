#!/usr/bin/env python3
"""Inspect the mandatory scene report; no filtering or mutation of the gate."""
import json
from pathlib import Path
import numpy as np
import trimesh
import manifold3d as m3d

CAD=Path(__file__).resolve().parents[1]
OUT=CAD/'independent-check/out/scene'
r=json.loads((OUT/'report.json').read_text())
def arch(n):return n.startswith(('p293d_','arch_crossbar_center','joint_sleeve_x'))
ah=[h for h in r['hits']+r['contained'] if arch(h['a']) or arch(h['b'])]
internal=[h for h in ah if arch(h['a']) and arch(h['b'])]
concept=[h for h in ah if 'f_side_front_concept' in (h['a'],h['b'])]
other=[h for h in ah if h not in internal and h not in concept]
def solid(n,a=0):
 mesh=trimesh.load(OUT/r['files'][f'{n}@{a}'],force='mesh')
 return m3d.Manifold(m3d.Mesh(vert_properties=np.asarray(mesh.vertices,np.float32),tri_verts=np.asarray(mesh.faces,np.uint32)))
au=m3d.Manifold()
for n in r['bodies']['0']:
 if arch(n):au=au+solid(n)
stats=[]
for p in sorted(Path(__file__).with_name('stl-candidate-p293d').glob('*.stl')):
 m=trimesh.load(p,force='mesh')
 stats.append({'stl':p.name,'bodies':len(m.split()),'watertight':m.is_watertight,
  'volume_mm3':abs(m.volume),'bounds_mm':m.bounds.tolist(),'size_mm':m.extents.tolist()})
result={'angles':list(r['bodies']),'bodies_per_angle':{a:len(v) for a,v in r['bodies'].items()},
 'full_scene_hits':len(r['hits']),'contained':len(r['contained']),
 'duplicates':len(r['duplicates_dropped']),'export_failures':r['export_failures'],
 'arch_internal_hits':internal,'arch_F_concept_hits':concept,'arch_other_hits':other,
 'arch_drive_min_gap_mm':au.min_gap(solid('drive_package_envelopes'),30),
 'candidate_stls':stats,
 'status':'ARCH_TARGETS_CLEAR' if not internal and not other and not r['export_failures'] else 'FAIL'}
print(json.dumps(result,indent=2))
raise SystemExit(0 if result['status']=='ARCH_TARGETS_CLEAR' and all(s['bodies']==1 and s['watertight'] for s in stats) else 1)
