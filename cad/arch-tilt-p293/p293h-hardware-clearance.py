#!/usr/bin/env python3
"""PARKED reference envelopes of catalogue fasteners; no candidate solids."""
import json,math
from pathlib import Path
import numpy as np,trimesh,manifold3d as M
H=Path(__file__).resolve().parent;O=H/'p293h-analysis';S=H.parent/'independent-check/out/scene'
r=json.loads((O/'baseline-report.json').read_text())
def solid(mesh):return M.Manifold(M.Mesh(vert_properties=np.asarray(mesh.vertices,np.float32),tri_verts=np.asarray(mesh.faces,np.uint32)))
def cy(x,y0,y1,z,d,hole=0):
 mesh=trimesh.creation.cylinder(d/2,y1-y0,sections=48)
 mesh.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2,[1,0,0]));mesh.apply_translation([x,(y0+y1)/2,z]);s=solid(mesh)
 return s-cy(x,y0-1,y1+1,z,hole) if hole else s
hardware={};L=math.hypot(132,125)
for side in [-1,1]:
 for t in [L-55,L-35]:
  x=258+t*132/L;z=120+t*125/L
  head=cy(x,232.1,234.9,z,8.1)
  if side<0:head=head.mirror([0,1,0])
  hardware[f'crown_hex_head_{side}_{t}']=head
 for lo,hi in [(10.8,14)]:
  yy=sorted([-205+side*lo,-205+side*hi]);hardware[f'B_full_nut_{side}']=cy(260,*yy,220,8.1,4.1)
hits=[];gap=100.
for a in [-10,-3,0,15,20]:
 meshes={n.rsplit('@',1)[0]:solid(trimesh.load(S/f,force='mesh')) for n,f in r['files'].items() if n.endswith('@'+str(a))}
 for n,h in hardware.items():
  gap=min(gap,h.min_gap(meshes['f_side_front_concept'],100))
  for name,b in meshes.items():
   aa=h.bounding_box();bb=b.bounding_box()
   if any(aa[i+3]<bb[i] or bb[i+3]<aa[i] for i in range(3)):continue
   v=(h^b).volume()
   if v>.0001:hits.append(dict(angle=a,fastener=n,body=name,volume_mm3=v))
out=dict(status='PARKED_SUPPLEMENTAL_FASTENER_ENVELOPE_ONLY_NOT_NEW_SOLID_GATE',raw_hits=hits,
         min_F_gap_mm=gap,max_abs_Y=max(max(abs(h.bounding_box()[1]),abs(h.bounding_box()[4])) for h in hardware.values()),
         assumptions=['Head circumscribedD8.1 x2.8, retain originalOD9x0.5 washer.',
                      'B normal nut3.2, outer radius4.05, bore4.1; same M4x16.',
                      'Crown smooth grip>=22mm and d>=3.8 MUST be checked, not inferred from head.'])
(O/'hardware-clearance.json').write_text(json.dumps(out,indent=2)+'\n');print(out)
