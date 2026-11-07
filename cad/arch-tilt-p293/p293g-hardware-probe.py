#!/usr/bin/env python3
"""Raw fastener envelope checks supplemental to untouched full-scene gate.
Includes foot sleeve/recess stack and crown single-sided pins; no masks.
"""
import json,math
from pathlib import Path
import numpy as np,trimesh,manifold3d as M
H=Path(__file__).parent;O=H/'p293g-candidate-results';S=H.parent/'independent-check/out/scene'
r=json.loads((O/'final-report.json').read_text())
def solid(m):return M.Manifold(M.Mesh(vert_properties=np.asarray(m.vertices,np.float32),tri_verts=np.asarray(m.faces,np.uint32)))
def cy(x,y0,y1,z,od,id=0,hex=False):
 m=trimesh.creation.cylinder(od/2,abs(y1-y0),sections=6 if hex else 48)
 m.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2,[1,0,0]));m.apply_translation([x,(y0+y1)/2,z]);s=solid(m)
 return s-cy(x,y0-1,y1+1,z,id) if id else s
hardware={}
for side in [-1,1]:
 def put(n,x,y0,y1,z,od,id=0,hex=False):
  s=cy(x,y0,y1,z,od,id,hex)
  if side<0:s=s.mirror([0,1,0])
  hardware[f'{n}_{side}_{z}']=s
 for z in [80,114]:
  for n,y0,y1,od,id in [('outer_sleeve',231.8,236.8,8,4.3),('inner_sleeve',203.3,211.3,8,4.3),
     ('inner_shim',211.4,211.6,9,4.3),('outer_shim',231.6,231.8,9,4.3),
     ('outer_washer',236.8,237.8,10,4.3),('button_head',237.8,240,7.6,0),
     ('shaft',192.8,237.8,4,0),('inner_washer',202.2,203.2,12,4.3),('nyloc',197.2,202.2,8.1,4.1)]:put(n,258,y0,y1,z,od,id)
 L=math.hypot(132,125)
 for t in [L-55,L-35]:
  x=258+t*132/L;z=120+t*125/L
  #10.9 button head2.2 +thin steel washer0.5; threaded-root bending screened.
  for n,y0,y1,od,id in [('crown_head',232.1,234.3,7.6,0),('crown_washer',231.6,232.1,9,4.3),
      ('crown_shaft',187.1,232.1,4,0),('crown_shim',211.2,211.6,9,4.3),
      ('crown_inner_washer',198.4,199.2,9,4.3),('crown_nyloc',193.4,198.4,8.1,4.1)]:put(n,x,y0,y1,z,od,id)
hits=[];Fgap=100.;worst=None;mass_estimates={}
for x,z in [(242,131),(242,164),(304,128),(280,166)]:
 for n,y0,y1,od,id in [('console_head',-239.2,-236.4,8.1,0),('console_outer_washer',-236.4,-235.6,9,4.3),
     ('console_shaft',-236.4,-191.4,4,0),('console_inner_washer',-203.6,-202.8,9,4.3),('console_nyloc',-202.8,-197.8,8.1,4.1)]:
  hardware[f'{n}_{x}_{z}']=cy(x,y0,y1,z,od,id)
for angle in [-10,-3,0,15,20]:
 extras={}
 for x,z,name,specs in [(260,220,'B',[(10,26,4,0),(25.2,26,9,4.3),(26,28.2,7.6,0),(11.8,14,8.1,4.1)])]:
  for sign in [-1,1]:
   for j,(y0,y1,od,id) in enumerate(specs):
    vals=sorted([-205+sign*y0,-205+sign*y1]);extras[f'{name}_{sign}_{j}']=cy(x,*vals,z,od,id)
 a=math.radians(angle);x=387+30*math.cos(a)-110*math.sin(a);z=51.837363636+30*math.sin(a)+110*math.cos(a)
 for u in [-15,5]:
  for v in [-224,-184]:
   for j,(z0,z1,od,id) in enumerate([(59.1,60.1,10,4.3),(60.1,60.6,9,4.3),(60.6,63.4,8.1,0),(30.6,60.6,4,0),(38,39,10,4.3),(33,38,8.1,4.1)]):
    mesh=trimesh.creation.cylinder(od/2,z1-z0,sections=48);mesh.apply_translation([u,v,(z0+z1)/2]);s=solid(mesh)
    if id:
     mesh=trimesh.creation.cylinder(id/2,z1-z0+2,sections=48);mesh.apply_translation([u,v,(z0+z1)/2]);s=s-solid(mesh)
    T=np.array([[math.cos(a),0,-math.sin(a),387],[0,1,0,0],[math.sin(a),0,math.cos(a),51.837363636]])
    extras[f'arm_mount_{u}_{v}_{j}']=s.transform(T)
 for sign in [-1,1]:
  for j,(y0,y1,od,id) in enumerate([(12.1,14.3,7.6,0),(14.3,14.8,8,4.3),(14.3,30.3,4,0),(23.2,24,9,4.3),(24,29,8.1,4.1)]):
   vals=sorted([-205+sign*y0,-205+sign*y1]);extras[f'N_{sign}_{j}']=cy(x,*vals,z,od,id)
 scene={k.rsplit('@',1)[0]:solid(trimesh.load(S/v,force='mesh')) for k,v in r['files'].items() if k.endswith('@'+str(angle))}
 if angle==0:
  for n,s in {**hardware,**extras}.items():
   mm=s.to_mesh();tm=trimesh.Trimesh(vertices=np.asarray(mm.vert_properties)[:,:3],faces=np.asarray(mm.tri_verts),process=False)
   group=next((g for g,prefix in [('crown','crown_'),('console','console_'),('B','B_'),('N','N_'),('arm_mount','arm_mount_')] if n.startswith(prefix)),'foot')
   mass_estimates[n]=dict(group=group,mass_g=abs(tm.volume)*.00785,volume_mm3=abs(tm.volume),CM_mm=tm.center_mass.tolist())
 for n,s in {**hardware,**extras}.items():
  gap=s.min_gap(scene['f_side_front_concept'],100)
  if gap<Fgap:Fgap=gap;worst=n
  aa=s.bounding_box()
  for q,t in scene.items():
   bb=t.bounding_box()
   if any(aa[i+3]<bb[i] or bb[i+3]<aa[i] for i in range(3)):continue
   vol=(s^t).volume()
   if vol>.0001:hits.append(dict(angle=angle,hardware=n,body=q,vol_mm3=vol))
data=dict(hits=hits,min_F_gap_mm=Fgap,closest=worst,mass_estimates=mass_estimates,max_abs_Y=max(max(abs(s.bounding_box()[1]),abs(s.bounding_box()[4])) for s in hardware.values()),
          scope='Foot sleeves/button-head M4x45/shims; crown10.9 button-head M4x45/washer0.5/shim0.4; console M4x45; B short M4x16/DIN439 nuts2.2; N inward M4x16; arm M4x30/OD10 washers1/top shim0.5. Raw vs all scene bodies at five gate angles. Base bolts and M3 mount hardware remain hardware fit-check items, not modelled here.')
(O/'hardware-probe.json').write_text(json.dumps(data,indent=2)+'\n');print({k:v for k,v in data.items() if k!='mass_estimates'})
