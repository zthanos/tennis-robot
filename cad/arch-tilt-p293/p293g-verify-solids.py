#!/usr/bin/env python3
"""Supplement the UNCHANGED scene gate: raw1deg sweep, exact mesh gaps, Y limit.
No subtraction/mating exemptions. Uses actual scene meshes and CAD rigid poses.
"""
import json,itertools,math
from pathlib import Path
import numpy as np
import trimesh,manifold3d as m3d
H=Path(__file__).parent;O=H/'p293g-candidate-results'
report=json.loads((O/'final-report.json').read_text())
S=H.parent/'independent-check/out/scene'
meshes={k.rsplit('@',1)[0]:trimesh.load(S/v,force='mesh') for k,v in report['files'].items() if k.endswith('@0')}
def solid(m):return m3d.Manifold(m3d.Mesh(vert_properties=np.asarray(m.vertices,np.float32),tri_verts=np.asarray(m.faces,np.uint32)))
base={n:solid(m) for n,m in meshes.items()}
P=np.array([387.,0.,51.837363636]);B=np.array([260.,-205.,220.])
def npos(a):
 t=np.radians(a);return P+[30*np.cos(t)-110*np.sin(t),-205,30*np.sin(t)+110*np.cos(t)]
def frame(a):
 u=npos(a)-B;u/=np.linalg.norm(u);return np.column_stack([[-u[2],0,u[0]],[0,-1,0],u])
def transform(n,a):
 t=np.radians(a);R=np.array([[np.cos(t),0,-np.sin(t)],[0,1,0],[np.sin(t),0,np.cos(t)]])
 if n in ['moving_module_concept','p293g_nut_arm']:return R,P-R@P
 if n in ['p293g_nut_trunnion','p293g_t8_nut_hardware']:
  R=frame(a)@frame(0).T;return R,npos(a)-R@npos(0)
 if n.startswith('p293g_') and ('hardware' in n or n in ['p293g_cradle','p293g_float_cap']):
  R=frame(a)@frame(0).T;return R,B-R@B
 return np.eye(3),np.zeros(3)
def posed(n,a):
 R,T=transform(n,a);return base[n].transform(np.column_stack([R,T]))
arch=[n for n in base if n.startswith(('p293','arch_crossbar_center','joint_sleeve_x'))]
fixed=[n for n in arch if not ('hardware' in n or n in ['p293g_cradle','p293g_float_cap','p293g_nut_arm','p293g_nut_trunnion'])]
hits=[];gaps=[];bounds=[];motor_x=[]
for a in range(-10,21):
 solids={n:posed(n,a) for n in base}
 motor_x.append(dict(angle=a,min_X_mm=solids['p293g_fit_hardware'].bounding_box()[0]))
 for n in arch:
  slab=solids[n].trim_by_plane([1,0,0],220).trim_by_plane([-1,0,0],-420)
  if not slab.is_empty():
   bb=slab.bounding_box();bounds.append(dict(angle=a,part=n,max_abs_Y=max(abs(bb[1]),abs(bb[4]))))
 for n,q in itertools.combinations(solids,2):
  if n not in arch and q not in arch:continue
  aa=solids[n].bounding_box();bb=solids[q].bounding_box()
  if any(aa[i+3]<bb[i] or bb[i+3]<aa[i] for i in range(3)):continue
  vol=(solids[n]^solids[q]).volume()
  if vol>0.0001:hits.append(dict(angle=a,a=n,b=q,volume_mm3=vol))
 if a in [-10,-3,0,15,20]:
  def gap(names,target):
   val,n=min((solids[n].min_gap(solids[target],100),n) for n in names if n!=target)
   return dict(mm=val,part=n)
  gaps.append(dict(angle=a,arch_to_moving=gap(fixed,'moving_module_concept'),
                   arch_to_cradle=gap(fixed,'p293g_cradle'),
                   arch_to_arm=gap(fixed,'p293g_nut_arm'),
                   crown_to_Tr8=gap([n for n in fixed if n.startswith('p293g_crown')],'p293g_tr8_hardware'),
                   arm_to_moving=gap(['p293g_nut_arm'],'moving_module_concept'),
                   arm_to_cradle=gap(['p293g_nut_arm'],'p293g_cradle'),
                   arch_tilt_to_drive=gap(arch,'drive_package_envelopes'),
                   arch_to_F=gap(fixed,'f_side_front_concept')))
 print('angle',a,flush=True)
data=dict(raw_hits=hits,minimum_gaps=gaps,motor_min_X=min(motor_x,key=lambda r:r['min_X_mm']),Y_limit=dict(limit_mm=240,worst=max(bounds,key=lambda r:r['max_abs_Y']),all=bounds),
          method='actual manifold meshes transformed by CAD kinematics; raw intersections each1deg; min_gap triangle distance, no masks')
(O/'solid-clearances.json').write_text(json.dumps(data,indent=2)+'\n')
print('hits',hits,'gaps',gaps,'worstY',data['Y_limit']['worst'])
