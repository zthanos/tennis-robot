import json, numpy as np, trimesh, manifold3d as m
from pathlib import Path
from math import sin,cos,radians,degrees,atan2,sqrt
root=Path(__file__).resolve().parents[3]; out=root/'cad/independent-check/out/scene'
dest=Path(__file__).parent
r=json.loads((out/'report.json').read_text()); pivot_z=23-55*sin(radians(-3))+30*cos(radians(-3))-4
P=np.array([387.,-205,pivot_z]); angles=[-10,-3,0,15,20]
def load(n,a):
 t=trimesh.load(out/r['files'][f'{n}@{a}'],force='mesh');return m.Manifold(m.Mesh(np.asarray(t.vertices,np.float32),np.asarray(t.faces,np.uint32)))
targets=['moving_module_concept','drive_package_envelopes','p271_basket_current','f_side_front_concept']
# Basket discovery module is instrumented to its actual leaf names.
names=r['bodies']['0'];targets=[n for n in names if n in targets or n.startswith(('p271_basket','p293d_','arch_crossbar_center','joint_sleeve_x'))]
ss0={n:load(n,0) for n in targets}
def scene(a):
 t=radians(a);R=np.array([[cos(t),0,-sin(t)],[0,1,0],[sin(t),0,cos(t)]])
 p=np.array([387.,0,pivot_z]);T=np.column_stack([R,p-R@p])
 return {n:s.transform(T) if n=='moving_module_concept' else s for n,s in ss0.items()}
ss={a:scene(a) for a in range(-10,21)}
def place(s,p,u):
 n=np.array([-u[2],0,u[0]]);v=np.array([0,-1,0]);return s.transform(np.column_stack([n,v,u,np.array(p)]))
def components(B,local,a,yoff=50,centre=60,direction=1):
 t=radians(a);x,z=local;N=P+np.array([x*cos(t)-z*sin(t),0,x*sin(t)+z*cos(t)])
 u=N-B;L=np.linalg.norm(u);u/=L;n=np.array([-u[2],0,u[0]])
 delta=centre*(sqrt(11)/6*n+np.array([0,5/6,0]))
 face=B+delta+(-17 if direction>0 else 40)*u
 motor=place(m.Manifold.cylinder(87,18.5,18.5,48),face,u*direction)
 bearings=m.Manifold()
 for s in [-15,20]:bearings= bearings+place(m.Manifold.cube([32,60,14],True),B+s*u,u)
 nut=place(m.Manifold.cube([32,44,40],True),N,u)
 maxL=max(np.linalg.norm(P+np.array([x*cos(radians(q))-z*sin(radians(q)),0,x*sin(radians(q))+z*cos(radians(q))])-B) for q in range(-10,21))
 back=45 if direction>0 else 25
 screw=place(m.Manifold.cylinder(maxL+30+back,4,4,32),B-back*u,u)
 pulley_s=-32 if direction>0 else 55
 # Convex stadium: two R10 / axial width16 guards in the SAME pulley plane.
 # This models the belt/guard outline, not an axis-aligned corner box.
 guard=m.Manifold.cylinder(16,10,10,64,True)
 belt=m.Manifold.batch_hull([place(guard,B+pulley_s*u,u),place(guard,B+delta+pulley_s*u,u)])
 thrust=place(m.Manifold.cylinder(12,13,13,40,True),B+34*u,u)
 joint=place(m.Manifold.cube([32,56,16],True),B,u)
 cable=place(m.Manifold.cylinder(10,10,10,32),face+87*direction*u,direction*u)
 return N,L,u,face,{'motor':motor,'bearings':bearings,'nut':nut,'screw':screw,'belt_keepout':belt,'thrust':thrust,'rocking_joint':joint,'cable_keepout':cable}
options=[('A',np.array([260.,-205,220]),(30,110),1),('B',np.array([280.,-205,265]),(30,120),-1)]
results=[]
for name,B,local,direction in options:
 rows=[]
 for a in range(-10,21):
  N,L,u,face,parts=components(B,local,a,direction=direction)
  hits=[];gaps={}
  for pn,s in parts.items():
   gaps[pn]={}
   for tn,t in ss[a].items():
    vol=(s^t).volume()
    if vol>1e-4:hits.append([pn,tn,round(vol,4)])
    if tn in ['moving_module_concept','drive_package_envelopes'] and a in angles:gaps[pn][tn]=s.min_gap(t,100)
  rows.append({'angle':a,'N':N.tolist(),'length':L,'alpha':degrees(atan2(u[2],u[0])),
    'moment_arm':abs((N[0]-P[0])*u[2]-(N[2]-P[2])*u[0]),
    'motor_xmin':parts['motor'].bounding_box()[0], 'face':face.tolist(),'hits':hits,'gaps':gaps,
    'nut_joint_relative_deg':degrees(atan2(u[2],u[0]))-a})
 extremes=[]
 for c in [57,63]:
  for a in range(-10,21):
   *_,parts=components(B,local,a,centre=c,direction=direction)
   for pn in ['motor','belt_keepout','cable_keepout']:
    for tn,t in ss[a].items():
     v=(parts[pn]^t).volume()
     if v>1e-4:extremes.append({'C':c,'angle':a,'part':pn,'target':tn,'mm3':v})
 results.append({'name':name,'B':B.tolist(),'local':local,'direction':direction,'rows':rows,'C57_C63_hits':extremes,
 'minimum_length_budget_mm':75 if direction>0 else 98,
 'screw_blank_mm':max(row['length'] for row in rows)+30+(45 if direction>0 else 25)})
encoder=m.Manifold.cube([28,13.2,28],True).translate([387,246.9,pivot_z])
encoder_checks=[{'angle':a,'target':n,'mm3':(encoder^ss[a][n]).volume()} for a in angles for n in ['moving_module_concept','drive_package_envelopes']]
data={'note':'ANALYSIS ENVELOPES, NOT MODIFIED CAD OR FULL SCENE GATE. No volume subtraction. 1deg samples, not continuous proof.',
 'options':results,'encoder_envelope_checks':encoder_checks,'actual_pivot_mm':[387,0,pivot_z],
 'targets':targets,'git_head':'9be0e03c','motor_nominal':'D37 x (17 gearbox +70 can), shaft projection15 provisional',
 'bearing_box_mm':[32,60,14],'nut_and_fork_box_mm':[32,44,40]}
(dest/'p293f-results.json').write_text(json.dumps(data,indent=2))
def mesh_data(s):
 t=s.to_mesh();return {'v':np.asarray(t.vert_properties)[:,:3].tolist(),'f':np.asarray(t.tri_verts).tolist()}
view={n:mesh_data(s) for n,s in ss0.items() if n.startswith(('p293d_','arch_crossbar_center','joint_sleeve_x')) or n in ['drive_package_envelopes','moving_module_concept','p271_basket_current']}
views={}
for name,B,local,direction in options:
 views[name]={str(a):{k:mesh_data(v) for k,v in components(B,local,a,direction=direction)[-1].items()} for a in [0,20]}
(dest/'p293f-view-meshes.json').write_text(json.dumps({'context':view,'options':views}))
for o in results:
 print(o['name'],'nominal hits',sum(len(row['hits']) for row in o['rows']),'C extremes hits',len(o['C57_C63_hits']),
 'Lmin',min(row['length'] for row in o['rows']),'screw',o['screw_blank_mm'])
 for row in o['rows']:
  if row['hits']:print(row['angle'],row['hits'])
 if o['C57_C63_hits']:print(o['C57_C63_hits'])
raise SystemExit(1 if any(any(row['hits'] for row in o['rows']) or o['C57_C63_hits'] for o in results) or any(x['mm3']>1e-4 for x in encoder_checks) else 0)
