#!/usr/bin/env python3
"""Nine views from ACTUAL final scene meshes, not concept sketches."""
import json
from pathlib import Path
import numpy as np,trimesh,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
H=Path(__file__).parent;O=H/'p293g-candidate-results';S=H.parent/'independent-check/out/scene'
d=json.loads((O/'final-report.json').read_text())
plt.rcParams.update({'font.size':8})
fig=plt.figure(figsize=(17,15),layout='constrained')
for col,a in enumerate([-10,0,20]):
 meshes={k.rsplit('@',1)[0]:trimesh.load(S/v,force='mesh') for k,v in d['files'].items() if k.endswith('@'+str(a))}
 for row in range(3):
  ax=fig.add_subplot(3,3,row*3+col+1,projection='3d');ax.set_proj_type('ortho')
  if row==0:limits=((190,460),(-250,245),(40,365));title='Assembly'
  elif row==1:limits=((230,355),(-245,-115),(135,265));title='Cradle / console'
  else:limits=((330,465),(-240,-175),(70,190));title='Nut arm / yoke'
  for n,m in meshes.items():
   if n.startswith(('p293','arch_crossbar','joint_sleeve')):colour='.64';alpha=1
   elif n=='moving_module_concept':colour='.80';alpha=.16
   elif row==0 and n in ['drive_package_envelopes','p271_basket_current']:colour='.76';alpha=.10
   else:continue
   if 'console' in n:colour='#507b9a'
   elif n in ['p293g_cradle','p293g_float_cap']:colour='#c28b4c'
   elif n in ['p293g_nut_arm','p293g_nut_trunnion']:colour='#548a6e'
   elif 'hardware' in n:colour='.28'
   tri=m.triangles;cent=tri.mean(axis=1)
   if row>0:tri=tri[cent[:,1]<-140]
   if row>0:
    clipped=trimesh.Trimesh(vertices=tri.reshape(-1,3),faces=np.arange(len(tri)*3).reshape(-1,3),process=False)
    for axis,(low,high) in enumerate(limits):
     normal=np.eye(3)[axis];clipped=trimesh.intersections.slice_mesh_plane(clipped,normal*low,normal)
     clipped=trimesh.intersections.slice_mesh_plane(clipped,normal*high,-normal)
    tri=clipped.triangles
   if len(tri):ax.add_collection3d(Poly3DCollection(tri,facecolors=colour,edgecolors='none',alpha=alpha,rasterized=True))
  if row==0:limits=((190,460),(-250,245),(40,365));title='Assembly'
  elif row==1:limits=((230,355),(-245,-115),(135,265));title='Cradle / console'
  else:limits=((330,465),(-240,-175),(70,190));title='Nut arm / yoke'
  ax.set(xlim=limits[0],ylim=limits[1],zlim=limits[2],xlabel='X [mm]',ylabel='Y [mm]',zlabel='Z [mm]',title=f'{title} | {a:+d} deg')
  ax.set_box_aspect([v[1]-v[0] for v in limits]);ax.view_init(elev=30 if row==2 else 22,azim=-20 if row==2 else -58)
fig.suptitle('P2-93g final candidate solids — FIT-CHECK / NO OPERATION until P2-93h stops\nBlue: fixed console; ochre: rocking cradle; green: moving nut arm; dark: purchased hardware envelopes',fontsize=12)
fig.savefig(O/'candidate-views.png',dpi=150);plt.close(fig)
