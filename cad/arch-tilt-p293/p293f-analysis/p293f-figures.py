"""Render analysis artifacts, never change CAD or STL geometry."""
import json,csv
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

d=Path(__file__).parent
r=json.loads((d/'p293f-results.json').read_text())
meshes=json.loads((d/'p293f-view-meshes.json').read_text())
P=np.array(r['actual_pivot_mm']);angles=[-10,-3,0,15,20]
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':10})
def tris(s):return np.asarray(s['v'])[np.asarray(s['f'])]
def drawmesh(ax,s,color,alpha=1,clip=None):
 t=tris(s)
 if clip is not None:t=t[clip(t.mean(axis=1))]
 if len(t):ax.add_collection3d(Poly3DCollection(t,facecolors=color,edgecolors='none',alpha=alpha,rasterized=True))
fig=plt.figure(figsize=(17,9),layout='constrained')
for j,o in enumerate(r['options']):
 ax=fig.add_subplot(1,2,j+1,projection='3d');ax.set_proj_type('ortho')
 for n,s in meshes['context'].items():
  if n.startswith(('p293d_','arch_crossbar_center','joint_sleeve_x')):
   drawmesh(ax,s,'#89939b',.60,lambda c:(c[:,1]<-120)&(c[:,0]>220))
  elif n=='drive_package_envelopes':drawmesh(ax,s,'#baaaa6',.20,lambda c:(c[:,0]>220)&(c[:,1]<0))
  elif n=='moving_module_concept':drawmesh(ax,s,'#d9aa74',.14,lambda c:c[:,1]<-100)
  elif n=='p271_basket_current':drawmesh(ax,s,'#86a78d',.20,lambda c:c[:,0]>185)
 colors={'motor':'#317ea5','bearings':'#425865','nut':'#805b86','screw':'#333b44','belt_keepout':'#81afc4','thrust':'#b79f54','rocking_joint':'#657c65','cable_keepout':'#b3c9d2'}
 for n,s in meshes['options'][o['name']]['0'].items():drawmesh(ax,s,colors[n],.85 if n!='belt_keepout' else .4)
 B=np.array(o['B']);N=np.array(next(x for x in o['rows'] if x['angle']==0)['N'])
 anchors=[np.array([258,-228,80]),np.array([258,-228,114])] if o['name']=='A' else [np.array([350.1,-228,207.2]),np.array([364.6,-228,220.9])]
 for a in anchors:ax.plot(*np.array([a,B]).T,'--',color='#355d47',lw=2)
 ax.scatter(*B,color='#223a2a',s=35);ax.text(*B,'  B: rocking axis Y',fontsize=9)
 ax.scatter(*N,color='#64386d',s=35);ax.text(*N,'  N: nut / stub trunnions',fontsize=9)
 ax.set(xlim=(225,480),ylim=(-270,-100),zlim=(45,365),xlabel='X [mm], FRONT +X',ylabel='Y [mm]',zlabel='Z [mm]')
 ax.set_box_aspect((255,170,320));ax.view_init(elev=23,azim=-64)
 ax.set_title(f"{o['name']} | {'Knee console, motor forward' if o['name']=='A' else 'Crown console, motor rearward'}\n0° — hardware envelopes, NOT candidate STL")
fig.suptitle('P2-93f on unchanged P2-93e arch | dashed lines = proposed bolted consoles, not designed solids',fontsize=13)
fig.savefig(d/'p293f-options-3d.png',dpi=150);plt.close(fig)

fig,axes=plt.subplots(1,2,figsize=(16,8),layout='constrained')
for ax,o in zip(axes,r['options']):
 ax.plot([258,258,390],[52,120,245],color='#79858a',lw=13,alpha=.5,label='P2-93e arch centreline (Y−228)')
 ax.plot([380,405],[245,245],color='#79858a',lw=13,alpha=.5)
 ax.add_patch(Rectangle((280,52),40,53,facecolor='#bf9e96',alpha=.35));ax.text(281,75,'drive motor\nenvelope',fontsize=9)
 ax.axvline(225,color='#73957b',linestyle='--');ax.text(227,300,'basket ends X225',rotation=90,color='#53745b')
 B=np.array(o['B']);ax.scatter(B[0],B[2],s=55,color='#304f3b');ax.text(B[0]+3,B[2]+5,'B (axis Y)')
 for a,c in zip(angles,['#a6b3bd','#8097a8','#597c96','#335d7c','#163b56']):
  row=next(x for x in o['rows'] if x['angle']==a);N=np.array(row['N'])
  ax.plot([B[0],N[0]],[B[2],N[2]],color=c,label=f'{a:+}°: L={row["length"]:.1f} mm')
  ax.scatter(N[0],N[2],s=28,color=c)
 ax.plot([x['N'][0] for x in o['rows']],[x['N'][2] for x in o['rows']],':',color='#3b5060')
 ax.scatter(P[0],P[2],marker='+',s=100,color='black');ax.text(P[0]+3,P[2]+3,f'P Z{P[2]:.3f}')
 for a,tag in [(-10,'−10° stop'),(20,'+20° stop')]:
  t=np.deg2rad(a);q=P+[-150*np.sin(t),0,150*np.cos(t)]
  ax.scatter(q[0],q[2],marker='s',color='#8b7450');ax.text(q[0]+2,q[2]+4,tag,fontsize=9)
 ax.set(xlim=(220,480),ylim=(40,325),xlabel='X [mm]',ylabel='Z [mm]',title=f"{o['name']}: B={o['B']} | nut local U/W={o['local']}")
 ax.set_aspect('equal');ax.grid(alpha=.2);ax.legend(fontsize=8,loc='upper right')
fig.suptitle('Kinematics in screw plane Y−205 | exact gpoint convention | stops shown as contact points only')
fig.savefig(d/'p293f-kinematics.png',dpi=150);plt.close(fig)

fig,axes=plt.subplots(2,1,figsize=(17,6.5),layout='constrained')
headers=['θ [°]','N X [mm]','N Z [mm]','B→N [mm]','stroke* [mm]','α [°]','α−θ [°]','h [mm]','F@8Nm [N]']
for ax,o in zip(axes,r['options']):
 ax.axis('off');rows=[]
 low=min(x['length'] for x in o['rows'])
 for x in o['rows']:
  if x['angle'] in angles:rows.append([x['angle']]+[f'{v:.2f}' for v in [x['N'][0],x['N'][2],x['length'],x['length']-low,x['alpha'],x['nut_joint_relative_deg'],x['moment_arm'],8000/x['moment_arm']]])
 table=ax.table(cellText=rows,colLabels=headers,cellLoc='center',loc='center');table.auto_set_font_size(False);table.set_fontsize(10);table.scale(1,1.7)
 for (i,j),cell in table.get_celld().items():cell.set_edgecolor('#cad0d5');cell.set_facecolor('#e8edf0' if i==0 else 'white')
 ax.set_title(f"Option {o['name']} | B {o['B']} mm | nut local {o['local']} mm | all N at Y−205",loc='left',fontsize=12)
 with (d/f'p293f-{o["name"]}-kinematics.csv').open('w') as f:csv.writer(f).writerows([headers]+rows)
fig.suptitle('P2-93f kinematic tables | *stroke = L − Lmin(+20°); α = screw angle above +X; h = screw force moment arm\nα−θ = nut articulation relative to moving yoke. F is ideal 8 Nm load only, not friction/impact/self-locking certification.',fontsize=11)
fig.savefig(d/'p293f-tables.png',dpi=150);plt.close(fig)
print('Rendered 3D, kinematics, tables and CSV without touching repo CAD.')
