#!/usr/bin/env python3
"""Render scientific analysis figures, no CAD/STL generation."""
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

HERE=Path(__file__).parent;OUT=HERE/'p293g-rear-brace-results'
mesh=json.loads((HERE/'p293f-analysis/p293f-view-meshes.json').read_text())
plt.rcParams.update({'font.size':9,'font.family':'DejaVu Sans'})
def draw(ax,data,colour,alpha=.6,clip=None):
    tri=np.asarray(data['v'])[np.asarray(data['f'])]
    if clip is not None:tri=tri[clip(tri.mean(axis=1))]
    if len(tri):ax.add_collection3d(Poly3DCollection(tri,facecolors=colour,edgecolors='none',alpha=alpha,rasterized=True))
def box3(ax,a,b,width,colour,alpha=.8):
    a=np.array(a,float);b=np.array(b,float);u=(b-a)/np.linalg.norm(b-a)
    n=np.cross([0,1,0],u);n/=np.linalg.norm(n);v=np.cross(u,n)
    p=np.array([p+sn*width/2*n+sv*width/2*v for p in [a,b] for sn,sv in [(-1,-1),(-1,1),(1,1),(1,-1)]])
    faces=[p[ii]for ii in [[0,1,2,3],[4,5,6,7],[0,1,5,4],[1,2,6,5],[2,3,7,6],[3,0,4,7]]]
    ax.add_collection3d(Poly3DCollection(faces,facecolors=colour,edgecolors='none',alpha=alpha))

R=np.array([65.,-220.,60.]);J=np.array([320.,-204.,190.]);u=(J-R)/np.linalg.norm(J-R)
L=np.linalg.norm(J-R);joint=R+(L-64)*u
fig=plt.figure(figsize=(15,10),layout='constrained')
ax=fig.add_subplot(2,2,1,projection='3d');ax.set_proj_type('ortho')
for name,data in mesh['context'].items():
    if name.startswith(('p293d_','arch_crossbar_center','joint_sleeve_x')):
        draw(ax,data,'.55',.6,lambda t:(t[:,1]<-120))
    elif name=='drive_package_envelopes':draw(ax,data,'#aa9890',.16,lambda t:(t[:,1]<-120)&(t[:,0]>0))
for name,data in mesh['options']['A']['0'].items():
    if name in ['motor','bearings','screw','belt_keepout','nut']:draw(ax,data,'#426c86',.6)
box3(ax,R+20*u,J-20*u,20,'#44715a')
ax.plot(*np.array([[35,-220,52],[200,-220,52]]).T,color='.4',lw=5)
ax.scatter(*R,c='black');ax.text(*R,' X65 rail node',fontsize=8)
ax.scatter(*J,c='black');ax.text(*J,' J (virtual)',fontsize=8)
ax.scatter(*joint,c='black')
ax.set(xlim=(35,450),ylim=(-270,-130),zlim=(20,365),xlabel='X [mm], FRONT +X',ylabel='Y [mm]',zlabel='Z [mm]',title='Selected 20x20 route + historical A hardware envelopes at 0 deg')
ax.set_box_aspect((415,140,345));ax.view_init(elev=23,azim=-65)
ax.set_yticks([-260,-220,-180,-140])

ax=fig.add_subplot(2,2,2)
ax.plot([258,258,390],[52,120,245],c='.6',lw=12,label='Arch centreline Y-228')
ax.plot([65,372],[60,232],'--',c='#996257',lw=3,label='High:351.9mm /29.3deg, Y-220')
ax.plot([65,320],[60,185],':',c='.4',lw=2,label='Original low:284.0mm /26.1deg, Y-220')
ax.plot([65,320],[60,190],c='#44715a',lw=5,label='Selected:286.7mm /27.0deg, Y-220 to-204')
ax.axvline(225,c='.5',ls=':');ax.text(227,230,'basket X225',rotation=90)
ax.plot([35,330],[52,52],c='.4',lw=4)
for a in [-10,0,20]:
    t=np.radians(a);n=np.array([387.,51.837363636])+[30*np.cos(t)-110*np.sin(t),30*np.sin(t)+110*np.cos(t)]
    ax.plot([260,n[0]],[220,n[1]],c='#426c86',alpha=.65)
ax.scatter(joint[0],joint[2],c='black');ax.text(joint[0]+3,joint[2]-12,'asymmetric splice')
ax.set(xlim=(35,440),ylim=(35,270),xlabel='X [mm]',ylabel='Z [mm]',title='XZ projection only: Y offsets are essential');ax.set_aspect('equal');ax.legend(fontsize=8,loc='upper left')

ax=fig.add_subplot(2,2,3)
ax.add_patch(Rectangle((35,22),90,30,fc='.85',ec='.4'))
ax.add_patch(Rectangle((64,22),2,30,fc='white'))
ax.add_patch(Rectangle((45,52),55,18,fc='none',ec='#44715a',ls='--'))
ax.plot([65,65],[16,74],c='.2',lw=3);ax.plot([59,71],[72,72],c='.2',lw=3)
ax.add_patch(Rectangle((59,76),12,60,fc='#cbdcd3',ec='#44715a',alpha=.7))
ax.add_patch(Rectangle((53,11),24,8,fc='#cbdcd3',ec='#44715a',alpha=.7))
pin=R+35*u
ax.plot([R[0],J[0]],[R[2],J[2]],c='.5',ls=':')
ax.scatter(R[0],R[2],c='black');ax.text(68,56,'R is virtual load line, NOT a crossing Y pin',fontsize=8)
ax.scatter(pin[0],pin[2],c='#44715a');ax.text(pin[0]+2,pin[2]+6,'offset clevis pin, node not yet designed',fontsize=8)
ax.text(39,35,'existing rail + sleeve\nD4 probe: EMPTY',fontsize=8)
ax.text(77,115,'top tool D12\nZ76..136',fontsize=8)
ax.text(79,11,'underside wrench reservation',fontsize=8)
ax.set(xlim=(35,135),ylim=(5,140),xlabel='X [mm]',ylabel='Z [mm]',title='X65 node/access: schematic reservations, not candidate geometry');ax.set_aspect('equal')

ax=fig.add_subplot(2,2,4)
ax.add_patch(Rectangle((-70,-10),69.8,20,fc='.75'));ax.add_patch(Rectangle((.2,-10),43.8,20,fc='.75'))
for yy in [-14,10]:ax.add_patch(Rectangle((-30,yy),60,4,fc='#44715a'))
for xx in [-22,-8,8,22]:ax.plot([xx,xx],[-17,17],c='.15',lw=2)
ax.text(-68,22,'long piece: 202.47mm',fontsize=9);ax.text(1,22,'short piece:43.8mm',fontsize=9)
ax.text(-29,-24,'2 x plates 4x20x60; four through-M4\nplate faces touch; shaft ends have0.4mm gap',fontsize=9)
ax.set(xlim=(-75,55),ylim=(-35,35),xlabel='Local distance from splice [mm]',ylabel='Normal to shaft [mm]',title='Asymmetric fishplate splice (20mm Y width, not a wide sleeve)');ax.set_aspect('equal')
fig.suptitle('P2-93g rear brace analysis — no CAD changes, no exact-solid approval of future end nodes',fontsize=13)
fig.savefig(OUT/'routes-and-details.png',dpi=155);plt.close(fig)

strength=json.loads((OUT/'strength.json').read_text())['variants']
nom=strength['selected190_EA1.0_rail2000.0'];sens=[v for k,v in strength.items()if k.startswith('selected190')]
names={'knee_hole':'Knee hole','pin_thread_8_8':'M4 pins (8.8 assumed)','foot_cheek_bearing':'Foot cheek bearing','foot_pad_pitch':'Foot pad pitch','foot_root_roll':'Foot root roll','foot_root_biaxial_sensitivity':'Foot root biaxial sensitivity','PETG_rail_OD12':'Rail OD12 washer','PETG_inboard_OD9':'Inboard OD9 washer','foot_bridge_torsion':'Foot bridge torsion','splice_two_4mm_fishplates_bearing':'Fishplate M4 bearing','R65_OD12_washer':'New R65 OD12 washer'}
rows=[]
for key,label in names.items():
    rows.append([label]+[f'{nom[lc]["minimum_margins"][key]:.3f}'for lc in ['LC1','LC2']]+[f'{min(v[lc]["minimum_margins"][key]for v in sens):.3f}'for lc in ['LC1','LC2']])
fig,ax=plt.subplots(figsize=(12,6),layout='constrained');ax.axis('off')
table=ax.table(cellText=rows,colLabels=['Original foot / selected brace','LC1 nominal','LC2 nominal','LC1 min9','LC2 min9'],loc='center',cellLoc='center',colWidths=[.43,.14,.14,.14,.14]);table.auto_set_font_size(False);table.set_fontsize(10);table.scale(1,1.8)
for (i,j),cell in table.get_celld().items():
    cell.set_edgecolor('.8')
    if i==0:cell.set_facecolor('.9')
    elif j>0 and float(rows[i-1][j])<1:cell.set_facecolor('#ead8d3')
ax.set_title('Margin = limit/stress; target1.0. Original foot is NOT demonstrated PASS.\nNominal EA100%, rail2000N/mm; min9:EA25/50/100% x rail200/1000/2000N/mm',fontsize=11)
fig.supxlabel('Source: rear-brace-strength.py; LC1 vs LC2 exclusive; PETG10/5MPa; cradle0.600kg + brace0.175kg/side; no physical proof.',fontsize=9)
fig.savefig(OUT/'strength-table.png',dpi=155);plt.close(fig)
