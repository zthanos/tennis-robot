#!/usr/bin/env python3
"""Scientific load-path diagrams, not candidate CAD or assembly drawings."""
from pathlib import Path
import json
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
dest=Path(__file__).parent/'p293g-results'
d=json.loads((dest/'preflight.json').read_text())
nodes=d['inputs']['nodes_XZ_mm']
fig,axs=plt.subplots(1,3,figsize=(14,5),layout='constrained')
for ax,mode,title in zip(axs,['cantilever','crown_only','rear_triangle'],
 ['1. Existing single-root path','2. Crown sharing, same root','3. Independent X170 rear triangle']):
    for i,j in [(0,1),(1,2),(2,3)]:
        ax.plot([nodes[i][0],nodes[j][0]],[nodes[i][1],nodes[j][1]],c='.35',lw=7)
    ax.plot([258,260],[120,220],c='#276789',lw=4,label='Proposed console')
    edges=[]
    if mode!='cantilever':edges+=[(4,2)]
    if mode=='rear_triangle':edges+=[(4,5),(2,5)]
    for i,j in edges:
        ax.plot([nodes[i][0],nodes[j][0]],[nodes[i][1],nodes[j][1]],c='#276789',lw=2,ls='--')
    for (x,z),name in zip(nodes,['D','K','S','C','B','R']):
        ax.plot(x,z,'o',c='black',ms=4); ax.annotate(name,(x,z),xytext=(5,4),textcoords='offset points')
    ax.axhline(52,color='.6',lw=2); ax.axvspan(280,320,ymin=0,ymax=.22,color='.8',alpha=.5)
    ax.set(xlim=(150,410),ylim=(25,275),xlabel='X [mm]',ylabel='Z [mm]',title=title,aspect='equal')
    margins=d['modes'][mode]['min_margins']
    ax.text(.02,.97,'knee %.2f | foot cheek %.2f\nfoot pad %.2f | PETG contact %.2f'%tuple(margins[k]['value'] for k in
       ['knee_lower','foot_cheek_bearing','foot_pad_pitch','PETG_pitch_contact']),transform=ax.transAxes,va='top',fontsize=9)
fig.suptitle('P2-93g preflight: load paths only — NONE qualified to target 1.5',fontsize=14)
fig.supxlabel('Source: unchanged P2-93e + planar screening; 8 Nm + ±100 N + assumed 0.600 kg. Dashed members are NOT CAD solids.',fontsize=9)
fig.savefig(dest/'load-paths.png',dpi=160); plt.close(fig)

fig,ax=plt.subplots(figsize=(9,4),layout='constrained')
# Plan-view hole map, deliberately schematic: no replacement printable template.
for x,y,label in [(240,-150,'X240 / Y-150'),(240,-160,'X240 / Y-160'),(230,-220,'X230 / Y-220'),(170,-220,'X170 / Y-220')]:
    ax.plot(x,y,'o',c='#276789' if x==170 else '.2',ms=8)
    ax.annotate(label,(x,y),xytext=(8,5),textcoords='offset points',fontsize=9)
ax.plot([192,271,271,192,192],[-234,-234,-206,-206,-234],c='.5',ls='--',label='Existing rail-pad span (schematic)')
ax.plot([170,170],[-245,-195],c='.5',ls=':',label='Side Z37 bore: NOT through -Y rail')
ax.set(xlim=(150,295),ylim=(-250,-125),xlabel='X [mm]; FRONT +X →',ylabel='Y [mm]',aspect='equal',title='Existing vertical holes — current template covers the three right-hand holes')
ax.legend(loc='upper left',fontsize=8)
fig.savefig(dest/'hole-map.png',dpi=160); plt.close(fig)
