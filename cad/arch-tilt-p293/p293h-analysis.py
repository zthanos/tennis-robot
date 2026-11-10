#!/usr/bin/env python3
"""PARKED P2-93h reference analysis: force/contact layout, no CAD solids.

Reuses the P2-93g section model in memory; writes only analysis JSON/PNG/MD.
The stop model is historical/reference input for work after the real moving
module exists; it is not a gate PASS, contact FEA, or authorization to design.
"""
import json,math,sys,hashlib
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
H=Path(__file__).resolve().parent;O=H/'p293h-analysis';O.mkdir(exist_ok=True)
source=(H/'p293g-solid-screen.py').read_text().split('\nenvelopes={}')[0]
source=source.replace("ap=argparse.ArgumentParser();ap.add_argument('--tag',default='candidate-03');args=ap.parse_args()", "args=argparse.Namespace(tag='final')")
source=source.replace("S=np.array([0.,0.,s*8000/132]) if lc=='LC2' else np.zeros(3)","S=STOP_FORCE if lc=='LC2' else np.zeros(3)")
source=source.replace("np.array([390.,-221.6,245.]),S", "STOP_POINT,S")
source=source.replace("cq=abs(cm[1])/20+np.linalg.norm(cf)/2", "cq=np.linalg.norm(cm)/20+np.linalg.norm(cf)/2")
ns={'__file__':str(H/'p293g-solid-screen.py')};exec(compile(source,'P293g-screen-in-memory-contact-variant','exec'),ns)
# 200g fixed-console budget, deliberately charged wholly to the stop side.
# G-model halves arch self-weight: doubling this row retains that full200g.
ns['mass']['p293d_h_stop_allowance']=dict(mass_g=400.,CM_mm=[390,-221.6,245])
P=np.array([387.,0.,51.837363636]);local=np.array([20.,185.,185.]);radius=math.hypot(local[0],local[2])
contacts=[];cases=[]
for a in [-10,20]:
 t=math.radians(a);r=np.array([20*math.cos(t)-185*math.sin(t),185.,20*math.sin(t)+185*math.cos(t)])
 c=P+r;tangent=np.array([-r[2],0,r[0]])/radius
 contacts.append(dict(angle=a,point_mm=c.tolist(),normal=tangent.tolist(),radius_mm=radius,force_N=8000/radius,
                      trigger_angle_deg=a+(3 if a<0 else -3),trigger_arc_mm=radius*math.radians(3)))
 for sign in [-1,1]:
  ns['STOP_POINT']=c*np.array([1,-1,1]);ns['STOP_FORCE']=sign*8000/radius*tangent
  cases.append(ns['evaluate']('LC2',a,sign))
env={k:{**(w:=max(cases,key=lambda c:c['rows'][k]['stress_MPa']))['rows'][k], 'angle':w['angle_deg'],'sign':w['sign']} for k in cases[0]['rows']}
q_axial_bound=max(c['rows']['crown_inboard_lug_bearing']['stress_MPa']*48 for c in cases)
pressure=(q_axial_bound+100)/(math.pi*(9**2-4.6**2)/4)
env['PETG_crown_washer']=dict(stress_MPa=pressure,limit_MPa=5,margin=5/pressure,
 detail='FULL Q as conservative axial/prying bound +100N preload, actualOD9 PETG annulus; NOT preload-only')
lc1cases=[ns['evaluate']('LC1',float(a),s) for a in np.linspace(-10,20,301) for s in [-1,1]]
lc1env={k:{**(w:=max(lc1cases,key=lambda c:c['rows'][k]['stress_MPa']))['rows'][k], 'angle':w['angle_deg'],'sign':w['sign']} for k in lc1cases[0]['rows']}
F=8000/radius
rootZ=12*(40-4.6)**2/6
striker_stress=1.8*(F*138/rootZ+F*19/(40*12**2/6)+F/(12*(40-4.6)))
local_checks={
 'striker_40x12_net_M4_root':dict(stress_MPa=striker_stress,limit_MPa=10,margin=10/striker_stress),
 'striker_torsion':dict(stress_MPa=1.8*F*19/(.2*40*12**2),limit_MPa=5,margin=5/(1.8*F*19/(.2*40*12**2))),
 'console_20x24_net_root':dict(stress_MPa=1.8*(F*50/(20*(24-4.6)**2/6)+F/(20*(24-4.6))),limit_MPa=10),
 'pad_effective_12x2_CONDITIONAL':dict(stress_MPa=F/24,limit_MPa=5,condition='coupon must demonstrate >=24mm2 backed contact, NOT assumed Hertz proof'),
 'striker_mount_OD12':dict(stress_MPa=(F*138/40+100)/(math.pi*(12**2-4.6**2)/4),limit_MPa=5),
 'crown_seat_20x20':dict(stress_MPa=F/400,limit_MPa=5),
 'larger_crown_OD12_seat_axial_bound':dict(stress_MPa=(q_axial_bound+100)/(math.pi*(12**2-4.6**2)/4),limit_MPa=5,
                                         condition='requires actual stiffOD12 washer/seat; not a flexible foil assumption'),
}
for row in local_checks.values():row['margin']=row['limit_MPa']/row['stress_MPa']
old=json.loads((H/'p293g-candidate-results/solid-screen.json').read_text())
hardware={}
for lc in ['LC1','LC2']:
 e=old['envelopes'][lc]
 hardware[lc]=dict(crown_thread_8_8=e['crown_pin_thread_10_9']['margin']*580/830,
                   foot_thread_8_8=e['M4_Z80_114_thread_8_8']['margin'],
                   deeper_hex_recess_cheek=e['foot_cheek_bearing']['margin']*4.4/5,
                   crown_DIN433_OD8=e['PETG_crown_washer']['margin']*(8**2-4.6**2)/(9**2-4.6**2))
hardware['proposed_actual_contact_LC2_crown_8_8']=env['crown_pin_thread_10_9']['margin']*580/830
smooth=[]
for case in cases:
 q=case['rows']['crown_inboard_lug_bearing']['stress_MPa']*48
 # Measured smooth d>=3.8 at root. Add FULL q/At as conservative axial bound.
 stress=math.hypot(q*16.4/(math.pi*3.8**3/32)+q/8.78,math.sqrt(3)*q/(math.pi*3.8**2/4))
 smooth.append(dict(angle=case['angle_deg'],sign=case['sign'],stress_MPa=stress,margin=580/stress))
hardware['crown_partial_thread_8_8_d3_8_axial_bound']=min(smooth,key=lambda x:x['margin'])
hardware['cut_stock_sleeve_min_L4_95_LC1']=old['envelopes']['LC1']['foot_cheek_bearing']['margin']*4.95/5
hardware['cut_stock_sleeve_min_L4_95_LC1_with_stop_budget']=lc1env['foot_cheek_bearing']['margin']*4.95/5
hardware['cut_stock_sleeve_min_L4_97_LC1_with_stop_budget']=lc1env['foot_cheek_bearing']['margin']*4.97/5
stls=[];totalg=totallo=totalhi=0
for name,row in old['print_STL_sanity'].items():
 qty=2 if any(s in name for s in ['_foot.','_crown.','_crossbar_end.']) else 1
 volume=row['volume_mm3'];g=volume*.00127
 # Planning only: effective deposited volume rate 4..7mm3/s +0.20h/setup.
 lo=volume/7/3600+.20;hi=volume/4/3600+.20
 stls.append(dict(STL=name,quantity=qty,solid_g_each=g,h_each_low=lo,h_each_high=hi,
                  all_solid=True,source='mesh volume; time NOT sliced',supports_purge_brim_excluded=True))
 totalg+=qty*g;totallo+=qty*lo;totalhi+=qty*hi
data=dict(status='PARKED_DEPENDS_ON_REAL_MOVING_MODULE_ANALYSIS_ONLY',contacts=contacts,LC1_with_fixed_budget=lc1env,actual_contact_LC2_screen=env,
          local_stop_screen=local_checks,hardware=hardware,print_planning=stls,
          print_total=dict(g=totalg,h_low=totallo,h_high=totalhi,physical_parts=sum(r['quantity'] for r in stls)),
          limitations=['Existing arch sections exact; proposed stop sections nominal, no mesh verification.',
                       'Both signs checked at each endpoint, no Tr8 force in LC2;200g fixed console budget.',
                       'Global foot support model remains P293g sectional screen, not contact FEA.',
                       'New stop Y-offset/axial pin loads require candidate joint verification.',
                       'Pad24mm2 is conditional finite contact, not a verified first-contact footprint.',
                       '8Nm is static load bound, NOT arbitrary impact/stall qualification.'],
          source_screen_SHA256=hashlib.sha256((H/'p293g-solid-screen.py').read_bytes()).hexdigest())
(O/'preflight.json').write_text(json.dumps(data,indent=2)+'\n')
fig,axs=plt.subplots(1,2,figsize=(13,6))
ax=axs[0];ax.plot([258,258,390],[52,120,245],color='.45',lw=9,label='Existing knee/crown, Y221.6')
angles=np.linspace(-10,20,100);arc=[]
for a in angles:
 t=np.radians(a);arc.append([387+20*np.cos(t)-185*np.sin(t),P[2]+20*np.sin(t)+185*np.cos(t)])
arc=np.array(arc);ax.plot(arc[:,0],arc[:,1],'--',color='#267e56',label='Proposed contact arc, Y185')
for c in contacts:
 p=c['point_mm'];ax.plot([387,p[0]],[P[2],p[2]],color='#267e56',alpha=.6)
 ax.plot([390,p[0]],[245,p[2]],color='#26659a',lw=3)
 ax.annotate(f"{c['angle']} deg / {c['force_N']:.1f} N",(p[0],p[2]),xytext=(0,-22),textcoords='offset points',ha='center')
ax.scatter([387],[P[2]],color='black');ax.set(xlabel='X [mm]',ylabel='Z [mm]',title='P2-93h CONTACT LAYOUT ONLY / not candidate solids',aspect='equal');ax.legend(fontsize=8)
ax=axs[1];names=['Foot M4 8.8','Crown M4 8.8 (old LC2)','OD8 DIN433 crown washer','Foot deeper hex recess']
values=[hardware['LC1']['foot_thread_8_8'],hardware['LC2']['crown_thread_8_8'],hardware['LC2']['crown_DIN433_OD8'],hardware['LC1']['deeper_hex_recess_cheek']]
ax.barh(names,values,color=['#267e56' if v>=1 else '#a54436' for v in values]);ax.axvline(1,color='black',ls='--');ax.set(xlabel='Screening margin [limit / stress]',title='Drop-in hardware alternatives, P2-93g baseline')
for i,v in enumerate(values):ax.text(v+.02,i,f'{v:.3f}',va='center')
fig.tight_layout();fig.savefig(O/'contact-and-hardware.png',dpi=170);plt.close(fig)
fig,axs=plt.subplots(1,2,figsize=(12,5))
ax=axs[0];phi=np.linspace(0,2*np.pi,300)
ax.plot(442+100*np.cos(phi),129+100*np.sin(phi),color='.5',label='Nominal moving flywheel R100 at0 deg')
for x0,y0,w,h,color,label in [(373,214.8,28,13.2,'#a54436','Inboard rectangular envelope: rejected'),(373,240.3,28,13.2,'#26659a','Historical outboard envelope: Y-limit FAIL')]:
 ax.add_patch(plt.Rectangle((x0,y0),w,h,fill=False,color=color,label=label))
ax.axhline(240,color='black',ls='--',label='Y limit240');ax.scatter([400],[216],color='#a54436');ax.set(xlim=(360,455),ylim=(185,260),xlabel='X [mm]',ylabel='Y [mm]',title='AS5600 packaging constraints at pivot Z51.837');ax.legend(fontsize=7)
ax=axs[1]
ax.add_patch(plt.Rectangle((0,-5),6,10,fill=False,color='.4',label='Proposed moving end boss (section only)'))
ax.add_patch(plt.Rectangle((3.4,-3.1),2.6,6.2,facecolor='white',edgecolor='black'))
ax.add_patch(plt.Rectangle((3.5,-3),2.5,6,facecolor='#b08a48',label='Diametric magnet6x2.5'))
ax.add_patch(plt.Rectangle((7.5,-4),1.5,8,facecolor='#267e56',label='Sensor package, chip centred on axis'))
ax.plot([-1,10],[0,0],'k--');ax.annotate('Closed floor >=1.2mm; no through bore',(2,0),(0,-8),arrowprops={'arrowstyle':'->'},fontsize=8)
ax.annotate('Face gap1.5mm',(6.75,3),(5,8),arrowprops={'arrowstyle':'->'},fontsize=8)
ax.set(xlim=(-1,11),ylim=(-10,11),xlabel='Axial section coordinate [mm]',ylabel='Radial coordinate [mm]',title='Blind magnetic seat topology, NOT CAD');ax.legend(fontsize=7,loc='lower right')
fig.tight_layout();fig.savefig(O/'encoder-constraints.png',dpi=170);plt.close(fig)
lines=['# P2-93h — PARKED generated reference tables','',
       '**PARKED — depends on the real moving module.** ANALYSIS ONLY; proposed contact/sections, NOT a new-solid gate PASS or authorization to design/print/order.','',
       'For future screens using the existing 8 N·m and 10/5 MPa bounds, margin >=1.0 is PASS. Do not add a higher target without evidence; borderline cases go to physical test.','',
       '| Existing element | LC1 with200g fixed budget | Proposed actual-contact LC2 margin |','|---|---:|---:|']
for k,v in env.items():lines.append(f"| {k} | {lc1env[k]['margin']:.3f} | {v['margin']:.3f} |")
lines+=['','| Proposed local section | Margin |','|---|---:|']
for k,v in local_checks.items():lines.append(f"| {k} | {v['margin']:.3f} |")
lines+=['',f"Crown partial-thread8.8 alternative: measured smooth grip>=22mm, d>=3.8, combined bending/shear +FULL q/8.78 axial bound: margin {hardware['crown_partial_thread_8_8_d3_8_axial_bound']['margin']:.3f}. NOT a generic fully-threaded8.8 swap."]
lines+=['','## Print planning (slicing failed; no G-code estimate)','','All nine models100% solid for conservative procurement; volumes exclude supports/purge/brim. Effective4–7mm³/s plus12min per separate print, NOT printer prediction.','','| STL | Qty | Solid g/each | Planning h/each |','|---|---:|---:|---:|']
for r in stls:lines.append(f"| {r['STL']} | {r['quantity']} | {r['solid_g_each']:.1f} | {r['h_each_low']:.2f}–{r['h_each_high']:.2f} |")
lines += ['',f'Total {totalg:.1f}g, {totallo:.2f}–{totalhi:.2f}h / {sum(r["quantity"] for r in stls)} separate prints; consumables/supports additional.']
(H/'P2-93h-tables.md').write_text('\n'.join(lines)+'\n')
print('Proposed LC2 min',min((v['margin'],k) for k,v in env.items()))
print('Hardware:',hardware);print('Printing:',data['print_total']);print('Contacts:',contacts)
