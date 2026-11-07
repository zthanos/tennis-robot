#!/usr/bin/env python3
"""Planar elastic load-sharing screen, corrected LC1/LC2, original foot.
New rail node and joint stiffness are explicit sensitivity inputs, not rigid-
triangle equal sharing. This script does NOT edit CAD or print STL.
"""
import json
from pathlib import Path
from math import pi, sin,cos,radians,hypot,sqrt
import numpy as np

DEST=Path(__file__).parent/'p293g-rear-brace-results'
P=np.array([387.,51.837363636]); B=np.array([260.,220.]); C=np.array([390.,245.])
E=1800.; A=20*(26-4.6); I=20*(26**3-4.6**3)/12

def solve(x,z,lc,a,sign,eff=1.,spring=2000.):
    # Actual upper member centreline to port point via near-rigid small adapter.
    j=np.array([x,120+(x-258)*125/132]); port=np.array([x,z])
    nodes=np.array([[258,97],[258,114],[258,120],j,C,B,[65,60],port],float)
    nd=24; kk=np.zeros((nd,nd)); elements=[]
    def beam(i,j,area,inertia,label):
        dx,dz=nodes[j]-nodes[i];l=hypot(dx,dz);c=dx/l;s=dz/l
        t=np.array([[c,s,0,0,0,0],[-s,c,0,0,0,0],[0,0,1,0,0,0],
                    [0,0,0,c,s,0],[0,0,0,-s,c,0],[0,0,0,0,0,1.]])
        u=E*area/l;v=E*inertia/l**3
        kl=np.array([[u,0,0,-u,0,0],[0,12*v,6*l*v,0,-12*v,6*l*v],
          [0,6*l*v,4*l*l*v,0,-6*l*v,2*l*l*v],[-u,0,0,u,0,0],
          [0,-12*v,-6*l*v,0,12*v,-6*l*v],[0,6*l*v,2*l*l*v,0,-6*l*v,4*l*l*v]])
        ids=np.array([3*i,3*i+1,3*i+2,3*j,3*j+1,3*j+2]);kk[np.ix_(ids,ids)]+=t.T@kl@t
        elements.append((label,ids,t,kl))
    for i,j,label in [(0,1,'lower_hole'),(1,2,'upper_hole'),(2,3,'incline'),(3,4,'upper')]:beam(i,j,A,I,label)
    beam(2,5,20*40,20*40**3/12,'console')
    beam(3,7,A*1e4,I*1e4,'adapter_rigid_idealisation')
    dv=nodes[7]-nodes[6]; length=np.linalg.norm(dv);unit=dv/length
    dy=16. # upper Y=-204, rail Y=-220, mirrored other side
    length3=hypot(length,dy)
    ids=np.array([18,19,21,22]);tv=np.r_[-unit,unit]
    ka=E*(20*(20-4.6))*eff/length*(length/length3)**3
    kk[np.ix_(ids,ids)]+=ka*np.outer(tv,tv)
    # Finite independent rail support; R rotation unused. D is ideal fixed.
    kk[18,18]+=spring;kk[19,19]+=spring
    angle=radians(a);N=P+[30*cos(angle)-110*sin(angle),30*sin(angle)+110*cos(angle)]
    u=(N-B)/np.linalg.norm(N-B);r=N-P;h=abs(r[0]*u[1]-r[1]*u[0])
    act=sign*8000/h*u if lc=='LC1' else np.zeros(2)
    stop=np.array([0.,sign*8000/132]) if lc=='LC2' else np.zeros(2)
    gravity=np.array([0.,-.6*9.81]); brace_weight=.175*9.81 # solid20x20 + provisional sleeve/end mass
    f=np.zeros(nd);f[15:17]=act+gravity;f[17]=-40*.6*9.81
    f[12:14]=stop
    if lc=='LC2_pure_couple_sensitivity':f[14]=sign*8000
    f[19]-=brace_weight/2;f[22]-=brace_weight/2
    fixed=[0,1,2,20];free=[i for i in range(nd) if i not in fixed]
    disp=np.zeros(nd);disp[free]=np.linalg.solve(kk[np.ix_(free,free)],f[free])
    rr=kk@disp-f
    fx,fz,md=rr[:3]
    q=hypot(abs(md)/34+abs(fx)/2,abs(fz)/2)
    brace_force=ka*(tv@disp[ids]); # positive tension, force at upper node=-T*unit
    bf=-brace_force*unit
    mp=md-45*fx # reaction moment translated DOWN to Z52
    # Account for line-of-action Y offset statically (lateral stiffness is
    # not solved). At upper portY-204, Fy=-projectedT*16/Lxz.
    bfy=-brace_force*dy/length
    mr=-(act[1]*55+stop[1]*78-.6*9.81*25+bf[1]*54+(z-52)*bfy-brace_weight/2*54)
    mz=-(act[0]*55-gravity[0]*25+bf[0]*54+(x-258)*bfy)
    uplift=max(0,-fz)
    rail=abs(mp)/34+abs(mr)/70+uplift
    inboard=abs(mr)/78+uplift
    ann=lambda od:pi*(od*od-4.3**2)/4
    margins={
      'pin_thread_8_8':580/hypot(q*4.2/(pi*3.14**3/32),sqrt(3)*q/(2*8.78)),
      'foot_cheek_bearing':5/(q/48),'foot_cheek_edge':5/(q/(4*6*3.7)),
      'knee_pin_bearing':5/(q/80),
      'foot_pad_pitch':10/(1.5*abs(mp)/(23.4*18**2/6)),
      'foot_root_roll':10/(1.8*abs(mr)/(19.4*20**2/6)),
      'foot_root_biaxial_sensitivity':10/(1.8*(abs(mr)/(19.4*20**2/6)+abs(mz)/(20*19.4**2/6))),
      'base_M4_8_8':580/hypot(rail/8.78,sqrt(3)*abs(fx)/8.78),
      'PETG_rail_OD12':5/((rail+100)/ann(12)),
      'PETG_inboard_OD9':5/((inboard+100)/ann(9)),
      'PETG_pitch_contact':5/(rail/96),
    }
    endforces={}
    for name,ii,t,kl in elements:
        end=kl@t@disp[ii];endforces[name]=end.tolist()
    # Real upper knee bore H114; no blanket Kt on plain stock.
    e=endforces['upper_hole']
    margins['knee_hole']=10/(1.8*(abs(e[2])/(I/13)+abs(e[0])/A))
    margins['knee_plain_lower']=10/(abs(md)/(20*26**2/6)+abs(fz)/(20*26))
    # Residual root torsion, signed transport through real tapered bridge.
    torsion=[]
    for tt in np.linspace(0,39/47,81):
        xx=240+18*tt;zz=70-8*tt
        torque=md-(97-zz)*fx+(258-xx)*fz
        b,hh=24+2*tt,36-16*tt;small,large=sorted([b,hh])
        torsion.append(abs(torque)/(.2*large*small**2))
    margins['foot_bridge_torsion']=5/max(torsion)
    # Proposed new stock/end detail: solid20x20 with two M4 per end, pitch20.
    ab=abs(brace_force);In=20*(20**3-4.6**3)/12
    margins['brace_axial']=5/(ab/(20*(20-4.6)))
    margins['brace_Euler_pin_pin']=pi*pi*E*In/length**2/ab
    margins['splice_two_M4_bearing']=5/(ab/(2*20*4))
    margins['splice_two_4mm_fishplates_bearing']=5/(ab/(2*2*4*4))
    margins['node_two_M4_two_6mm_cheeks_bearing']=5/(ab/(2*2*6*4))
    # Pad axisZ60, soleZ52, 20mm compression arm; conservative one-bolt uplift.
    rforce=np.array([-spring*disp[18],-spring*disp[19]])
    # Independent global equilibrium: applied forces/couples + both supports.
    applied=f.reshape(-1,3)
    assert np.linalg.norm(applied[:,:2].sum(axis=0)+rr[:2]+rforce)<1e-3
    global_m=sum(nodes[i,0]*v[1]-nodes[i,1]*v[0]+v[2] for i,v in enumerate(applied))
    global_m+=nodes[0,0]*fz-nodes[0,1]*fx+md+nodes[6,0]*rforce[1]-nodes[6,1]*rforce[0]
    assert abs(global_m)<.1
    anchor_t=max(0,-rforce[1])+abs(rforce[0])*8/20
    margins['R65_OD12_washer']=5/((anchor_t+100)/ann(12))
    margins['R65_sleeve_web17_bearing']=5/(abs(rforce[0])/(17*4))
    margins['R65_rail_two_skins_bearing_sensitivity']=5/(abs(rforce[0])/(2*3.2*4))
    return {'angle':a,'sign':sign,'margins':margins,'D_reaction_N_Nmm':rr[:3].tolist(),
            'R_reaction_N':rforce.tolist(),'brace_axial_N':brace_force*length3/length,'root_roll_Nmm':mr,
            'upper_port_force_N':bf.tolist(),'port_displacement_XZ_mm':disp[21:23].tolist()}

def main():
    DEST.mkdir(exist_ok=True);variants={}
    for label,x,z in [('high',372,232),('low',320,185),('selected190',320,190)]:
        profiles=[(e,k) for e in (1.,.5,.25) for k in (2000.,1000.,200.)] if label=='selected190' else [(1.,2000.),(.5,1000.),(.25,200.)]
        for eff,spring in profiles:
            key=f'{label}_EA{eff}_rail{spring}'
            cases={lc:[solve(x,z,lc,float(a),s,eff,spring) for a in np.linspace(-10,20,61) for s in (-1,1)] for lc in ['LC1','LC2','LC2_pure_couple_sensitivity']}
            variants[key]={lc:{'minimum_margins':{k:min(v['margins'][k] for v in vv) for k in vv[0]['margins']},
                               'max_abs_brace_N':max(abs(v['brace_axial_N']) for v in vv),
                               'max_abs_R65_Fx_N':max(abs(v['R_reaction_N'][0]) for v in vv)} for lc,vv in cases.items()}
            print(key,variants[key],flush=True)
    (DEST/'strength.json').write_text(json.dumps({'status':'PLANAR_SCREEN_NOT_RELEASE','variants':variants,
       'assumptions':['D fixed, rail R finite springs200..2000N/mm, splice EA25..100percent',
         'port Y204 to railY220: axial stiffness projection and static Fy moments included; lateral 3D stiffness not solved',
         'new brace solid20x20 plus provisional total0.175kg/side; cradle0.600kg',
         'original foot6mm cheeks,width28,OD12/OD9,all original hole positions',
         'R65 skin-only bearing and internal-web bearing shown separately; no unstated equal load sharing',
         'local node/rail support/contact/ports and bolt pull-through require coupon/solid validation']},indent=2)+'\n')

if __name__=='__main__':main()
