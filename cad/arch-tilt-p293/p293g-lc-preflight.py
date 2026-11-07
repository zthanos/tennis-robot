#!/usr/bin/env python3
"""Corrected P2-93g: mutually exclusive LC1/LC2. Analysis only, no CAD writes."""
import json
from pathlib import Path
from math import sin, cos, radians, hypot, pi, sqrt, ceil
import numpy as np

DEST=Path(__file__).parent/'p293g-results'
P=np.array([387.,51.837363636]); B=np.array([260.,220.]); C=np.array([390.,245.])
MASS=.600; G=MASS*9.81; PRELOAD=100.

def evaluate(lc,angle,sign,cheek=6.,pad_w=28.,root_w=24.,root_h=20.,washer=12.):
    a=radians(angle)
    N=P+np.array([30*cos(a)-110*sin(a),30*sin(a)+110*cos(a)])
    u=(N-B)/np.linalg.norm(N-B)
    r=N-P; h=abs(r[0]*u[1]-r[1]*u[0])
    act=sign*8000/h*u if lc=='LC1' else np.zeros(2)
    stop=np.array([0.,sign*8000/132]) if lc=='LC2' else np.zeros(2)
    assert not (np.linalg.norm(act)>0 and np.linalg.norm(stop)>0)
    if lc=='LC1':assert abs(np.linalg.norm(act)*h-8000)<1e-8
    loads=[(B,act),(C,stop),(np.array([300.,245.]),np.array([0.,-G]))]
    def resultant(origin):
        f=sum((f for _,f in loads),np.zeros(2))
        m=sum((p[0]-origin[0])*f[1]-(p[1]-origin[1])*f[0] for p,f in loads)
        return f,float(m)
    f,md=resultant(np.array([258.,97.])); fx,fz=f
    # Signed equilibrium translated to Z52; do not add moments a second time.
    _,mp=resultant(np.array([258.,52.]))
    # Base roll about the inboard Y150 datum, not mixed Y150/Y228 datums.
    mr=act[1]*55+stop[1]*78-G*25
    # 34mm pin couple Fx and direct Fx add at the most loaded pin.
    q=hypot(abs(md)/34+abs(fx)/2,abs(fz)/2)
    span=20.8+cheek
    pin_m=q*(2*span-20)/8
    As=8.78
    def pin_stress(d):return hypot(pin_m/(pi*d**3/32),sqrt(3)*q/(2*As))
    # Contact model retained, no friction/equal sharing credit; uplift envelope.
    uplift=max(0,fz)
    rail_force=abs(mp)/34+abs(mr)/70+uplift
    inboard_force=abs(mr)/78+uplift
    ann=lambda od:pi*(od**2-4.3**2)/4
    rows={}
    def add(key,stress,limit,detail):
        rows[key]={'stress_MPa':float(stress),'limit_MPa':limit,'margin':float(limit/stress) if stress else 1e30,'detail':detail}
    # Actual pin bore at Z114: net section and notch Kt; plain stock at D97: Kt1.
    _,m114=resultant(np.array([258.,114.]))
    znet=(20*(26**3-4.6**3)/12)/13
    hole_stress=1.8*(abs(m114)/znet+abs(fz)/(20*(26-4.6)))
    plain_stress=abs(md)/(20*26**2/6)+abs(fz)/(20*26)
    add('knee',max(hole_stress,plain_stress),10,'Kt1.8 ONLY at actual Z114 bore; plain stem Kt1')
    add('pin_thread_8_8',pin_stress(3.14),580,'conditional class8.8, threaded root; bending + double shear')
    add('pin_smooth_8_8',pin_stress(4.),580,'conditional class8.8, smooth full grip; bending + double shear')
    add('knee_pin_bearing',q/(20*4),5,'actual bolt diameter4, knee width20')
    add('foot_cheek_bearing',q/(2*cheek*4),5,'actual bolt4; no preload')
    add('foot_cheek_edge',q/(4*cheek*(6-2.3)),5,'actual edge ligament3.7; no preload')
    add('foot_pad_pitch',1.5*abs(mp)/((pad_w-4.6)*18**2/6),10,'Kt1.5 at real X230 through bore; grip height18 unchanged')
    add('foot_root_roll',1.8*abs(mr)/((root_w-4.6)*root_h**2/6),10,'Kt1.8 at inboard actual bore, same net-root proxy as prior screen')
    # Real hull bridge: source cubes end Y167 and begin Y214. Its minimum
    # width and minimum height do NOT occur at the same section. No invented
    # bore/net-section or Kt on this bore-free interval. Rear rail pad begins
    # Y206; union only increases sections from there. At Y167..206 the first
    # hull gives the following actual rectangular sections.
    bridge_stress=[]
    for t in np.linspace(0,39/47,81):
        b=24+2*t; hh=36-16*t; short,long=sorted([b,hh])
        _,torque=resultant(np.array([240+18*t,70-8*t]))
        bridge_stress.append(abs(torque)/(.2*long*short**2))
    add('foot_bridge_torsion',max(bridge_stress),5,'actual bore-free hull Y167..206; Kt1, rectangular torsion modulus0.2*long*short^2')
    add('base_M4_8_8',hypot(rail_force/As,sqrt(3)*abs(fx)/As),580,'external load only: preload excluded per corrected contract')
    add('PETG_rail_washer',(rail_force+PRELOAD)/ann(washer),5,'ONLY washer checks include residual preload100N')
    add('PETG_inboard_washer',(inboard_force+PRELOAD)/ann(9),5,'ONLY washer checks include residual preload100N')
    add('PETG_pitch_contact',rail_force/96,5,'contact8x12: NO residual preload')
    add('PETG_reverse_roll_contact',abs(mr)/70/48,5,'contact4x12: NO residual preload')
    # Console stock20Yx40 normal; no invented bore at root. No joint certification.
    # Exclude the hard-stop force, which is not carried by the console.
    mc_console=(B[0]-258)*act[1]-(B[1]-120)*act[0]-(300-258)*G
    add('console_stock',abs(mc_console)/(20*40**2/6)+np.linalg.norm(act+[0,-G])/(20*40),10,'plain20x40 stock Kt1; final mounting bores not yet designed')
    return {'LC':lc,'angle_deg':float(angle),'sign':sign,'h_mm':h,'F_Tr8_N':float(np.linalg.norm(act)),
            'root_D_force_N':f.tolist(),'root_D_moment_Nmm':md,'base_pitch_Nmm':mp,
            'base_roll_Nmm':float(mr),'pin_N':q,'rail_external_N':rail_force,'rows':rows}

def envelope(lc,**kwargs):
    cases=[evaluate(lc,a,s,**kwargs) for a in np.linspace(-10,20,301) for s in (-1,1)]
    return {'checks':{k:{**(w:=max(cases,key=lambda c:c['rows'][k]['stress_MPa']))['rows'][k],
                          'angle_deg':w['angle_deg'],'sign':w['sign']} for k in cases[0]['rows']},
            'F_Tr8_max_N':max(c['F_Tr8_N'] for c in cases),'pin_max_N':max(c['pin_N'] for c in cases),
            'base_pitch_abs_max_Nmm':max(abs(c['base_pitch_Nmm']) for c in cases)}

def main():
    original={lc:envelope(lc) for lc in ['LC1','LC2']}
    qmax=max(v['pin_max_N'] for v in original.values())
    mpmax=max(v['base_pitch_abs_max_Nmm'] for v in original.values())
    washer_force=max(v['checks']['PETG_rail_washer']['stress_MPa']*pi*(12**2-4.3**2)/4 for v in original.values())
    # Local dimensional suggestions are NOT exported geometry and NOT a global
    # minimum-volume optimisation. Outward bosses do not move the bore axes.
    changes=dict(cheek=8.,pad_w=40.,root_w=24.,root_h=20.,washer=14.)
    local={lc:envelope(lc,**changes) for lc in ['LC1','LC2']}
    assert original['LC2']['F_Tr8_max_N']==0
    assert min(c['margin'] for v in original.values() for c in v['checks'].values())<1
    assert min(c['margin'] for v in local.values() for c in v['checks'].values())>=1
    # Regression of the corrected preload contract: ONLY washer seats change.
    global PRELOAD
    with_preload=evaluate('LC1',0,1)
    saved=PRELOAD; PRELOAD=0.
    without_preload=evaluate('LC1',0,1); PRELOAD=saved
    for key,row in with_preload['rows'].items():
        delta=abs(row['stress_MPa']-without_preload['rows'][key]['stress_MPa'])
        assert (delta>0) if key in ('PETG_rail_washer','PETG_inboard_washer') else delta<1e-12
    data={'status':'CORRECTED_LOAD_PREFLIGHT_NOT_RELEASE','target_margin':1.,
          'supersedes':'P2-93g-preflight.md combined load and target1.5',
          'inputs':{'LC1':'F=8000/h(theta) at B, both signs; NO stop; +gravity',
                    'LC2':'one crown stop equivalent force8000/132, both signs; ZERO Tr8; +gravity',
                    'mass_assumed_kg':MASS,'CM_mm':[300,-175,245],
                    'preload_N_only_under_washers':PRELOAD,'Kt':'only actual bore/net-root; no generic stock multiplier'},
          'original':original,'local_sizing_trial':{'dimensions':changes,'envelopes':local},
          'independent_local_minima_mm':{
              'cheek_thickness':qmax/(2*4*5),
              'pad_width_at_fixed_18mm_grip':4.6+1.5*mpmax*6/(10*18**2),
              'rail_washer_OD_at_ID4_3':sqrt(4*washer_force/(pi*5)+4.3**2)},
          'limitations':['Analytical proxies, not 3D solid FEA, collision proof or physical proof.',
             'Bridge torsion uses source hull rectangles; local 3D load introduction/junction stresses not solved.',
             'No probability or printed PETG strength claimed;10/5MPa retained as requested.',
             '0.600kg/CM are assumptions pending final geometry/hardware.',
             'Local sizing trial not implemented; footprints, real sections, hardware grip and collision gate require approval/verification.']}
    DEST.mkdir(exist_ok=True)
    (DEST/'corrected-load-cases.json').write_text(json.dumps(data,indent=2)+'\n')
    for variant,vals in [('original',original),('local_trial',local)]:
        print(variant)
        for lc,v in vals.items():print(lc,'Fmax',round(v['F_Tr8_max_N'],3),'pin',round(v['pin_max_N'],3),
           {k:round(r['margin'],3) for k,r in v['checks'].items()})

if __name__=='__main__':main()
