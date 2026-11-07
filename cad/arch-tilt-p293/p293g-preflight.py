#!/usr/bin/env python3
"""P2-93g read-only geometry preflight. Linear planar beam/truss SCREEN, not FEA release.

HISTORICAL: combined load / target1.5 superseded by p293g-lc-preflight.py.

Outputs analysis files only. No SCAD/STL/QA mutations. N, mm, MPa.
"""
import hashlib
import json
from pathlib import Path
from math import pi, sin, cos, radians, hypot
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).parent / 'p293g-results'
E = 1800.
P = np.array([387., 51.837363636])
B = np.array([260., 220.])
K = np.array([258., 120.])
C = np.array([390., 245.])
D = np.array([258., 97.])
S = C - 55 * (C-K)/np.linalg.norm(C-K)
R = np.array([170., 60.])
nodes = np.array([D, K, S, C, B, R])
area = 20*(26-4.6)
inertia = 20*(26**3-4.6**3)/12
section = inertia/13

def solve(mode, angle, sign, mass):
    """D fixed, R independently fixed only in rear-truss model.
    Explicit full one-side 8Nm crown load PLUS independent actuator bound.
    This is a prescribed conservative combination, not a solved robot FBD.
    """
    stiffness = np.zeros((18,18)); elements = []
    def beam(i,j,a,ii,name):
        dx,dz=nodes[j]-nodes[i]; l=hypot(dx,dz); c=dx/l; s=dz/l
        t=np.array([[c,s,0,0,0,0],[-s,c,0,0,0,0],[0,0,1,0,0,0],
                    [0,0,0,c,s,0],[0,0,0,-s,c,0],[0,0,0,0,0,1.]])
        u=E*a/l; v=E*ii/l**3
        kl=np.array([[u,0,0,-u,0,0],[0,12*v,6*l*v,0,-12*v,6*l*v],
          [0,6*l*v,4*l*l*v,0,-6*l*v,2*l*l*v],[-u,0,0,u,0,0],
          [0,-12*v,-6*l*v,0,12*v,-6*l*v],[0,6*l*v,2*l*l*v,0,-6*l*v,4*l*l*v]])
        ids=np.array([3*i,3*i+1,3*i+2,3*j,3*j+1,3*j+2])
        stiffness[np.ix_(ids,ids)]+=t.T@kl@t
        elements.append((name,ids,t,kl,l))
    def truss(i,j,a,name):
        dx,dz=nodes[j]-nodes[i]; l=hypot(dx,dz); c=dx/l; s=dz/l
        ids=np.array([3*i,3*i+1,3*j,3*j+1]); v=np.array([-c,-s,c,s])
        stiffness[np.ix_(ids,ids)]+=E*a/l*np.outer(v,v)
        elements.append((name,ids,v,E*a/l,l))
    beam(0,1,area,inertia,'knee_lower')
    beam(1,2,area,inertia,'knee_incline')
    beam(2,3,area,inertia,'upper')
    beam(1,4,20*(40-4.6),20*(40**3-4.6**3)/12,'console')
    if mode in ('crown_only','rear_triangle'):
        truss(4,2,12*26,'B_S')
    if mode=='rear_triangle':
        truss(4,5,12*26,'B_R'); truss(2,5,12*26,'S_R')
    a=radians(angle); n=P+np.array([30*cos(a)-110*sin(a),30*sin(a)+110*cos(a)])
    u=(n-B)/np.linalg.norm(n-B)
    f=np.zeros(18); f[3*3+1]=-8000/132
    f[12:14]=sign*100*u+[0,-mass*9.81]
    f[14]=-mass*9.81*(300-B[0])
    fixed=[0,1,2,15,16,17]; free=[i for i in range(18) if i not in fixed]
    disp=np.zeros(18); disp[free]=np.linalg.solve(stiffness[np.ix_(free,free)],f[free])
    reactions=stiffness@disp-f
    # Force and moment at common old foot. No friction or bilateral balancing.
    fx,fz,md=reactions[:3]
    # Moment couple and direct Fx are COLLINEAR at one pin, not orthogonal.
    q=hypot(abs(md)/34+abs(fx)/2,abs(fz)/2)
    # Check global static equilibrium, including nodal applied couples.
    balanced=(reactions+f).reshape(-1,3)
    assert np.linalg.norm(balanced[:,:2].sum(axis=0)) < 1e-6
    assert abs(sum(nodes[i,0]*r[1]-nodes[i,1]*r[0]+r[2] for i,r in enumerate(balanced))) < 1e-5
    # Conservative local contact envelope, retained original contact arms.
    mp=abs(md)+45*abs(fx)
    # Out-of-plane envelope is deliberately NOT solved by this XZ model.
    # Use full C roll plus all new vertical loads with their Y offsets.
    mr=8000/132*78 + abs(sign*100*u[1])*23 + mass*9.81*53
    uplift=max(0,-fz)
    rail=mp/34+mr/70+uplift+100
    inboard=mr/78+uplift+100
    ann=lambda od:pi*(od*od-4.3**2)/4
    def bolt_margin(load,bending=False,smooth=False):
        tau=load/(2*8.78) if bending else load/8.78
        sigma=load*4.2/(pi*(4 if smooth else 3.14)**3/32) if bending else load/8.78
        return 580/hypot(sigma,3**.5*tau)
    values={
      'M4_pin_threaded_8_8':bolt_margin(q,True),
      'M4_pin_smooth_8_8':bolt_margin(q,True,True),
      # Projected bearing area uses the actual 4mm bolt, NOT clearance bore4.6.
      'foot_cheek_bearing':5/(q/(2*6*4.0)),
      'foot_cheek_edge_shear':5/(q/(4*6*(6-2.3))),
      'foot_pad_pitch':10/(1.5*mp/((28-4.6)*18**2/6)),
      'foot_root_roll':10/(1.8*mr/((24-4.6)*20**2/6)),
      'base_M4_8_8':580/hypot(rail/8.78,3**.5*abs(fx)/8.78),
      'PETG_rail_OD12':5/(rail/ann(12)),
      'PETG_inboard_OD9':5/(inboard/ann(9)),
      'PETG_pitch_contact':5/(rail/96),
    }
    forces={}
    for name,ids,t,kl,l in elements:
        if isinstance(kl,float):
            force=float(kl*(t@disp[ids])); forces[name]=force
            # End holes excluded: this is stock axial stress / buckling only.
            ii=26*12**3/12
            pcr=pi*pi*E*ii/l**2
            values[name+'_stock']=min(5*12*26/abs(force),pcr/abs(force))
        else:
            end=kl@t@disp[ids]; forces[name]=end.tolist()
            if name.startswith('knee'):
                stress=1.8*(max(abs(end[2]),abs(end[5]))/section+max(abs(end[0]),abs(end[3]))/area)
                values[name]=10/stress
            if name=='console':
                z=20*(40**3-4.6**3)/12/20
                stress=1.8*(max(abs(end[2]),abs(end[5]))/z+abs(end[0])/(20*(40-4.6)))
                values[name]=10/stress
    return dict(angle=angle,sign=sign,mass_kg=mass,margins=values,
                root_D_N_Nmm=reactions[:3].tolist(),root_R_N_Nmm=reactions[15:].tolist(),
                pin_force_N=q,base_pitch_Nmm=mp,base_roll_Nmm=mr,
                rail_bolt_N=rail,inboard_bolt_N=inboard,forces=forces)

def main():
    OUT.mkdir(exist_ok=True)
    all_cases={}
    for mode in ('cantilever','crown_only','rear_triangle'):
        cases=[solve(mode,a,s,.600) for a in np.linspace(-10,20,61) for s in (-1,1)]
        worst={k:min(cases,key=lambda r:r['margins'][k]) for k in cases[0]['margins']}
        all_cases[mode]={'min_margins':{k:dict(value=v['margins'][k],angle=v['angle'],sign=v['sign']) for k,v in worst.items()},
                         'worst_pin_case':max(cases,key=lambda r:r['pin_force_N'])}
    kinematics=[]
    for a in np.linspace(-10,20,301):
        t=radians(a); v=P+np.array([30*cos(t)-110*sin(t),30*sin(t)+110*cos(t)])-B
        kinematics.append(np.linalg.norm(v))
    templates=list((Path(__file__).parent/'stl-candidate-p293d').glob('p293e_foot_template*.stl'))
    data={'status':'PREFLIGHT_ONLY_NOT_RELEASE_NO_GEOMETRY_CHANGE',
          'inputs':{'E_MPa':E,'PETG_bending_MPa':10,'PETG_bearing_shear_MPa':5,'target':1.5,
                    'one_side_crown_moment_Nm':8,'axial_Tr8_bound_N':100,'mass_allowance_kg':.6,
                    'CM_XZ_mm':[300,245],'Y_arch_B_CM_mm':[-228,-205,-175],
                    'residual_preload_assumption_N':100,'new_strut_stock_mm':[12,26],
                    'new_console_stock_mm':[20,40],'nodes_XZ_mm':nodes.tolist()},
          'modes':all_cases,
          'approved_A_length_mm':{'min':min(kinematics),'max':max(kinematics),'planning_blank':max(kinematics)+75},
          'template_sha256':{str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in templates},
          'limitations':['Linear planar screening, not full 3D solid FEA or CAD collision check.',
            'Force-sharing is stiffness dependent. New anchor/console/struts have no candidate solids.',
            'Full roll envelope retained even in rear triangle; no claimed out-of-plane load relief.',
            'New R anchor and crown/end joints NOT qualified; stock strut margin excludes holes/joints.',
            '8.8 steel proof580MPa is conditional; hardware grade and grip are unverified.',
            'PETG values are screening assumptions, not creep/fatigue/process allowables.',
            'Mass .600kg and CM are assumptions, not measured cradle data.']}
    (OUT/'preflight.json').write_text(json.dumps(data,indent=2)+'\n')
    print(json.dumps({m:{k:round(v['value'],3) for k,v in r['min_margins'].items()} for m,r in all_cases.items()},indent=2))

if __name__=='__main__': main()
