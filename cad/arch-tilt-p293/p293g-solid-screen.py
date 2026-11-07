#!/usr/bin/env python3
"""Actual mesh sections/masses; mutually exclusive LC1/LC2 screening, not FEA.

Run after p293g-run-gate.py. Hardware CAD masses are estimates, not weighing.
"""
import json, math, argparse, subprocess
from pathlib import Path
import numpy as np
import trimesh
from shapely.geometry import Polygon, Point
HERE=Path(__file__).parent; OUT=HERE/'p293g-candidate-results'
ap=argparse.ArgumentParser();ap.add_argument('--tag',default='candidate-03');args=ap.parse_args()
report=json.loads((OUT/f'{args.tag}-report.json').read_text())
scene=HERE.parent/'independent-check/out/scene'
meshes={k.rsplit('@',1)[0]:trimesh.load(scene/v,force='mesh') for k,v in report['files'].items() if k.endswith('@0')}

def section(mesh,origin,normal,axes,select=lambda p:True):
    path=mesh.section(plane_origin=origin,plane_normal=normal)
    loops=[np.asarray(p)[:,axes] for p in path.discrete if select(np.asarray(p))]
    polys=[Polygon(p) for p in loops]
    A=0.;first=np.zeros(2);second=np.zeros(2);lo=np.array([np.inf]*2);hi=-lo
    for i,p in enumerate(loops):
        depth=sum(q.contains(Point(p[0])) for j,q in enumerate(polys) if j!=i)
        x,y=p[:-1].T;xx,yy=p[1:].T;c=x*yy-xx*y
        area=c.sum()/2;sgn=(-1)**depth*np.sign(area)
        A+=sgn*area;first+=sgn*np.array([((x+xx)*c).sum(),((y+yy)*c).sum()])/6
        second+=sgn*np.array([((x*x+x*xx+xx*xx)*c).sum(),((y*y+y*yy+yy*yy)*c).sum()])/12
        lo=np.minimum(lo,p.min(axis=0));hi=np.maximum(hi,p.max(axis=0))
    cm=first/A;I=second-A*cm*cm;Z=I/np.maximum(cm-lo,hi-cm)
    return dict(A_mm2=A,centroid=cm.tolist(),I_mm4=I.tolist(),Z_mm3=Z.tolist(),bounds=[lo.tolist(),hi.tolist()])

foot=meshes['p293g_foot'];knee=meshes['p293d_knee[side=-1]']
sections={'rail_pad_X230':section(foot,[230,0,0],[1,0,0],[1,2],lambda p:p[:,1].min()>190),
          'knee_Z114':section(knee,[0,0,114],[0,0,1],[0,1]),
          'knee_Z97':section(knee,[0,0,97],[0,0,1],[0,1]),
          'console_Z180':section(meshes['p293g_console_inboard'],[0,0,180],[0,0,1],[0,1])}

archprefix=('p293d_','p293g_crown','p293g_crossbar','arch_crossbar_center','joint_sleeve_x')
def density(n):
    if 'hardware' not in n:return .00127  # g/mm3, solid PETG
    if 'pulley' in n:return .00270
    if 'belt' in n:return .00120
    if 'nut_hardware' in n:return .00850
    return .00785

mass={}
for n,m in meshes.items():
    if not n.startswith(('p293g_',)+archprefix):continue
    g=abs(m.volume)*density(n)
    source='mesh volume x nominal material density'
    if n=='p293g_fit_hardware':g=205.;source='DFRobot FIT0186 nominal205g, not physical weighing'
    if n=='p293g_kfl08_hardware':g=22.;source='nominal supplier48x27 KFL08; purchased G15 unmeasured'
    mass[n]=dict(volume_mm3=abs(m.volume),mass_g=g,CM_mm=m.center_mass.tolist(),source=source,
                 watertight=bool(m.is_watertight),bodies=len(m.split()))

P=np.array([387.,0,51.837363636]);B=np.array([260.,-205.,220.])
def N(a):
    t=math.radians(a);return P+np.array([30*math.cos(t)-110*math.sin(t),-205,30*math.sin(t)+110*math.cos(t)])
def frame(a):
    u=(N(a)-B)/np.linalg.norm(N(a)-B);n=np.array([-u[2],0,u[0]])
    return np.column_stack([n,[0,-1,0],u])
R0=frame(0)
localcradle=meshes['p293g_cradle'].copy()
localcradle.vertices=(localcradle.vertices-B)@R0
sections['cradle_front_v0']=section(localcradle,[0,0,0],[0,1,0],[0,2],lambda p:p[:,2].max()<0)
sections['cradle_spine_u20']=section(localcradle,[0,0,20],[0,0,1],[0,1],lambda p:p[:,1].min()>0)
sections['cradle_bridge_n15']=section(localcradle,[15,0,0],[1,0,0],[1,2],lambda p:p[:,1].min()>0 and p[:,2].min()>-10 and p[:,2].max()<10)
sections['cradle_B_boss_n0']=section(localcradle,[0,0,0],[1,0,0],[1,2],lambda p:p[:,1].min()>0 and p[:,2].min()>-11 and p[:,2].max()<11)
localcrown=meshes['p293g_crown[side=-1]'].copy();vv=localcrown.vertices.copy()
cc=132/math.hypot(132,125);ss=125/math.hypot(132,125)
localcrown.vertices=np.column_stack([(vv[:,0]-258)*cc+(vv[:,2]-120)*ss,vv[:,1]+221.6,-(vv[:,0]-258)*ss+(vv[:,2]-120)*cc])
sections['crown_lower_pin']=section(localcrown,[math.hypot(132,125)-55,0,0],[1,0,0],[1,2])
moving={'p293g_nut_arm','p293g_nut_trunnion','p293g_t8_nut_hardware'}
fixed={'p293g_console_inboard','p293g_console_outboard'}
fasteners=json.loads((OUT/'hardware-probe.json').read_text()).get('mass_estimates',{})
assert fasteners, 'Run p293g-hardware-probe.py first to measure the actual M4 envelopes.'
def gravities(a,base=False,kfl_mass=None):
    loads=[]
    for n,row in mass.items():
        if n in moving:continue  # carried by moving yoke; moving torque already bounded by8Nm
        cm=np.array(row['CM_mm']);g=row['mass_g']
        if n=='p293g_kfl08_hardware' and kfl_mass is not None:g=kfl_mass
        if n.startswith('p293g_foot'):
            if not base or cm[1]>0:continue
        elif n.startswith(archprefix):
            # Symmetric arch gravity shared between feet. Actual solid mass/CM,
            # half of complete arch (including positive side and centre spans).
            g/=2
            # Symmetric crown/crossbeam gravity transfers at this leg line,
            # not at the opposite leg's Y or the full arch CM at Y0.
            cm[1]=-221.6
        elif n not in fixed:
            cm=B+frame(a)@R0.T@(cm-B)
        loads.append((cm,np.array([0.,0.,-g*.00981])))
    for row in fasteners.values():
        cm=np.array(row['CM_mm']);g=row['mass_g'];group=row['group']
        if group in ['N','arm_mount']:continue
        if group=='foot':
            if not base or cm[1]>0:continue
        elif group=='crown':g/=2;cm[1]=-221.6
        loads.append((cm,np.array([0.,0.,-g*.00981])))
    return loads

def evaluate(lc,a,s,kfl_mass=None):
    p=N(a);u=(p-B)/np.linalg.norm(p-B);r=p-P
    h=abs(np.cross(r,u)[1]);F=s*8000/h*u if lc=='LC1' else np.zeros(3)
    S=np.array([0.,0.,s*8000/132]) if lc=='LC2' else np.zeros(3)
    forcepoints=[(B,F),(np.array([390.,-221.6,245.]),S)]
    def resultant(origin,base=False):
        loads=forcepoints+gravities(a,base,kfl_mass)
        f=sum((f for _,f in loads),np.zeros(3))
        m=sum((np.cross(p-origin,f) for p,f in loads),np.zeros(3))
        return f,m
    f,m=resultant(np.array([258.,-221.6,97.]));fx,_,fz=f
    fb,mb=resultant(np.array([258.,-150.,52.]),True)
    mp=abs(mb[1]);roll=abs(mb[0]);q=math.hypot(abs(m[1])/34+abs(fx)/2,abs(fz)/2)
    rail=mp/34+roll/70+max(0,fb[2]);inboard=roll/71.6+max(0,fb[2])
    rows={}
    def add(n,stress,limit,detail):rows[n]=dict(stress_MPa=float(stress),limit_MPa=limit,margin=float(limit/stress) if stress else 1e30,detail=detail)
    # Hole section uses actual STL contour incl hole. Kt only actual notch.
    _,mh=resultant(np.array([258.,-221.6,114.]))
    sh=sections['knee_Z114'];ss=sections['knee_Z97']
    add('knee',max(1.8*(abs(mh[1])/sh['Z_mm3'][0]+abs(fz)/sh['A_mm2']),abs(m[1])/ss['Z_mm3'][0]+abs(fz)/ss['A_mm2']),10,'actual mesh sections; Kt1.8 only Z114 hole')
    pm=q*(2*28.6-20)/8
    add('M4_Z80_114_thread_8_8',math.hypot(pm/(math.pi*3.14**3/32),math.sqrt(3)*q/(2*8.78)),580,'conditional grade8.8, threaded root, bending+shear')
    # Unequal effective support centres: longer inboard8mm sleeve, shorter
    # outboard5mm sleeve. Conservative60%Q on EACH, not an assumed50/50.
    add('foot_cheek_bearing',.6*q/(8*5),5,'OD8 steel spacer; minimum5mm PETG bearing length;60% of full pin load')
    # X/Z edge ligaments are different; do not apply the large horizontal
    # pin force to the much shorter vertical edge ligament.
    qx=abs(m[1])/34+abs(fx)/2;qz=abs(fz)/2
    add('foot_cheek_edge',.6*qx/(2*5*(13-4.1))+.6*qz/(2*5*(6-4.1)),5,'OD8.2 bore, effective5mm; directional X/Z ligament sum, no generic Kt')
    add('knee_bearing',q/80,5,'20mm solid knee, actual4mm bolt')
    add('rail_pad_pitch',1.5*mp/sections['rail_pad_X230']['Z_mm3'][1],10,'actual section incl45deg flare, seat14.6; Kt1.5 actual bore')
    add('foot_root_roll',1.8*roll/((24-4.6)*20**2/6),10,'inboard real bore net section, unchanged root')
    bridge=[]
    for t in np.linspace(0,39/47,41):
        b=24+2*t;hh=36-16*t;short,long=sorted([b,hh]);_,mm=resultant(np.array([240+18*t,-150.,70-8*t]),True)
        bridge.append(abs(mm[0])/(.2*long*short**2))
    add('foot_bridge_torsion',max(bridge),5,'unchanged bore-free source hull; Kt1; roll moment about X')
    add('base_M4_8_8',math.hypot(rail/8.78,math.sqrt(3)*abs(fx)/8.78),580,'external load only, no preload')
    add('PETG_rail_OD14',(rail+100)/(math.pi*(14**2-4.3**2)/4),5,'100N residual preload only at washer seats')
    add('PETG_inboard_OD9',(inboard+100)/(math.pi*(9**2-4.3**2)/4),5,'100N residual preload only at washer seats')
    add('PETG_pin_head_OD10',100/(math.pi*(10**2-8.2**2)/4),5,'100N residual preload only; conservative PETG annulus outside sleeve bore')
    add('PETG_pin_nut_OD12',100/(math.pi*(12**2-8.2**2)/4),5,'100N residual preload only; inboard washer outside sleeve bore')
    add('PETG_crown_washer',100/(math.pi*(9**2-4.6**2)/4),5,'100N residual preload only under OD9 steel washer0.5; actual knee bore4.6')
    add('pitch_contact',rail/96,5,'8x12 contact, no preload')
    z=sections['console_Z180']['Z_mm3'][0]
    carried=[(n,row) for n,row in mass.items() if n.startswith('p293g_') and n not in moving and not n.startswith(archprefix) and not n.startswith('p293g_foot')]
    cweight=sum(row['mass_g']*.00981 for n,row in carried)
    cmoment=0.
    for n,row in carried:
        cm=np.array(row['CM_mm'])
        if n not in fixed:cm=B+frame(a)@R0.T@(cm-B)
        cmoment-=(cm[0]-260)*row['mass_g']*.00981
    for row in fasteners.values():
        if row['group'] in ['console','B']:
            cweight+=row['mass_g']*.00981
            cmoment-=(row['CM_mm'][0]-260)*row['mass_g']*.00981
    mc=abs((B[0]-260)*F[2]-(B[2]-180)*F[0]+cmoment)
    add('console_stem',mc/z+np.linalg.norm(F+[0,0,-cweight])/sections['console_Z180']['A_mm2'],10,'actual Z180 stock, all console/carried solid gravity included conservatively, Kt1')
    add('cradle_B_bearing',np.linalg.norm(F+[0,0,-cweight])/(2*8*4),5,'2x8mm pivot bosses;4mm stub pins; carried gravity included')
    frameforce=np.linalg.norm(F+[0,0,-cweight])
    memberZ=min(sections['cradle_spine_u20']['Z_mm3'][0],sections['cradle_bridge_n15']['Z_mm3'][1])
    memberA=min(sections['cradle_spine_u20']['A_mm2'],sections['cradle_bridge_n15']['A_mm2'])
    framemoment=np.linalg.norm(F)*26+abs(cmoment)
    add('cradle_bridge_spine',framemoment/memberZ+frameforce/memberA,10,'actual mesh sections; full one-path force26mm +actual solid gravity moment aboutB')
    boss=sections['cradle_B_boss_n0']
    add('cradle_B_boss_net',1.8*(framemoment/boss['Z_mm3'][1]+frameforce/boss['A_mm2']),10,'actual mesh D20/4.6 boss, Kt1.8 real bore; full path force +actual gravity moment')
    add('console_B_ear_bearing',.6*np.linalg.norm(F+[0,0,-cweight])/(2.8*4),5,'actual2.8mm ear ligament below head recess;60% total pivot load on either ear')
    add('console_fork_leg',.6*frameforce*27/(6*20**2/6)+frameforce/(6*20),10,'actual6x20 fork leg at barZ193,27mm lever; no bore at stock root/Kt1')
    mw=sum(mass[n]['mass_g']*.00981 for n in moving)+sum(row['mass_g']*.00981 for row in fasteners.values() if row['group'] in ['N','arm_mount'])
    add('nut_clevis_bearing',.6*(np.linalg.norm(F)+mw)/(4*4),5,'2x4mm moving fork legs;60% total load on either side, local gravity included')
    # Moving-arm sections MUST use the moving U/W components, not world X/Z.
    t=math.radians(a);Rm=np.array([[math.cos(t),0,-math.sin(t)],[0,1,0],[math.sin(t),0,math.cos(t)]])
    fl=Rm.T@F;fu=abs(fl[0])+mw;fw=abs(fl[2])+mw
    add('nut_arm_riser',1.5*((fu*53+fw*40)/(10*30**2/6)+fw*13.2/(30*10**2/6)+fw/(30*10)),10,'actual30x10; FULL path U/W biaxial moments FU53+FW40 and13.2mm Y offset; Kt1.5 at R3 root')
    add('nut_arm_riser_torsion',1.5*fu*13.2/(.2*30*10**2),5,'actual30x10 torsion; full path/13.2mm offset; Kt1.5 at R3 root')
    add('nut_arm_shelf',1.5*((fw*40+fu*30)/(30*16**2/6)+fu*13.2/(16*30**2/6)+fu/(30*16)),10,'actual30x16 low shelf; full path U/W bending,40mm span +30mm pin-height couple; Kt1.5 at R3 root')
    add('nut_arm_shelf_torsion_U',1.5*fw*13.2/(.2*30*16**2),5,'actual30x16 shelf, full13.2mm Y offset; Kt1.5 at R3 root')
    add('nut_arm_shelf_torsion_Y',1.5*fu*30/(.2*40*16**2),5,'actual40x16 transverse shelf section, full pin-height couple; Kt1.5 at R3 root')
    add('nut_arm_fork_legs',1.5*(fu*30/(4*24**2/6)+fw/(4*24)),10,'actual4x24 upward fork web; full one-leg force30mm lever; Kt1.5 at R3 root')
    add('nut_arm_bight',1.5*(fu*21.2/(14*24**2/6)+fw*21.2/(24*14**2/6)),10,'actual24x14 bight, full path biaxial load; Kt1.5 at R3 junction')
    add('nut_arm_bight_torsion',1.5*fu*30/(.2*24*14**2),5,'actual24x14 lower crossbar, full one-leg moment30mm ABOVE centre; Kt1.5 at R3 junction')
    add('nut_trunnion_plate',1.5*(np.linalg.norm(F)+mw)*33.6/4/((32-10.6)*8**2/6),10,'actual32x8 plate/10.6 bore;33.6mm pin span; Kt1.5 real bore')
    ownF=np.zeros(3);ownM=np.zeros(3);origin=np.array([-5.,-204.,47.])
    for n in moving:
        row=mass[n];cm0=np.array(row['CM_mm'])
        cmw=P+Rm@(cm0-P) if n=='p293g_nut_arm' else N(a)+frame(a)@R0.T@(cm0-N(0))
        cml=Rm.T@(cmw-P);gg=Rm.T@np.array([0.,0.,-row['mass_g']*.00981])
        ownF+=gg;ownM+=np.cross(cml-origin,gg)
    for row in fasteners.values():
        if row['group'] not in ['N','arm_mount']:continue
        cm0=np.array(row['CM_mm'])
        cmw=P+Rm@(cm0-P) if row['group']=='arm_mount' else N(a)+(cm0-N(0))
        cml=Rm.T@(cmw-P);gg=Rm.T@np.array([0.,0.,-row['mass_g']*.00981])
        ownF+=gg;ownM+=np.cross(cml-origin,gg)
    load=fl+ownF;moment=np.cross(np.array([30.,-205.,110.])-origin,fl)+ownM
    tension=abs(moment[1])/40+abs(moment[0])/80+max(0,load[2])/4
    add('arm_yoke_OD10_washer',(tension+100)/(math.pi*(10**2-4.6**2)/4),5,'actual four bolts20x40 pitch; local U/W force+actual moving mass moments; preload100 only washers')
    add('arm_yoke_M4_8_8',math.hypot(tension/8.78,math.sqrt(3)*np.linalg.norm(load[:2])/4/(math.pi*3.14**2/4)),580,'four mounting M4, no preload in external steel load')
    add('arm_base_plate',1.5*(abs(moment[1])/((52-2*4.6)*12**2/6)+abs(load[2])/((52-2*4.6)*12)),10,'actual12mm base/net two4.6 bores across52mm; full moment bound; Kt1.5 real holes')
    # locating bearing mount at maximum permitted48mm pitch, slotted plate net section
    gravity_allowance=sum(row['mass_g']*.00981 for n,row in mass.items() if 'hardware' in n and 'nut_hardware' not in n)
    add('cradle_front_plate',1.5*(np.linalg.norm(F)+gravity_allowance)*48/4/sections['cradle_front_v0']['Z_mm3'][1],10,'actual mesh front section incl18mm bore, Kt1.5;48mm span; all fixed hardware gravity added conservatively')
    # Crown: two original pins20mm apart, now single inboard12x26mm lug.
    # Upper-member gravity included; no LC1 actuator moment at crown.
    L=math.hypot(132,125);joint=np.array([258+(L-45)*132/L,-221.6,120+(L-45)*125/L])
    crownloads=[(np.array([390.,-221.6,245.]),S)]
    for n,row in mass.items():
        if n.startswith(archprefix) and 'knee' not in n:
            cm=np.array(row['CM_mm']);crownloads.append((cm,np.array([0.,0.,-row['mass_g']*.00981/2])))
    for row in fasteners.values():
        if row['group']=='crown':crownloads.append((np.array(row['CM_mm']),np.array([0.,0.,-row['mass_g']*.00981/2])))
    cf=sum((f for _,f in crownloads),np.zeros(3));cm=sum((np.cross(p-joint,f) for p,f in crownloads),np.zeros(3))
    cq=abs(cm[1])/20+np.linalg.norm(cf)/2
    add('crown_inboard_lug_bearing',cq/(12*4),5,'single12mm inboard lug, original20mm pin pitch')
    lug=sections['crown_lower_pin']
    add('crown_lug_local_biaxial',1.8*(cq*16.4/lug['Z_mm3'][0]+cq/lug['A_mm2']),10,'actual mesh lug/4.6 bore, Kt1.8 at real hole; local pin-root moment16.4mm')
    add('crown_pin_thread_10_9',math.hypot(cq*16.4/(math.pi*3.14**3/32),math.sqrt(3)*cq/(math.pi*3.14**2/4)),830,'REQUIRES10.9 button-head pin; root3.14, cantilever16.4mm; no smooth-grip credit')
    return dict(LC=lc,angle_deg=a,sign=s,F_Tr8_N=np.linalg.norm(F),base_pitch_Nmm=mp,base_roll_Nmm=roll,rows=rows)

envelopes={}
for lc in ['LC1','LC2']:
    cases=[evaluate(lc,float(a),s) for a in np.linspace(-10,20,301) for s in [-1,1]]
    envelopes[lc]={k:{**(w:=max(cases,key=lambda c:c['rows'][k]['stress_MPa']))['rows'][k], 'angle_deg':w['angle_deg'],'sign':w['sign']} for k in cases[0]['rows']}
print('sections',json.dumps(sections))
print('mass total CAD g including probed M4',sum(r['mass_g'] for r in mass.values())+sum(r['mass_g'] for r in fasteners.values()))
for k in envelopes['LC1']:print(k,*[round(envelopes[lc][k]['margin'],3) for lc in ['LC1','LC2']])
data=dict(status='SECTIONAL_SCREEN_NOT_PHYSICAL_RELEASE',sections=sections,masses=mass,envelopes=envelopes,
          assumptions=['solidPETG density1.27g/cm3; CAD estimates, not measured masses','FIT0186 nominal205g, KFL08 provisional22g',
          'LC1=8000/h atB both signs +actual solids gravity; LC2=one crown vertical60.606N, zeroTr8 +gravity',
          '8Nm includes moving gravity; moving nut arm/clevis masses reported, not double-added atB',
          'Full symmetric arch CAD mass gravity split equally across feet; LC2 stop force wholly on one side',
          'No preload in moments; residual100N only washers; this is screening, not joint/contact FEA'])
data['fastener_mass_estimates']=fasteners
data['total_modelled_mass_g']=sum(r['mass_g'] for r in mass.values())+sum(r['mass_g'] for r in fasteners.values())
data['target_margin']=1.
# Unmeasured purchased G15: sampled sensitivity, NOT a substitute for weighing.
oldmass=mass['p293g_kfl08_hardware']['mass_g'];mass['p293g_kfl08_hardware']['mass_g']=100.
try:
    data['G15_100g_sensitivity_sampled']={}
    for lc in ['LC1','LC2']:
        cases=[evaluate(lc,a,s) for a in [-10,-3,0,15,20] for s in [-1,1]]
        data['G15_100g_sensitivity_sampled'][lc]=min(row['margin'] for case in cases for row in case['rows'].values())
finally:mass['p293g_kfl08_hardware']['mass_g']=oldmass
data['print_STL_sanity']={}
for path in sorted((HERE/'stl-candidate-p293g').glob('*.stl')):
    mesh=trimesh.load(path,force='mesh')
    scene_name=next(n for n in meshes if n.split('[')[0].split('#')[0]==path.stem)
    delta=abs(abs(mesh.volume)-abs(meshes[scene_name].volume))/abs(mesh.volume)
    data['print_STL_sanity'][path.name]=dict(watertight=bool(mesh.is_watertight),bodies=len(mesh.split()),volume_mm3=abs(mesh.volume),bounds=mesh.bounds.tolist(),scene_volume_relative_difference=delta)
root=HERE.resolve().parents[1]
data['unchanged_files']={}
for name in ['cad/independent-check/scene_check.py','cad/independent-check/scadtree.py','cad/independent-check/parts.py','cad/check-current-scene.sh',
             'cad/arch-tilt-p293/stl-candidate-p293d/p293d_knee.stl','cad/arch-tilt-p293/stl-candidate-p293d/p293e_foot_template.stl','cad/arch-tilt-p293/stl-candidate-p293d/p293e_foot_template_R.stl']:
    old=subprocess.check_output(['git','rev-parse','HEAD:'+name],cwd=root,text=True).strip()
    new=subprocess.check_output(['git','hash-object',name],cwd=root,text=True).strip()
    data['unchanged_files'][name]=dict(HEAD_blob=old,working_blob=new,identical=old==new)
data['screen_pass']=all(row['margin']>=1 for lc in envelopes.values() for row in lc.values())
(OUT/'solid-screen.json').write_text(json.dumps(data,indent=2)+'\n')
lines=['# P2-93g — generated solid tables','', 'Source: `p293g-solid-screen.py --tag final`; CAD mass estimates, NOT physical weighing. Screening is NOT RELEASED.','',
       '| Check | LC1 stress MPa | LC1 margin | LC2 stress MPa | LC2 margin |','|---|---:|---:|---:|---:|']
for key in envelopes['LC1']:
    a=envelopes['LC1'][key];b=envelopes['LC2'][key]
    lines.append(f"| {key} | {a['stress_MPa']:.4f} | {a['margin']:.3f} | {b['stress_MPa']:.4f} | {b['margin']:.3f} |")
lines+=['',f"Total modelled CAD mass incl probed M4: {data['total_modelled_mass_g']:.2f}g.",f"G15 mass100g sensitivity at five gate angles, both signs (NOT a continuous envelope): {data['G15_100g_sensitivity_sampled']}"]
lines+=['','| Actual scene solid | CAD volume cm3 | Mass g | CM X/Y/Z mm |','|---|---:|---:|---|']
for n,row in mass.items():lines.append(f"| {n} | {row['volume_mm3']/1000:.3f} | {row['mass_g']:.2f} | {', '.join(f'{x:.2f}' for x in row['CM_mm'])} |")
lines+=['','Supplemental M4 hardware envelope masses (steel density7.85g/cm3, conservative solid shafts/hex circumscribed envelopes, NOT physical weighing):','', '| Support group | CAD mass g |','|---|---:|']
for group in sorted({r['group'] for r in fasteners.values()}):lines.append(f"| {group} | {sum(r['mass_g'] for r in fasteners.values() if r['group']==group):.2f} |")
lines+=['','| Section | Area mm2 | Centroid mm | I mm4 (coordinate-square integrals) | Z mm3 |','|---|---:|---|---|---|']
for n,row in sections.items():lines.append(f"| {n} | {row['A_mm2']:.3f} | {', '.join(f'{v:.3f}' for v in row['centroid'])} | {', '.join(f'{v:.3f}' for v in row['I_mm4'])} | {', '.join(f'{v:.3f}' for v in row['Z_mm3'])} |")
lines+=['','| Angle deg | Arch → moving | Arch → cradle | Arch → arm | Crown → Tr8 | Arm → moving | Arm → cradle | Arch/tilt → drive | Arch → F |','|---:|---:|---:|---:|---:|---:|---:|---:|---:|']
gaps=json.loads((OUT/'solid-clearances.json').read_text())['minimum_gaps']
for r in gaps:lines.append('| '+str(r['angle'])+' | '+' | '.join(f"{r[k]['mm']:.3f}" for k in ['arch_to_moving','arch_to_cradle','arch_to_arm','crown_to_Tr8','arm_to_moving','arm_to_cradle','arch_tilt_to_drive','arch_to_F'])+' |')
(HERE/'P2-93g-solid-tables.md').write_text('\n'.join(lines)+'\n')
assert data['screen_pass'], 'A sectional screening row FAILS; do not release/claim PASS.'
assert all(r['watertight'] and r['bodies']==1 for r in data['print_STL_sanity'].values()), 'Invalid print STL'
assert all(r['scene_volume_relative_difference']<1e-5 for r in data['print_STL_sanity'].values()), 'STL/scene geometry mismatch; export and gate must use the same sources'
assert all(r['identical'] for r in data['unchanged_files'].values()), 'Protected QA/knee/template changed'
