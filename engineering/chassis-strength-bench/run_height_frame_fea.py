#!/usr/bin/env python3
"""Mixed S6/B32R ideal-joint screen of the height study; NOT print certification.

Units mm/N/MPa/tonne. Metal plate/spacer assumptions are explicit. Meshes are
generated anew; old low-chassis reports are never relabelled as upper-frame FEA.
"""
from __future__ import annotations

import argparse
import hashlib
import itertools
import json
import math
from pathlib import Path
import re
import subprocess

from run_global_frame_fea import ROOT, SOLVER, runtime_env

GMSH = Path('/snap/freecad/current/usr/bin/gmsh')
CAD = ROOT.parents[1] / 'cad/modular-test-chassis/height-infrastructure-study.scad'
PITCH = math.radians(20)
G = 9806.65
SCENARIOS = ('gravity', 'turn_reference', 'motor_torque_reference',
             'motor_torque_peak_screen', 'carry_rear_2g', 'wheel_mass_1p2kg')


def xyz(x, y, z):
    return (410+x*math.cos(PITCH)-z*math.sin(PITCH), y,
            298+x*math.sin(PITCH)+z*math.cos(PITCH))


def rotate(v):
    return (v[0]*math.cos(PITCH)-v[2]*math.sin(PITCH), v[1],
            v[0]*math.sin(PITCH)+v[2]*math.cos(PITCH))


def cross(a, b):
    return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])


def mount_z(x):
    return 298+(x-410)*math.tan(PITCH)-57/math.cos(PITCH)


def mount_x(x):
    return (x-410-57*math.sin(PITCH))/math.cos(PITCH)


def panel_geometry(upper, mesh):
    """Accepted reliefs sampled as overlapping disks (not exact CAD notch mesh).

    Motor/plate bolt holes are ideal point attachments; shaft openings retained.
    The sampling approximates the source's short circle-hull segments.
    """
    lines = ['SetFactory("OpenCASCADE");', 'Rectangle(1)={-128,-157,0,256,314};']
    cuts, sid = [], 2
    half = math.sqrt(45**2-39**2)
    lines.append(f'Rectangle({sid})={{100,{-half},0,220,{2*half}}};')
    cuts.append(sid); sid += 1
    if upper:
        for yy in (-129,129):
            lines.append(f'Disk({sid})={{0,{yy},0,6,6}};')
            cuts.append(sid); sid += 1
    else:
        for i in range(141):
            t=i*.0005
            xx=38.8738059257+5269.52537961*t-.5*3351.79740459*t*t
            zz=-4.57450083064-196.876959017*t-.5*9208.98768370*t*t
            d=zz+39
            if 0 <= d < 38:
                rr=math.sqrt(38**2-d*d)
                # Cuts beyond the panel need not be meshed.
                if xx-rr < 128:
                    lines.append(f'Disk({sid})={{{xx},0,0,{rr},{rr}}};')
                    cuts.append(sid); sid += 1
    lines.append('s[]=BooleanDifference{Surface{1}; Delete;}{Surface{'+','.join(map(str,cuts))+'}; Delete;};')
    points=[(x,y) for x in (-115,115) for y in (-145,145)]
    if upper:
        points += [(15*math.cos(a*math.pi/2),y+15*math.sin(a*math.pi/2))
                   for y in (-129,129) for a in range(4)]
    for i,(x,y) in enumerate(points):
        pid=1000+i
        lines += [f'Point({pid})={{{x},{y},0,{mesh}}};', f'Point{{{pid}}} In Surface{{s[0]}};']
    lines += ['Physical Surface("PANEL")={s[]};', 'Mesh.ElementOrder=2;',
              f'Mesh.MeshSizeMax={mesh};', f'Mesh.MeshSizeMin={mesh/4};',
              'Mesh.MeshSizeFromCurvature=16;', 'Mesh.Algorithm=6;']
    return '\n'.join(lines)+'\n'


def mesh_panel(work, upper, mesh):
    stem=f'panel_{"upper" if upper else "lower"}_{mesh:g}'
    geo=work/f'{stem}.geo'; geo.write_text(panel_geometry(upper,mesh))
    result=subprocess.run([str(GMSH),str(geo),'-2','-format','inp','-o',str(work/f'{stem}.inp'),'-v','2'],
                          env=runtime_env(),capture_output=True,text=True,timeout=60)
    (work/f'{stem}.log').write_text(result.stdout+result.stderr)
    if result.returncode: raise RuntimeError(f'Gmsh failed: {stem}')
    nodes, triangles, mode = {}, [], ''
    for line in (work/f'{stem}.inp').read_text().splitlines():
        if line.startswith('*'):
            mode='node' if line.upper().startswith('*NODE') else (
                'element' if line.upper().startswith('*ELEMENT') and
                re.search(r'TYPE=(CPS6|S6|CPE6)',line.upper()) else '')
            continue
        fields=line.split(',')
        if mode=='node' and len(fields)>=4:
            nodes[int(fields[0])]=tuple(map(float,fields[1:4]))
        elif mode=='element' and len(fields)==7:
            triangles.append(tuple(map(int,fields[1:])))
    if not triangles: raise RuntimeError(f'No quadratic triangles in {stem}')
    return nodes,triangles


class Model:
    def __init__(self):
        self.nodes={}; self.index={}; self.buckets={}; self.beams=[]; self.shells=[]; self.members=[]
    def node(self,p):
        key=tuple(round(x,5) for x in p)
        if key in self.index: return self.index[key]
        bucket=tuple(math.floor(v*1e4) for v in p)
        for offset in itertools.product((-1,0,1),repeat=3):
            neighbour=tuple(bucket[k]+offset[k] for k in range(3))
            for n in self.buckets.get(neighbour,()):
                if math.dist(p,self.nodes[n])<1e-4: return n
        n=len(self.nodes)+1; self.index[key]=n; self.nodes[n]=key
        self.buckets.setdefault(bucket,[]).append(n)
        return n
    def add(self,a,b,w=20,h=20,t=2.4,kind='BOX',material='POLYMER',group='upper_frame'):
        self.members.append((tuple(a),tuple(b),w,h,t,kind,material,group))
        self.node(a); self.node(b)
    def mesh(self,size):
        # Split at ALL attachment/intersection nodes before making quadratic beams.
        junctions=list(self.nodes.values())
        for a,b,w,h,t,kind,material,group in self.members:
            delta=[b[i]-a[i] for i in range(3)]; length=math.dist(a,b)
            tangent=[x/length for x in delta]
            axis=(0,0,1) if abs(tangent[2])<.95 else (0,1,0)
            dot=sum(axis[i]*tangent[i] for i in range(3))
            normal=[axis[i]-dot*tangent[i] for i in range(3)]
            norm=math.sqrt(sum(x*x for x in normal)); normal=[x/norm for x in normal]
            stations=[0.,1.]
            for p in junctions:
                s=sum((p[i]-a[i])*delta[i] for i in range(3))/(length*length)
                if 1e-8 < s < 1-1e-8 and math.dist(p,[a[i]+s*delta[i] for i in range(3)])<3e-5:
                    stations.append(s)
            stations=sorted(set(round(s,10) for s in stations))
            for lo,hi in zip(stations,stations[1:]):
                if length*(hi-lo)<1e-3: continue
                count=max(1,math.ceil(length*(hi-lo)/size))
                ids=[self.node([a[k]+(lo+(hi-lo)*j/(2*count))*delta[k] for k in range(3)])
                     for j in range(2*count+1)]
                for j in range(count):
                    self.beams.append((ids[2*j:2*j+3],w,h,t,kind,material,group,normal))
        # Every shell is connected to the chassis through the real candidate anchors.
        graph={n:set() for n in self.nodes}
        for ids,*_ in self.beams:
            for n in ids: graph[n].update(ids)
        for ids,*_ in self.shells:
            for n in ids: graph[n].update(ids)
        seen=set(); pending=[self.index[(-165.,-220.,37.)]]
        while pending:
            n=pending.pop()
            if n not in seen: seen.add(n); pending.extend(graph[n]-seen)
        if len(seen)!=len(self.nodes):
            unseen=set(graph)-seen; components=[]
            while unseen:
                pending=[next(iter(unseen))]; component=set()
                while pending:
                    n=pending.pop()
                    if n not in component: component.add(n); pending.extend(graph[n]-component)
                unseen-=component
                components.append((len(component),[self.nodes[n] for n in list(component)[:3]],
                                   [p for p in [(350.,220.,37.),(350.,260.,52.),
                                                 tuple(round(v,5) for v in (350,260,mount_z(350)))]
                                    if self.index.get(p) in component]))
            raise RuntimeError(f'Disconnected structural graph: {components[:12]}')


def build(work,mesh):
    m=Model()
    # Low chassis current 50x20 ramp-back geometry, not the old 25 mm variant.
    stations=sorted(set([-165,-115,-100,-65,35,200,250,262.4,294.4,300,326.4,330,350,440,455]))
    for side in (-1,1):
        for a,b in zip(stations,stations[1:]):
            m.add((a,side*220,37),(b,side*220,37),50,30,3.2,group='chassis')
    m.add((-165,-220,37),(-165,220,37),50,30,3.2,group='chassis')
    ys=[-220,-160,0,160,220]
    for a,b in zip(ys,ys[1:]):
        m.add((294.4,a,37 if abs(a)==220 else 32),(294.4,b,37 if abs(b)==220 else 32),
              50,20,3.2,group='chassis')
    for side in (-1,1):
        for direction in (-1,1):
            m.add((294.4+direction*32,side*220,37),(294.4,side*160,32),4,20,0,'RECT',group='chassis')
    # Body: centreline joints, back X bracing; no load-bearing roof skins.
    for x in (-115,250):
        for side in (-1,1):
            m.add((x,side*220,37),(x,side*220,52),30,50,0,'RECT')
            m.add((x,side*220,52),(x,side*220,412))
    for side in (-1,1):
        m.add((-115,side*220,412),(250,side*220,412))
    for x in (-115,250,-100): m.add((x,-220,412),(x,220,412))
    for side in (-1,1):
        a=(-115,side*210,97); b=(-115,-side*210,392)
        m.add((-115,side*220,97),a,6,6,0,'RECT')
        m.add(a,b,6,6,0,'RECT')
        m.add(b,(-115,-side*220,392),6,6,0,'RECT')
    # Independent launcher support and outboard roots. Bracket is equivalent RECT.
    for x in (350,440):
        for side in (-1,1):
            m.add((x,side*220,37),(x,side*260,52),30,20,0,'RECT',group='outboard_root')
            m.add((x,side*260,52),(x,side*260,mount_z(x)),group='launcher_posts')
    for side in (-1,1):
        m.add(xyz(-128,side*145,-57),xyz(128,side*145,-57),group='launcher_rails')
        m.add((350,side*260,97),(440,side*260,mount_z(440)-10),6,6,0,'RECT')
    for x in (350,440):
        m.add(xyz(mount_x(x),-260,-57),xyz(mount_x(x),260,-57),group='launcher_crossbars')
        for side in (-1,1):
            m.node(xyz(mount_x(x),side*145,-57))
    # Cover frame at 213..233, top 402..422; centreline extends to centres of bars.
    for side in (-1,1):
        for a,b in zip((250,350,440),(350,440,570)):
            m.add((a,side*260,223),(b,side*260,223))
        for x in (350,440):
            m.add((x,side*260,223),(x,side*260,mount_z(x)))
        for x in (250,570): m.add((x,side*260,223),(x,side*260,412))
        m.add((250,side*260,412),(570,side*260,412))
    for x in (250,570):
        for z in (223,412): m.add((x,-260,z),(x,260,z))
    # Boss/bolt assembly idealised as bonded concentric beam segments.
    for x in (-115,115):
        for y in (-145,145):
            m.add(xyz(x,y,-57),xyz(x,y,-43),16,16,5.75,'PIPE',group='plate_boss')
            m.add(xyz(x,y,-43),xyz(x,y,43),12,12,4.35,'PIPE','STEEL','spacers')
    for upper in (False,True):
        local,triangles=mesh_panel(work,upper,mesh)
        used={n for tri in triangles for n in tri}
        ids={n:m.node(xyz(local[n][0],local[n][1],43 if upper else -43)) for n in used}
        for tri in triangles: m.shells.append(([ids[n] for n in tri],'PLATE','upper_plate' if upper else 'lower_plate'))
    # Load-bearing points must be embedded by Gmsh, never a nearest-node guess.
    flange=[]
    for y in (-129,129):
        points=[xyz(15*math.cos(a*math.pi/2),y+15*math.sin(a*math.pi/2),43) for a in range(4)]
        ns=[]
        for p in points:
            key=tuple(round(x,5) for x in p)
            if key not in m.index: raise RuntimeError('Missing embedded flange attachment')
            ns.append(m.index[key])
        flange.append(ns)
    m.mesh(mesh)
    return m,flange


def section_area(w,h,t,kind):
    if kind=='BOX': return w*h-(w-2*t)*(h-2*t)
    if kind=='PIPE': return math.pi*((w/2)**2-(w/2-t)**2)
    return w*h


def material_profiles(polymer):
    return {'POLYMER':(polymer['reference_elastic_modulus_gpa']*1000,polymer['poisson_ratio'],polymer['density_kg_m3']/1e12),
            'PLATE':(69000,.33,2700/1e12), 'STEEL':(210000,.30,7850/1e12)}


def scenario_loads(m,flange,scenario):
    acceleration=(0,0,-G*(2 if scenario=='carry_rear_2g' else 1))
    if scenario=='turn_reference': acceleration=(0,.5*G,-G)
    torque={'motor_torque_reference':620.,'motor_torque_peak_screen':2635.}.get(scenario,0.)
    wheel_mass=1.2 if scenario=='wheel_mass_1p2kg' else .9
    loads={}
    def add(n,d,v): loads[(n,d)]=loads.get((n,d),0)+v
    # Equivalent motor+wheel forces and CG moments at each flange; hub excluded.
    # All kg values converted to tonnes before mm/s² acceleration.
    for side,ns in zip((-1,1),flange):
        force=tuple((.49+wheel_mass)/1000*a for a in acceleration)
        moments=[0.,0.,0.]
        for mass,zz in ((.49,79.5),(wheel_mass,0.)):
            r=rotate((0,0,zz-43))
            f=tuple(mass/1000*a for a in acceleration)
            mm=cross(r,f)
            moments=[moments[k]+mm[k] for k in range(3)]
        rt=rotate((0,0,side*torque)); moments=[moments[k]+rt[k] for k in range(3)]
        # Distribute F and use four couples: no moments applied to a free point node.
        local_m=(moments[0]*math.cos(PITCH)+moments[2]*math.sin(PITCH),
                 moments[1],-moments[0]*math.sin(PITCH)+moments[2]*math.cos(PITCH))
        for i,n in enumerate(ns):
            theta=i*math.pi/2; x=15*math.cos(theta); y=15*math.sin(theta)
            extra=rotate((-local_m[2]*y/(4*15**2),local_m[2]*x/(4*15**2),
                          (local_m[0]*y-local_m[1]*x)/(2*15**2)))
            for k in range(3): add(n,k+1,force[k]/4+extra[k])
    if scenario=='turn_reference':
        for side in (-1,1):
            n=m.index[(300.,side*220.,37.)]
            add(n,1,side*10); add(n,6,-1300)
    return acceleration,loads


def write_deck(path,m,flange,polymer,scenario):
    profiles=material_profiles(polymer)
    lines=['*HEADING','Height frame ideal bonded beam/shell screen, NOT print release','*NODE,NSET=ALLN']
    lines += [f'{n},'+','.join(map(str,p)) for n,p in m.nodes.items()]
    lines += ['*ELEMENT,TYPE=B32R,ELSET=BEAMS']
    lines += [f'{i},'+','.join(map(str,b[0])) for i,b in enumerate(m.beams,1)]
    lines += ['*ELEMENT,TYPE=S6,ELSET=PANELS']
    lines += [f'{len(m.beams)+i},'+','.join(map(str,s[0])) for i,s in enumerate(m.shells,1)]
    for name,(e,nu,rho) in profiles.items():
        lines += [f'*MATERIAL,NAME={name}','*ELASTIC',f'{e},{nu}','*DENSITY',str(rho)]
    groups={}
    for i,(_,w,h,t,kind,mat,group,normal) in enumerate(m.beams,1):
        key=(w,h,t,kind,mat,tuple(round(v,7) for v in normal))
        groups.setdefault(key,[]).append(i)
    for i,((w,h,t,kind,mat,normal),eids) in enumerate(groups.items()):
        lines += [f'*ELSET,ELSET=B{i}']
        lines += [','.join(map(str,eids[j:j+12])) for j in range(0,len(eids),12)]
        dims=f'{h},{w},{t},{t},{t},{t}' if kind=='BOX' else (f'{w/2},{t}' if kind=='PIPE' else f'{h},{w}')
        lines += [f'*BEAM SECTION,ELSET=B{i},MATERIAL={mat},SECTION={kind}',dims,','.join(map(str,normal))]
    lines += ['*SHELL SECTION,ELSET=PANELS,MATERIAL=PLATE','8','*BOUNDARY']
    if scenario=='carry_rear_2g':
        supports=[m.index[(-165.,side*220.,37.)] for side in (-1,1)]
        lines += [f'{n},1,6' for n in supports]
    else:
        # Four vertical rail supports; minimum horizontal restraints to remove rigid motion.
        supports=[m.index[(x,side*220.,37.)] for x in (-65.,300.) for side in (-1,1)]
        lines += [f'{n},3,3' for n in supports]
        lines += [f'{supports[0]},1,2',f'{supports[1]},1,1']
    acc,loads=scenario_loads(m,flange,scenario)
    lines += ['*STEP','*STATIC','*DLOAD']
    for k,a in enumerate(acc):
        if a:
            direction=[0,0,0]; direction[k]=1 if a>0 else -1
            lines += [f'BEAMS,GRAV,{abs(a)},'+','.join(map(str,direction)),
                      f'PANELS,GRAV,{abs(a)},'+','.join(map(str,direction))]
    lines += ['*CLOAD']+[f'{n},{d},{v}' for (n,d),v in loads.items() if abs(v)>1e-10]
    lines += ['*NODE PRINT,NSET=ALLN','U,RF','*EL PRINT,ELSET=BEAMS','S',
              '*EL PRINT,ELSET=PANELS','S','*END STEP']
    path.write_text('\n'.join(lines)+'\n')
    return supports,acc,loads


def parse_result(path,m,flange,supports):
    displacements,rf={},{}; stress={}; mode=''
    for line in path.read_text().splitlines():
        low=line.lower()
        if 'displacements' in low: mode='u'
        elif 'forces' in low: mode='rf'
        elif 'stresses' in low: mode='s'
        fields=line.split()
        try:
            if mode in ('u','rf') and len(fields)==4:
                (displacements if mode=='u' else rf)[int(fields[0])]=tuple(map(float,fields[1:]))
            elif mode=='s' and len(fields)>=8:
                eid=int(fields[0]); sx,sy,sz,xy,xz,yz=map(float,fields[2:8])
                vm=math.sqrt(((sx-sy)**2+(sy-sz)**2+(sz-sx)**2)/2+3*(xy*xy+xz*xz+yz*yz))
                group=m.beams[eid-1][6] if eid<=len(m.beams) else m.shells[eid-len(m.beams)-1][2]
                stress[group]=max(stress.get(group,0),vm)
        except ValueError: continue
    if set(displacements)!=set(m.nodes) or not {'upper_plate','lower_plate'}<=set(stress):
        raise RuntimeError('Missing displacement or plate stress output')
    u=[tuple(sum(displacements[n][k] for n in ns)/len(ns) for k in range(3)) for ns in flange]
    return {'max_node_displacement_mm':max(math.sqrt(sum(v*v for v in p)) for p in displacements.values()),
            'max_flange_displacement_mm':max(math.sqrt(sum(v*v for v in p)) for p in u),
            'relative_flange_translation_mm':math.dist(*u),
            'flange_mean_translation_mm':u,
            'peak_integration_stress_mpa_by_group':stress,
            'reaction_sum_n':[sum(rf[n][k] for n in supports) for k in range(3)]}


def mass_and_force(m,polymer,acc,loads):
    profiles=material_profiles(polymer); masses={}
    for ids,w,h,t,kind,mat,group,_ in m.beams:
        volume=math.dist(m.nodes[ids[0]],m.nodes[ids[2]])*section_area(w,h,t,kind)
        masses[group]=masses.get(group,0)+volume*profiles[mat][2]*1000
    for ids,mat,group in m.shells:
        a,b,c=(m.nodes[n] for n in ids[:3])
        ab=[b[k]-a[k] for k in range(3)]; ac=[c[k]-a[k] for k in range(3)]
        area=.5*math.sqrt(sum(v*v for v in cross(ab,ac)))
        masses[group]=masses.get(group,0)+area*8*profiles[mat][2]*1000
    total=sum(masses.values())
    force=[total/1000*a for a in acc]
    for (_,d),v in loads.items():
        if d<=3: force[d-1]+=v
    return masses,force


def run(work,meshes=(20,10,5),scenarios=SCENARIOS,materials=None,reuse=False):
    all_materials=json.loads((ROOT/'config/materials.json').read_text())['materials']
    materials=materials or tuple(all_materials)
    cases=[]
    for mesh in meshes:
        model,flange=build(work,mesh)
        for name in materials:
            profile=all_materials[name]
            for scenario in scenarios:
                stem=f'height_{name.replace("-","_")}_{scenario}_{mesh:g}'
                deck=work/f'{stem}.inp'
                old=deck.read_bytes() if deck.exists() else None
                supports,acc,loads=write_deck(deck,model,flange,profile,scenario)
                log=work/f'{stem}.log'
                cached=(reuse and old==deck.read_bytes() and log.exists() and
                        'Job finished' in log.read_text() and '*ERROR' not in log.read_text().upper()
                        and (work/f'{stem}.dat').exists())
                if not cached:
                    p=subprocess.run([str(SOLVER),'-i',stem],cwd=work,env=runtime_env(),
                                     capture_output=True,text=True,timeout=90)
                    log.write_text(p.stdout+p.stderr)
                    if p.returncode or '*ERROR' in p.stdout.upper(): raise RuntimeError(f'Solver failed {stem}')
                result=parse_result(work/f'{stem}.dat',model,flange,supports)
                masses,force=mass_and_force(model,profile,acc,loads)
                err=math.dist(result['reaction_sum_n'],[-f for f in force])/max(1,math.sqrt(sum(f*f for f in force)))
                if err>.01: raise RuntimeError(f'Force balance >1% for {stem}: {err}')
                cases.append({'material':name,'scenario':scenario,'mesh_mm':mesh,
                              'beam_elements':len(model.beams),'shell_elements':len(model.shells),
                              'model_mass_kg_by_group':masses,'applied_resultant_force_n':force,
                              'force_balance_relative_error':err,**result})
                print(f'{stem}: flange {result["max_flange_displacement_mm"]:.5f} mm, balance {err:.3g}',flush=True)
    convergence=[]
    if len(meshes)>1:
        for name in materials:
            for scenario in scenarios:
                pair=sorted([c for c in cases if c['material']==name and c['scenario']==scenario],key=lambda c:c['mesh_mm'])[:2]
                change=abs(pair[0]['max_flange_displacement_mm']-pair[1]['max_flange_displacement_mm'])/max(pair[0]['max_flange_displacement_mm'],1e-12)
                convergence.append({'material':name,'scenario':scenario,'relative_change_finest_two_meshes':change})
    return {'status':'PRELIMINARY_IDEAL_BONDED_SCREEN_NOT_RELEASE',
            'cad_sha256':hashlib.sha256(CAD.read_bytes()).hexdigest(),
            'runner_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            'material_profiles_sha256':hashlib.sha256((ROOT/'config/materials.json').read_bytes()).hexdigest(),
            'solver':'CalculiX B32R hollow beams and S6 quadratic shell plates',
            'geometry':{'pitch_deg':20,'nip_x_mm':410,'nip_z_mm':298,'plate_thickness_mm':8,
                        'plate_anchor_pattern_mm':[230,290],'spacer_outer_d_mm':12,'spacer_bore_mm':3.3},
            'assumptions':{'plate':'User-confirmed 8 mm aluminium plate architecture; alloy unconfirmed; E=69 GPa, nu=.33, density=2700 kg/m3',
                           'spacers':'Unconfirmed steel E=210 GPa, density=7850 kg/m3; ideal bonded joints',
                           'motor_mass_kg_each':.49,'wheel_mass_kg_each':.9,'hub_mass_kg':0,
                           'turn':'Prescribed 0.5g lateral plus old opposing ±10 N / -1300 Nmm front loads',
                           'carry':'2g downward, rear chassis corners clamped: artificial rear-only lifting screen',
                           'torque_reference_nm_each':.62,'torque_peak_screen_nm_each':2.635,
                           'wheel_mass_sensitivity_kg_each':1.2},
            'validation':{'flange_displacement_convergence':convergence,
                          'convergence_gate':'PASS' if convergence and max(c['relative_change_finest_two_meshes'] for c in convergence)<.02 else 'NOT_VERIFIED',
                          'max_force_balance_relative_error':max(c['force_balance_relative_error'] for c in cases)},
            'limitations':['Isotropic provisional polymer, not actual printed PETG/CF allowables',
                           'Ideal rigid joints; no sleeve slip, bolt preload, pullout or socket compliance',
                           'Beam centreline idealisation; W, Gamma, motor pods, local notch stresses omitted',
                           '8 mm metal plate/spacer architecture user-confirmed; alloy, spacer stock and fasteners unmeasured; bolt holes treated as points',
                           'Lower relief uses overlapping sampled disks instead of exact circle hulls',
                           'No mass for intake, drive motors/wheels, LiDAR, skins, hub, basket or battery',
                           'No rotating imbalance, resonance, transient ball contact, fatigue, creep or bearing capacity',
                           'Four ideal vertical rail supports are not a tyre/contact/suspension model',
                           'Integration-point stress peaks diagnostic; not hole stress or a certified safety factor'],
            'cases':cases}


if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('--work',type=Path,default=ROOT/'fea/work/height_frame')
    p.add_argument('--output',type=Path,default=ROOT/'results/height_frame/report.json')
    p.add_argument('--meshes',type=float,nargs='+',default=[20,10,5])
    p.add_argument('--materials',nargs='+',choices=['PLA','PETG','PETG-CF'])
    p.add_argument('--scenarios',nargs='+',choices=SCENARIOS,default=SCENARIOS)
    p.add_argument('--reuse',action='store_true',help='Reuse completed solver output ONLY when deck bytes match')
    args=p.parse_args()
    if any(v<=0 for v in args.meshes): p.error('Mesh lengths must be positive')
    args.work.mkdir(parents=True,exist_ok=True)
    report=run(args.work,args.meshes,args.scenarios,args.materials,args.reuse)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2)+'\n')
