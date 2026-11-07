#!/usr/bin/env python3
"""Read-only analytic envelopes; not CAD candidates or a replacement scene gate."""
import ast, json
from pathlib import Path
from math import sin,cos,radians,degrees,atan2,sqrt
import numpy as np
import trimesh
import manifold3d as m

ROOT=Path(__file__).resolve().parents[2]
DEST=Path(__file__).parent/'p293g-rear-brace-results'
SCENE=ROOT/'cad/independent-check/out/scene'
P=np.array([387.,-205.,51.837363636])
B=np.array([260.,-205.,220.])
# Reuse exactly the historical P2-93f A envelope functions, without executing
# its script body or overwriting its historical results.
source=(Path(__file__).parent/'p293f-analysis/p293f-analysis.py').read_text()
tree=ast.parse(source)
for node in tree.body:
    if isinstance(node,ast.FunctionDef) and node.name in ('place','components'):
        exec(compile(ast.Module(body=[node],type_ignores=[]),str(Path(__file__)), 'exec'),globals())

def beam(a,b,width=20,trim_a=0,trim_b=0):
    a=np.array(a,dtype=float);b=np.array(b,dtype=float)
    u=(b-a)/np.linalg.norm(b-a);a=a+trim_a*u;b=b-trim_b*u
    # Cross-section20x20; one direction mainly Y, one normal in XZ.
    n=np.cross([0,1,0],u);n/=np.linalg.norm(n);v=np.cross(u,n)
    return m.Manifold.cube([width,width,np.linalg.norm(b-a)],True).transform(np.column_stack([n,v,u,(a+b)/2]))

def cube(p,size):return m.Manifold.cube(size,True).translate(p)
def near(a,b):
    aa=a.bounding_box();bb=b.bounding_box()
    return all(aa[i]<=bb[i+3] and bb[i]<=aa[i+3] for i in range(3))
def vol(a,b):return float((a^b).volume()) if near(a,b) else 0.

def main():
    DEST.mkdir(exist_ok=True)
    report=json.loads((SCENE/'report.json').read_text())
    names=[f'rail_extension_v3[side={s}]' for s in (-1,1)]+[f'clamp_sleeve_x[side={s}xx=35]' for s in (-1,1)]
    solids={}
    for name in report['bodies']['0']:
        path=SCENE/report['files'][name+'@0']
        mesh=trimesh.load(path,force='mesh')
        solids[name]=m.Manifold(m.Mesh(np.asarray(mesh.vertices,np.float32),np.asarray(mesh.faces,np.uint32)))
    holes=[]
    for name in names:
        s=-1 if 'side=-1' in name else 1
        for dx in (0,5):
            probe=m.Manifold.cylinder(32,2,2,64).translate([65+dx,s*220,21])
            holes.append({'body':name,'x':65+dx,'Y':s*220,'probe_D_mm':4,'intersection_mm3':vol(solids[name],probe)})
    angles=list(range(-10,21))
    tilt={a:components(B,(30,110),a)[-1] for a in angles}
    # Actual new stop solids do not exist. Explicitly reserved envelopes only:
    # 16mm square console bars + 20x16x16 moving striker blocks, based on F §6.
    fixed={}
    for side in (-1,1):
        for label,root,contact in [
          ('lower',[390,side*228,245],[413.047,side*205,199.558]),
          ('upper',[350.06,side*228,207.18],[335.697,side*205,192.791])]:
            fixed[f'{label}_stop_console_reservation_{side}']=beam(root,contact,16)
    strikers={}
    for a in angles:
        t=radians(a)
        strikers[a]={f'striker_reservation_{s}':cube([387-150*sin(t),s*205,P[2]+150*cos(t)],[20,16,16]) for s in (-1,1)}
    proposals=[('high_220',372,232,220),('low_220',320,185,220),
               ('high_outboard',372,232,250),('low_outboard',320,185,250)]
    proposals += [('low_inboard',320,185,200),('high_inboard',372,232,200)]
    proposals += [('low_203',320,185,203),('low_204',320,185,204),('low_204_5',320,185,204.5)]
    # Search starts lie on the actual upper-member centreline +6mm Z port offset.
    proposals += [(f'search_{x}_{y}',x,120+(x-258)*125/132+6,y) for y in (200,205,210) for x in (300,320,340,350,365)]
    results=[]
    for label,x,z,y in proposals:
        braces={s:beam([65,s*220,60],[x,s*y,z],20,20,20) for s in (-1,1)}
        length=float(np.linalg.norm(np.array([x-65,y-220,z-60])))
        hits=[];gaps={};context=[]
        for a in angles:
            for s,br in braces.items():
                for target,obj in {**tilt[a],**fixed,**strikers[a]}.items():
                    v=vol(br,obj)
                    if v>1e-4:hits.append({'angle':a,'side':s,'target':target,'mm3':v})
                    if target in ('motor','bearings','screw','belt_keepout'):
                        gaps[target]=min(gaps.get(target,1e9),float(br.min_gap(obj,100)))
        for s,br in braces.items():
            for target,obj in solids.items():
                v=vol(br,obj)
                if v>1e-4:context.append({'side':s,'target':target,'mm3':v})
        results.append({'name':label,'upper_mm':[x,-y,z],'rail_mm':[65,-220,60],
                        'centre_length_mm':length,'angle_XZ_deg':degrees(atan2(z-60,x-65)),
                        'beam_trim_mm_each_end':20,'beam20_hits':hits,'minimum_gaps_mm':gaps,'baseline_context_hits_at0':context})
        print(label,'tilt/reservation hits',len(hits),'context',[(h['target'],round(h['mm3'],2)) for h in context],flush=True)
    data={'status':'ANALYSIS_ENVELOPES_NOT_CANDIDATES','hole_probes':holes,'routes':results,
          'notes':['NO volume exclusions. Body ends physically shortened20mm for future node adapters, not QA masking.',
                   'New stop consoles/strikers do not exist: reservations are explicitly surrogate, not exact-solid PASS.',
                   'Historical thrust box retained as diagnostic, not a requirement to buy thrust bearing.',
                   '31 samples1deg; no continuous-motion proof. End adapters, splice and rail pad are not yet solids.']}
    (DEST/'clearance.json').write_text(json.dumps(data,indent=2)+'\n')

if __name__=='__main__':main()
