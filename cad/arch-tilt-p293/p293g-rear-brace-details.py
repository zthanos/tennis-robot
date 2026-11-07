#!/usr/bin/env python3
"""Analysis-only sleeve/tool reservations for the selected low/Y204 route."""
import importlib.util,json,argparse
from pathlib import Path
import numpy as np
import trimesh
import manifold3d as m
here=Path(__file__).parent
parser=argparse.ArgumentParser();parser.add_argument('--z',type=float,default=185.)
parser.add_argument('--splice',choices=['sleeve','plates'],default='plates')
args=parser.parse_args()
spec=importlib.util.spec_from_file_location('clearance',here/'p293g-rear-brace-clearance.py')
c=importlib.util.module_from_spec(spec);spec.loader.exec_module(c)
report=json.loads((c.SCENE/'report.json').read_text());solids={}
for name in report['bodies']['0']:
    t=trimesh.load(c.SCENE/report['files'][name+'@0'],force='mesh')
    solids[name]=m.Manifold(m.Mesh(np.asarray(t.vertices,np.float32),np.asarray(t.faces,np.uint32)))
objects={};selected=[]
for s in (-1,1):
    r=np.array([65,s*220,60.]);j=np.array([320,s*204,args.z]);u=(j-r)/np.linalg.norm(j-r)
    # Asymmetric split, shifted above/beyond the basket X225. Two pieces,
    # NOT a midpoint splice that collides with the basket/front system.
    joint_s=np.linalg.norm(j-r)-64;mid=r+joint_s*u
    if args.splice=='sleeve':
        splice=c.beam(mid-35*u,mid+35*u,28)-c.beam(mid-37*u,mid+37*u,20.4)
    else:
        n=np.cross([0,1,0],u);n/=np.linalg.norm(n);v=np.cross(u,n)
        parts=[m.Manifold.cube([4,20,60],True).transform(np.column_stack([n,v,u,mid+sgn*12*n])) for sgn in (-1,1)]
        splice=parts[0]+parts[1]
    objects[f'splice_{s}']=splice
    objects[f'top_socket_reservation_{s}']=m.Manifold.cylinder(60,6,6,48).translate([65,s*220,76])
    objects[f'underside_wrench_reservation_{s}']=c.cube([65,s*220,15],[24,40,8])
    br=c.beam(r,j,20,20,20)
    objects[f'brace_{s}']=br
    selected.append({'side':s,'centre_length_mm':float(np.linalg.norm(j-r)),
      'upper_point_mm':j.tolist(),'splice_station_mm_from_R':float(joint_s),
      'shaft_piece_lengths_mm':[float(joint_s-20-.2),43.8],
      'beam_splice_raw_intersection_mm3':c.vol(br,splice),
      'top_socket_vs_brace_mm3':c.vol(br,objects[f'top_socket_reservation_{s}']),
      'arch_gap_mm':min(float(br.min_gap(v,100)) for n,v in solids.items() if n.startswith(('p293d_knee','p293d_crown'))),
      'basket_gap_mm':min(float(br.min_gap(v,100)) for n,v in solids.items() if n.startswith('p271'))})
hits=[]
for name,obj in objects.items():
    for target,solid in solids.items():
        v=c.vol(obj,solid)
        if v>1e-4:hits.append({'object':name,'target':target,'angle':0,'mm3':v})
    for angle in range(-10,21):
        for centres in (57,60,63):
            for target,solid in c.components(c.B,(30,110),angle,centre=centres)[-1].items():
                v=c.vol(obj,solid)
                if v>1e-4:hits.append({'object':name,'target':target,'angle':angle,'GT2_centres':centres,'mm3':v})
        for side in (-1,1):
            for label,root,contact in [
              ('lower',[390,side*228,245],[413.047,side*205,199.558]),
              ('upper',[350.06,side*228,207.18],[335.697,side*205,192.791])]:
                v=c.vol(obj,c.beam(root,contact,16))
                if v>1e-4:hits.append({'object':name,'target':label+'_console_reservation','angle':angle,'mm3':v})
            t=np.radians(angle)
            striker=c.cube([387-150*np.sin(t),side*205,c.P[2]+150*np.cos(t)],[20,16,16])
            v=c.vol(obj,striker)
            if v>1e-4:hits.append({'object':name,'target':'striker_reservation','angle':angle,'mm3':v})
        if 'moving_module_concept' in solids:
            from math import sin,cos,radians
            t=radians(angle);rot=np.array([[cos(t),0,-sin(t)],[0,1,0],[sin(t),0,cos(t)]])
            p=np.array([387,0,c.P[2]]);transform=np.column_stack([rot,p-rot@p])
            v=c.vol(obj,solids['moving_module_concept'].transform(transform))
            if v>1e-4:hits.append({'object':name,'target':'moving_module_concept','angle':angle,'mm3':v})
data={'status':'SURROGATE_DETAIL_ENVELOPES_NOT_CAD','splice':args.splice,'selected':selected,'raw_hits':hits,
      'scope':['Splice: either hollow sleeve28x28x70 or two real4normal x20Y x60long fishplates touching beam faces; no QA exclusions.',
        'Two shaft pieces with0.4mm axial end gap; no overlapping solid splice.',
        'Top tool D12 Z76..136; underside24x40x8 aroundZ15; real fastener CAD not present.',
        'R65/Z60 is a virtual load line, NOT a Y-pin crossing the vertical rail bolt.',
        'Rail pad,end nodes and bolts not final solids; their access/edge ligaments remain pending.']}
(c.DEST/f'details-z{args.z:g}-{args.splice}.json').write_text(json.dumps(data,indent=2)+'\n')
print(json.dumps(data,indent=2))
