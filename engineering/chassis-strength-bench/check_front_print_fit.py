#!/usr/bin/env python3
"""Exact CAD intersections and STL topology, not physical-fit certification."""
import argparse
import hashlib
import json
from pathlib import Path
import struct
import subprocess
from measure_middle_t_design import ROOT, mesh_stats

CAD=ROOT/'cad/modular-test-chassis/modular-chassis-v2.scad'
CHECKS=('qa_front_sleeves','qa_front_lock_bores','qa_gamma_holes',
        'front_brace_wheel_interference','front_brace_ramp_interference')


def topology(data):
    count=struct.unpack_from('<I',data,80)[0] if len(data)>=84 else 0
    if len(data)==84+50*count:
        vertices=[]
        for i in range(count):
            record=struct.unpack_from('<12fH',data,84+50*i)
            vertices.extend(tuple(record[j:j+3]) for j in (3,6,9))
    else:
        vertices=[tuple(map(float,line.split()[1:])) for line in data.decode().splitlines()
                  if line.strip().startswith('vertex ')]
    parents={}
    def find(v):
        parents.setdefault(v,v)
        while parents[v]!=v:
            parents[v]=parents[parents[v]]
            v=parents[v]
        return v
    degenerate=0
    for i in range(0,len(vertices),3):
        triangle=[tuple(round(x,6) for x in v) for v in vertices[i:i+3]]
        a,b,c=triangle
        ab=[b[k]-a[k] for k in range(3)]
        ac=[c[k]-a[k] for k in range(3)]
        cross=[ab[1]*ac[2]-ab[2]*ac[1],ab[2]*ac[0]-ab[0]*ac[2],ab[0]*ac[1]-ab[1]*ac[0]]
        if sum(x*x for x in cross)<1e-16:
            degenerate+=1
        root=find(a)
        for v in (b,c):
            parents[find(v)]=root
    groups={}
    for v in parents:
        groups.setdefault(find(v),[]).append(v)
    volumes={root:0.0 for root in groups}
    for i in range(0,len(vertices),3):
        a,b,c=vertices[i:i+3]
        root=find(tuple(round(x,6) for x in a))
        volumes[root]+=(a[0]*(b[1]*c[2]-b[2]*c[1])
                      +a[1]*(b[2]*c[0]-b[0]*c[2])
                      +a[2]*(b[0]*c[1]-b[1]*c[0]))/6000
    return {'connected_components':len(groups),
            'positive_volume_shells':sum(v>1e-8 for v in volumes.values()),
            'shell_signed_volume_cm3':list(volumes.values()),
            'component_bounds_mm':[
                [[min(v[k] for v in points) for k in range(3)],
                 [max(v[k] for v in points) for k in range(3)]]
                for points in groups.values()],
            'degenerate_triangles':degenerate}


def check(work):
    work.mkdir(parents=True,exist_ok=True)
    intersections=[]
    for name in CHECKS:
        result=subprocess.run(['/snap/bin/openscad-nightly','-o',str(work/f'{name}.stl'),
            '-D',f'part="{name}"',str(CAD)],text=True,capture_output=True,timeout=120)
        log=result.stdout+result.stderr
        (work/f'{name}.log').write_text(log)
        empty='Current top level object is empty.' in log and 'ERROR:' not in log
        if not empty:
            raise RuntimeError(f'Intersection or compiler failure: {name}; {log}')
        intersections.append({'check':name,'empty':True})
    meshes=[]
    for path in sorted((CAD.parent/'stl-v2').glob('*.stl')):
        data=path.read_bytes()
        stats={**mesh_stats(data),**topology(data)}
        # A closed interior air pocket has a separate negative-volume shell;
        # it is not a detached solid. Require exactly one positive solid shell.
        if stats['nonmanifold_edge_count'] or stats['positive_volume_shells']!=1 or stats['degenerate_triangles']:
            raise RuntimeError(f'Invalid solid: {path.name}: {stats}')
        meshes.append({'part':path.stem,'sha256':hashlib.sha256(data).hexdigest(),**stats})
    indexed={row['part']:row for row in meshes}
    for side in ('left','right'):
        envelope=indexed[f'front_motor_gamma_{side}']['envelope_mm']
        if any(abs(a-b)>0.01 for a,b in zip(envelope,(255,85,30))):
            raise RuntimeError(f'Unexpected front part envelope: {envelope}')
    return {'status':'CAD_FIT_CHECKED_PHYSICAL_FRONT_SLEEVE_COUPON_REQUIRED',
        'cad_sha256':hashlib.sha256(CAD.read_bytes()).hexdigest(),
        'intersections':intersections,'stls':meshes,
        'nominal_fit':{'main_socket_mm':[43.6,23.6],'main_sleeve_mm':[43,23],
            'front_socket_mm':[43.6,13.6],'front_sleeve_mm':[43,13],
            'clearance_per_face_mm':0.3,'main_insertion_mm':60,'front_insertion_mm':30,
            'motor_nut_access_vertical_gap_mm':2.3},
        'limits':['No physical sleeve-fit, torque or strength test',
            'Rear corner sleeves have no locking holes: full chassis is NOT print-released',
            'Electronics board mounting holes remain a layout placeholder',
            '255 mm front part leaves only 1 mm on a nominal 256 mm bed; brim and machine exclusion zones need slicer check',
            'Gamma hole test covers a 1 mm slice above the mating plane, not the full upright',
            'Wheel checks use specified envelopes, not measured wheel/motor geometry']}


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--work',type=Path,default=ROOT/'engineering/chassis-strength-bench/fea/work/front_fit')
    parser.add_argument('--output',type=Path,default=ROOT/'engineering/chassis-strength-bench/results/front_print_fit/report.json')
    args=parser.parse_args()
    report=check(args.work)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({'status':report['status'],'intersections':report['intersections'],
                      'stl_count':len(report['stls'])},indent=2))
