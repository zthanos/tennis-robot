#!/usr/bin/env python3
"""Check add-on keeper only; do not modify or re-export the host sleeve."""
import hashlib
import json
from pathlib import Path
import subprocess
from measure_middle_t_design import ROOT, mesh_stats
from check_front_print_fit import topology

CAD=ROOT/'cad/drive-motor-mount/m3-side-nut-keeper.scad'
WORK=ROOT/'engineering/chassis-strength-bench/fea/work/nut_keeper'
OUTPUT=ROOT/'engineering/chassis-strength-bench/results/nut_keeper/report.json'


def check():
    WORK.mkdir(parents=True,exist_ok=True)
    checks=[]
    for name in ('qa_sleeve','qa_front_sleeve','qa_nut'):
        output=WORK/f'{name}_checked.stl'
        result=subprocess.run(['/snap/bin/openscad-nightly','-o',str(output),
            '-D',f'part="{name}"',str(CAD)],capture_output=True,text=True,timeout=60)
        log=result.stdout+result.stderr
        (WORK/f'{name}.log').write_text(log)
        empty='Current top level object is empty.' in log
        if 'ERROR:' in log or (result.returncode and not empty):
            raise RuntimeError(log)
        # Planar tab/host contact can be emitted as zero-volume surface.
        volume=0.0 if empty else mesh_stats(output.read_bytes())['solid_volume_cm3']
        if volume>1e-8:
            raise RuntimeError(f'Keeper collision: {name}: {volume} cm3')
        checks.append({'check':name,'overlap_volume_cm3':volume,'empty':empty})
    path=CAD.parent/'nut-keeper-stl/sleeve.stl'
    data=path.read_bytes()
    stats={**mesh_stats(data),**topology(data)}
    if stats['positive_volume_shells']!=1 or stats['nonmanifold_edge_count'] or stats['degenerate_triangles']:
        raise RuntimeError(f'Invalid keeper STL: {stats}')
    return {'status':'ADD_ON_CAD_CHECKED_PHYSICAL_RETENTION_UNVERIFIED',
        'source_sha256':hashlib.sha256(CAD.read_bytes()).hexdigest(),
        'stl_sha256':hashlib.sha256(data).hexdigest(),
        'checks':checks,'mesh':stats,
        'assumptions':{'nut_across_flats_mm':5.5,'nut_thickness_preview_mm':2.4,
            'entrance_to_screw_axis_mm':9,'channel_height_mm':3.7},
        'limits':['No positive latch in existing housing',
                  'Physical insertion, nut centering and resistance to falling out require trial',
                  'No change to existing load-bearing nut seat or screw stack',
                  'Applies to current 43 mm wide sleeves; not certification of older exports']}


if __name__=='__main__':
    report=check()
    OUTPUT.parent.mkdir(parents=True,exist_ok=True)
    OUTPUT.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({'status':report['status'],'checks':report['checks'],
                      'envelope_mm':report['mesh']['envelope_mm']},indent=2))
