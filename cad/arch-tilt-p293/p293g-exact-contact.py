#!/usr/bin/env python3
"""Raw native-CAD confirmation of coplanar M4 washer contact, no volume masks.

Rigid transforms preserve intersection: G(A intersect B)=G(A) intersect G(B).
Intersect actual CAD in the common moving frame BEFORE lossy world-STL export.
Retain the global mesh flags unchanged. A deliberate -0.05mm penetration is
a mandatory positive control, so a real overlap cannot pass as 'mating'.
"""
import json,subprocess,tempfile,os
from pathlib import Path
import trimesh
H=Path(__file__).resolve().parent;O=H/'p293g-candidate-results'
binary=os.environ.get('OPENSCAD_BIN','/snap/bin/openscad-nightly')
results={}
# Snap's private /tmp is not the host's /tmp: use a repo-local owned temp dir.
with tempfile.TemporaryDirectory(prefix='.exact-contact-',dir=O) as work:
 for label,offset in [('nominal',0.),('penetration_control',-.05)]:
  source=f'''use <{H/'p293g-tilt-candidate.scad'}>
// Both full solids in their common moving frame. NO masks or exemptions.
module real_washer(){{difference(){{cylinder(d=10,h=1,$fn=48);translate([0,0,-1])cylinder(d=4.3,h=3,$fn=48);}}}}
intersection(){{
 p293g_nut_arm();
 union()for(u=[-15,5],y=[-224,-184])translate([u,y,{59.1+offset:.8f}])real_washer();
}}
'''
  path=Path(work)/f'{label}.scad';stl=Path(work)/f'{label}.stl';path.write_text(source)
  run=subprocess.run([binary,'--backend=CGAL','-o',str(stl),str(path)],text=True,capture_output=True)
  log=run.stdout+run.stderr
  empty='Current top level object is empty' in log and 'ERROR:' not in log
  volume=float(abs(trimesh.load(stl,force='mesh').volume)) if stl.exists() else 0.
  results[label]=dict(offset_mm=offset,returncode=run.returncode,EMPTY=empty,volume_mm3=volume,raw_log=log,raw_CAD_source=source)
results['method']='Raw actual-CAD intersection; affine invariance, no solids removed before checking. CGAL exact-predicate backend, not independent rounded world STL planes.'
results['original_global_mesh_flags']=json.loads((O/'hardware-probe.json').read_text())['hits']
results['PASS']=results['nominal']['EMPTY'] and results['nominal']['volume_mm3']==0 and results['penetration_control']['volume_mm3']>1
(O/'exact-washer-contact.json').write_text(json.dumps(results,indent=2)+'\n')
print({key:results[key] for key in ['PASS','method']})
print({key:{k:v for k,v in results[key].items() if k not in ['raw_log','raw_CAD_source']} for key in ['nominal','penetration_control']})
assert results['PASS'], 'Native CAD contact check or positive penetration control failed'
