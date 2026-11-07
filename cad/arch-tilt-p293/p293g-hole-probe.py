#!/usr/bin/env python3
"""Read existing scene meshes only; no modifications to gate or geometry."""
import json
from pathlib import Path
import numpy as np
import trimesh
import manifold3d as m
root=Path(__file__).resolve().parents[2]
scene=root/'cad/independent-check/out/scene'
report=json.loads((scene/'report.json').read_text())
names=['rail_extension_v3[side=-1]','clamp_sleeve_x[side=-1xx=200]']
probes={
 # Existing gate mesh has 8-sided nominal D4.5 bores, inscribed D4.157.
 # D4 verifies the clear M4 core without changing/subtracting any body.
 'vertical_X170_Y-220_Z21..53_D4':m.Manifold.cylinder(32,2,2,64).translate([170,-220,21]),
 'side_X170_Z37_Y-194..-246_D4':m.Manifold.cylinder(52,2,2,64).rotate([90,0,0]).translate([170,-194,37]),
 # Positive controls displaced 5mm off each bore into the sleeve web.
 'control_vertical_X175_Y-220':m.Manifold.cylinder(32,2.2,2.2,64).translate([175,-220,21]),
 'control_side_X175_Z37':m.Manifold.cylinder(52,2.2,2.2,64).rotate([90,0,0]).translate([175,-194,37]),
}
rows=[]
for name in names:
    path=scene/report['files'][name+'@0']
    mesh=trimesh.load(path,force='mesh')
    solid=m.Manifold(m.Mesh(np.asarray(mesh.vertices,np.float32),np.asarray(mesh.faces,np.uint32)))
    for pn,p in probes.items():
        rows.append({'body':name,'mesh':str(path.relative_to(root)),'probe':pn,
                     'raw_intersection_mm3':float((solid^p).volume())})
data={'note':'Existing exported scene only. D4 core in nominal D4.5 octagonal bore, no volume exclusions. Side rail bore is NOT clear on -Y side. Not a new candidate collision gate.', 'rows':rows}
dest=Path(__file__).parent/'p293g-results/hole-probe.json'
dest.write_text(json.dumps(data,indent=2)+'\n')
print(json.dumps(data,indent=2))
