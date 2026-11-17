#!/usr/bin/env python3
"""Independent check: report section islands with no material in the layer below
(floating starts that a slicer prints in mid-air).  Usage:
  check-floating-islands.py cad/check-stl-slices.py MESH.stl
Output: one FLOAT line per unsupported island; no output = none found."""
import sys, importlib.util
spec=importlib.util.spec_from_file_location("cs", sys.argv[1]); cs=importlib.util.module_from_spec(spec); spec.loader.exec_module(cs)
from pathlib import Path
mesh=list(cs.triangles(Path(sys.argv[2])))
zmax=max(v[2] for t in mesh for v in t)
def outers(loops):
    # outer loops = those contained by an even number of other loops
    res=[]
    for i,l in enumerate(loops):
        p=cs.interior_point(l)
        depth=sum(cs.contains(o,l[0]) for j,o in enumerate(loops) if j!=i)
        if depth%2==0: res.append(l)
    return res
def filled(loops,p): return sum(cs.contains(l,p) for l in loops)%2==1
prev=None; z=0.1
while z<zmax:
    try: loops=cs.section_loops(mesh,z)
    except Exception: loops=cs.section_loops(mesh,z+0.0013)
    if prev is not None:
        for l in outers(loops):
            # island is floating if none of its vertices (sampled) lie on material of previous layer
            xs=[q[0] for q in l]; ys=[q[1] for q in l]
            pts=[(x0,y0) for i in range(41) for j in range(41)
                 for x0 in [min(xs)+(max(xs)-min(xs))*i/40] for y0 in [min(ys)+(max(ys)-min(ys))*j/40]
                 if cs.contains(l,(x0,y0)) and filled(loops,(x0,y0))]
            if pts and not any(filled(prev,p) for p in pts):
                xs=[q[0] for q in l]; ys=[q[1] for q in l]
                print(f"FLOAT Z{z:.2f} area {abs(cs.polygon_area(l)):.1f} x {min(xs):.1f}..{max(xs):.1f} y {min(ys):.1f}..{max(ys):.1f}")
    prev=loops; z+=0.2
