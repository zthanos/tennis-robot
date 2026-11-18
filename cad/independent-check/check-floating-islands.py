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
def bbox(loop):
    xs=[p[0] for p in loop]; ys=[p[1] for p in loop]
    return min(xs),min(ys),max(xs),max(ys)
def boxes_overlap(a,b):
    return a[0] <= b[2] and b[0] <= a[2] and a[1] <= b[3] and b[1] <= a[3]
def orient(a,b,c):
    return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])
def proper_cross(a,b,c,d):
    # A proper crossing has material on both sides; endpoint-only tangency is
    # deliberately not accepted as layer support.
    return orient(a,b,c)*orient(a,b,d) < 0 and orient(c,d,a)*orient(c,d,b) < 0
def samples(loop):
    return loop + [((a[0]+b[0])/2,(a[1]+b[1])/2)
                   for a,b in zip(loop,loop[1:]+loop[:1])]
def overlaps_below(current,loops,previous):
    current_box=bbox(current)
    if any(filled(previous,p) for p in samples(current)+[cs.interior_point(current)]):
        return True
    for old in previous:
        if not boxes_overlap(current_box,bbox(old)): continue
        if any(cs.contains(current,p) and filled(loops,p) for p in samples(old)):
            return True
        if any(proper_cross(a,b,c,d)
               for a,b in zip(current,current[1:]+current[:1])
               for c,d in zip(old,old[1:]+old[:1])):
            return True
    return False
prev=None; z=0.1
while z<zmax:
    try: loops=cs.section_loops(mesh,z)
    except Exception: loops=cs.section_loops(mesh,z+0.0013)
    if prev is not None:
        for l in outers(loops):
            # Exact boundary/interior overlap replaces the former 41x41
            # raster.  It preserves the no-material-below definition without
            # making every export spend minutes sampling empty bounding boxes.
            if not overlaps_below(l,loops,prev):
                xs=[q[0] for q in l]; ys=[q[1] for q in l]
                print(f"FLOAT Z{z:.2f} area {abs(cs.polygon_area(l)):.1f} x {min(xs):.1f}..{max(xs):.1f} y {min(ys):.1f}..{max(ys):.1f}")
    prev=loops; z+=0.2
