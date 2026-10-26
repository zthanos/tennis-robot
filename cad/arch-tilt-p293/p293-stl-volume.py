#!/usr/bin/env python3
"""Print signed-tetrahedron absolute volume (mm^3) for an ASCII STL."""
import pathlib, re, sys

vertex = re.compile(r"^\s*vertex\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)")

def cross(a,b):
    return (a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0])

tri=[]; vol6=0.0
for line in pathlib.Path(sys.argv[1]).read_text(errors="ignore").splitlines():
    m=vertex.match(line)
    if not m: continue
    tri.append(tuple(float(m.group(i)) for i in range(1,4)))
    if len(tri)==3:
        c=cross(tri[1],tri[2]); vol6 += sum(tri[0][i]*c[i] for i in range(3)); tri=[]
print(abs(vol6)/6.0)
