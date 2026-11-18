#!/usr/bin/env python3
"""Verify handed exports after the intentional P2-98 L/R engravings."""
from __future__ import annotations
import pathlib, re, sys

root = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else "print-arch")
vertex = re.compile(r"^\s*vertex\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)")
pairs = {
    "foot": "y", "knee": "z", "crown": "z", "crossbar_end": "y",
    "mast": "y", "mast_cap": "y", "lidar_half": "y",
}

def vertices(path: pathlib.Path):
    out=[]
    for line in path.read_text(errors="ignore").splitlines():
        m=vertex.match(line)
        if m: out.append(tuple(round(float(m.group(i)),5) for i in range(1,4)))
    return out

def volume(v):
    total=0.0
    for i in range(0,len(v),3):
        a,b,c=v[i:i+3]
        total += (a[0]*(b[1]*c[2]-b[2]*c[1])
                  -a[1]*(b[0]*c[2]-b[2]*c[0])
                  +a[2]*(b[0]*c[1]-b[1]*c[0]))
    return abs(total)/6

for name,axis in pairs.items():
    lv=vertices(root/f"{name}_L.stl"); rv=vertices(root/f"{name}_R.stl")
    assert lv and rv, f"{name}: missing mesh vertices"
    assert min(p[2] for p in lv)==0 and min(p[2] for p in rv)==0, f"{name}: not on bed Z=0"
    lvol,rvol=volume(lv),volume(rv)
    # The base solids are generated from the same source and mirrored by the
    # export wrapper.  Only the 0.6 mm L/R glyph cavities differ, so their
    # volume delta must remain smaller than one 8 mm label envelope.
    assert abs(lvol-rvol)<20, f"{name}: non-label volume changed"
    lb=[round(max(p[i] for p in lv)-min(p[i] for p in lv),5) for i in range(3)]
    rb=[round(max(p[i] for p in rv)-min(p[i] for p in rv),5) for i in range(3)]
    assert lb==rb, f"{name}: bounding extents changed"
    print(f"HANDED PAIR PASS {name}: axis={axis.upper()} "
          f"volumes={lvol/1000:.3f}/{rvol/1000:.3f}cm3 bbox={lb}")
