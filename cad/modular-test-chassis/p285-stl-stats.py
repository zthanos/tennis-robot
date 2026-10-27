#!/usr/bin/env python3
"""Print volume (mm^3) and bounding box of an STL (ASCII or binary)."""
import struct, sys
from pathlib import Path

def tris(p):
    d = Path(p).read_bytes()
    if len(d) >= 84:
        n = struct.unpack_from("<I", d, 80)[0]
        if 84 + 50 * n == len(d):
            for i in range(n):
                o = 84 + 50 * i + 12
                yield [struct.unpack_from("<fff", d, o + 12 * k) for k in range(3)]
            return
    v = []
    for line in d.decode().splitlines():
        f = line.split()
        if len(f) == 4 and f[0] == "vertex":
            v.append(tuple(map(float, f[1:])))
            if len(v) == 3:
                yield v; v = []

for p in sys.argv[1:]:
    vol = 0.0; lo = [1e9] * 3; hi = [-1e9] * 3
    for a, b, c in tris(p):
        vol += (a[0]*(b[1]*c[2]-b[2]*c[1]) - a[1]*(b[0]*c[2]-b[2]*c[0])
                + a[2]*(b[0]*c[1]-b[1]*c[0])) / 6
        for q in (a, b, c):
            for k in range(3):
                lo[k] = min(lo[k], q[k]); hi[k] = max(hi[k], q[k])
    print(f"{Path(p).name}: vol={vol:.3f} mm3 bbox=" +
          " ".join(f"{lo[k]:.2f}..{hi[k]:.2f}" for k in range(3)))
