#!/usr/bin/env python3
"""Close-up PNG per collision in out/report.json: parts translucent, overlap red."""
import json
import os
import subprocess
from pathlib import Path

OUT = Path(__file__).resolve().parent / "out"
OPENSCAD = os.environ.get("OPENSCAD_BIN", "/snap/bin/openscad-nightly")

for h in json.loads((OUT / "report.json").read_text())["hits"]:
    a, b, bb = h["a"], h["b"], h["bbox"]
    c = [(bb[i] + bb[i + 3]) / 2 for i in range(3)]
    size = max(bb[i + 3] - bb[i] for i in range(3))
    d = max(120.0, 2.2 * size)
    eye = [c[0] + d, c[1] - d, c[2] + 0.8 * d]
    clip = f"translate([{c[0]},{c[1]},{c[2]}]) cube({2*d},center=true);"
    scad = OUT / f"view__{a}__{b}.scad"
    scad.write_text(
        f'intersection(){{ color([0.55,0.6,0.7,0.35]) import("{a}.stl"); {clip} }}\n'
        f'intersection(){{ color([0.95,0.7,0.2,0.35]) import("{b}.stl"); {clip} }}\n'
        f'color([1,0,0]) import("hit__{a}__{b}.stl");\n')
    png = OUT / f"hit__{a}__{b}.png"
    subprocess.run([OPENSCAD, "-o", str(png), "--imgsize=900,700", "--colorscheme=Tomorrow",
                    f"--camera={eye[0]},{eye[1]},{eye[2]},{c[0]},{c[1]},{c[2]}",
                    "--projection=p", str(scad)], capture_output=True, timeout=600)
    print(png.name, "ok" if png.exists() else "FAIL")
