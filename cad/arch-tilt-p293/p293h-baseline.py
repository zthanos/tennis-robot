#!/usr/bin/env python3
"""PARKED P2-93h reference: preserve current-scene baseline; no CAD edits."""
import json, subprocess, hashlib
from pathlib import Path
H=Path(__file__).resolve().parent
ROOT=H.parents[1]
O=H/'p293h-analysis'
O.mkdir(exist_ok=True)
protected=[ROOT/'cad/independent-check'/n for n in ['scadtree.py','selftest.py']]
before={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in protected}
r=subprocess.run(['bash','cad/check-current-scene.sh'],cwd=ROOT,text=True,capture_output=True)
(O/'baseline-stdout.txt').write_text(r.stdout)
(O/'baseline-stderr.txt').write_text(r.stderr)
(O/'baseline-report.json').write_bytes((ROOT/'cad/independent-check/out/scene/report.json').read_bytes())
after={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in protected}
assert before==after
(O/'baseline-meta.json').write_text(json.dumps(dict(returncode=r.returncode,protected_SHA256=after),indent=2)+'\n')
print('Baseline exit:',r.returncode,'; protected files unchanged')
print(r.stdout[-1500:])
