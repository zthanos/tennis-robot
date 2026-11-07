#!/usr/bin/env python3
"""Run the unmodified mandatory gate and retain its exact stdout/stderr/JSON."""
import argparse,json,subprocess
from pathlib import Path

root=Path(__file__).resolve().parents[2]
dest=Path(__file__).parent/'p293g-candidate-results'
parser=argparse.ArgumentParser();parser.add_argument('--tag',default='gate')
args=parser.parse_args();dest.mkdir(exist_ok=True)
run=subprocess.run(['bash','cad/check-current-scene.sh'],cwd=root,text=True,capture_output=True)
(dest/f'{args.tag}-stdout.txt').write_text(run.stdout)
(dest/f'{args.tag}-stderr.txt').write_text(run.stderr)
report=root/'cad/independent-check/out/scene/report.json'
if report.exists():(dest/f'{args.tag}-report.json').write_bytes(report.read_bytes())
print(run.stdout,end='');print(run.stderr,end='')
raise SystemExit(run.returncode)
