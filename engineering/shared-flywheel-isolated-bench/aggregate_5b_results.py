#!/usr/bin/env python3
"""Create the machine-readable P2-42 result matrix from final case summaries."""
from __future__ import annotations
import argparse, hashlib, json
from pathlib import Path

def sha(path: Path) -> str: return hashlib.sha256(path.read_bytes()).hexdigest()
def main() -> None:
    ap=argparse.ArgumentParser(); ap.add_argument("--root",type=Path,required=True); ap.add_argument("--output",type=Path,required=True); ap.add_argument("--image-digest",required=True); a=ap.parse_args()
    cases={}
    for name in ("empty","full45","ballast"):
        folder=a.root/name; summary=json.loads((folder/"summary.json").read_text())
        cases[name]={"pass":summary["pass"],"scope":summary["scope"],"metrics":summary["metrics"],"urdf_sha256":sha(folder/"model.urdf"),"preflight":json.loads((folder/"preflight.json").read_text())["status"]}
    result={"status":"PASS" if all(x["pass"] for x in cases.values()) else "FAIL","gate":"P2-42_GAZEBO_5B_COMPLETE_PAUSED_BEFORE_5C_5D","container_image_digest":a.image_digest,"cases":cases,"p16_scope":"B: abstract joint actuator; force/current/linkage stress/singularity excluded","terminal_scope":"joint limits only; physical stop impact excluded","cad_changed":False,"p2_40_remains_valid":True}
    a.output.write_text(json.dumps(result,indent=2)+"\n"); print(json.dumps(result,indent=2))
if __name__=="__main__": main()
