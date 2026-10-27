#!/usr/bin/env python3
"""Static P2-43 gate before the first 5c-1 simulation."""
from __future__ import annotations
import argparse,hashlib,json,xml.etree.ElementTree as ET
from pathlib import Path
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
 ap=argparse.ArgumentParser();ap.add_argument("--narrow",type=Path,required=True);ap.add_argument("--wide",type=Path,required=True);ap.add_argument("--criteria",type=Path,required=True);ap.add_argument("--contract",type=Path,required=True);ap.add_argument("--ball",type=Path,required=True);ap.add_argument("--output",type=Path,required=True);a=ap.parse_args();errors=[]
 c=json.loads(a.criteria.read_text());contract=json.loads(a.contract.read_text())
 if sha(a.ball)!=contract["ball_model"]["source_sha256"]:errors.append("calibrated ball model hash changed")
 if c["matrix"]["run_count"]!=900:errors.append("matrix is not 900 runs")
 for x in c["criteria"]:
  if x["hard_fail"] and (x.get("threshold") is None or not x.get("rationale") or not x.get("source")):errors.append(f"incomplete hard criterion {x['name']}")
 widths={}
 for variant,path in (("narrow",a.narrow),("wide",a.wide)):
  root=ET.parse(path).getroot();links={x.attrib["name"]:x for x in root.findall("link")};frame=links.get("flywheel_launcher_frame_link")
  if frame is None:errors.append(f"{variant}: module missing");continue
  names={x.attrib.get("name") for x in frame.findall("collision")}
  required={"intake_cheek_1_col","intake_cheek_-1_col","intake_moving_guide_floor_col","cad_upper_plate_col"}
  if not required<=names:errors.append(f"{variant}: guide collisions missing {sorted(required-names)}")
  cheek=frame.find("collision[@name='intake_cheek_1_col']");origin=[float(v) for v in cheek.find("origin").attrib["xyz"].split()];widths[variant]=origin[1]
  for side in ("left","right"):
   cyl=links[f"flywheel_{side}_link"].find(f"collision[@name='flywheel_{side}_col']/geometry/cylinder")
   if abs(float(cyl.attrib["radius"])-.1)>1e-12 or abs(float(cyl.attrib["length"])-.06)>1e-12:errors.append(f"{variant}: wheel geometry mismatch")
 if not widths.get("wide",0)>widths.get("narrow",1):errors.append("wide cheek is not wider than narrow")
 result={"status":"PASS" if not errors else "FAIL","errors":errors,"criteria_hard_count":sum(x["hard_fail"] for x in c["criteria"]),"ball_model_sha256":sha(a.ball),"cheek_positive_y_centres_m":widths,"matrix_runs":c["matrix"]["run_count"]}
 a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(result,indent=2)+"\n");print(json.dumps(result,indent=2))
 if errors:raise SystemExit(1)
if __name__=="__main__":main()
