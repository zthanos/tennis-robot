#!/usr/bin/env python3
"""Static consistency gate for the P2-42 harness."""
from __future__ import annotations
import argparse, json, xml.etree.ElementTree as ET
from pathlib import Path

def vec(text): return [float(x) for x in text.split()]
def main():
    ap=argparse.ArgumentParser(); ap.add_argument("--urdf",type=Path,required=True); ap.add_argument("--case",choices=("empty","full45","ballast"),required=True); ap.add_argument("--mapping",type=Path,required=True); ap.add_argument("--criteria",type=Path,required=True); ap.add_argument("--output",type=Path,required=True); a=ap.parse_args()
    root=ET.parse(a.urdf).getroot(); mapping=json.loads(a.mapping.read_text()); criteria=json.loads(a.criteria.read_text()); errors=[]
    collisions={x.attrib["name"]:x for x in root.findall(".//collision") if "name" in x.attrib}
    for f in mapping["features"]:
        col=collisions.get(f["collision"])
        if col is None: errors.append(f"missing {f['collision']}"); continue
        actual=vec(col.find("origin").attrib.get("xyz","0 0 0"))
        if any(abs(x-y)>1e-12 for x,y in zip(actual,f["origin_xyz_m"])): errors.append(f"origin mismatch {f['collision']}")
        geom=f["geometry"]
        if geom["type"]=="box":
            size=vec(col.find("geometry/box").attrib["size"])
            if any(abs(x-y)>1e-12 for x,y in zip(size,geom["size_m"])): errors.append(f"size mismatch {f['collision']}")
        else:
            cyl=col.find("geometry/cylinder")
            if cyl is None or abs(float(cyl.attrib["radius"])-geom["radius_m"])>1e-12 or abs(float(cyl.attrib["length"])-geom["length_m"])>1e-12: errors.append(f"cylinder mismatch {f['collision']}")
    for item in criteria["criteria"]:
        if item["hard_fail"] and (item.get("threshold") is None or not item.get("rationale") or not item.get("source")): errors.append(f"incomplete hard criterion {item['name']}")
    module_present=root.find("link[@name='flywheel_launcher_frame_link']") is not None
    expected_module=a.case!="ballast"
    if module_present!=expected_module: errors.append(f"load semantic mismatch module_present={module_present}")
    if criteria["load_cases"][a.case].startswith("spawn") != (a.case=="ballast"): errors.append("criteria load semantics mismatch")
    plugin=root.find(".//plugin[@name='phase2_v3::Tilt5bSystem']")
    if plugin is None: errors.append("5b plugin missing")
    result={"status":"PASS" if not errors else "FAIL","case":a.case,"errors":errors,"fixtures_checked":len(mapping["features"]),"hard_criteria_checked":sum(x["hard_fail"] for x in criteria["criteria"]),"module_present":module_present}
    a.output.write_text(json.dumps(result,indent=2)+"\n"); print(json.dumps(result,indent=2))
    if errors: raise SystemExit(1)
if __name__=="__main__": main()
