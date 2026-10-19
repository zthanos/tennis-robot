#!/usr/bin/env python3
import argparse, hashlib, json, xml.etree.ElementTree as ET
from pathlib import Path

ap=argparse.ArgumentParser(); ap.add_argument('--root',type=Path,required=True); ap.add_argument('--criteria',type=Path,required=True); ap.add_argument('--ball',type=Path,required=True); ap.add_argument('--output',type=Path,required=True); a=ap.parse_args()
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest(); errors=[]; details={}; c=json.loads(a.criteria.read_text())
if c.get('status')!='FROZEN_BEFORE_FIRST_P2_47_R3_GAZEBO_RUN': errors.append('criteria not frozen')
expected={'high_hood':('basket_high_hood_roof_col','intake_fixed_ramp_floor_5h_col'),'one_way_flap':('basket_one_way_flap_col',),'lip_cover':('basket_center_retention_lip_col','basket_light_cover_roof_col','intake_fixed_ramp_floor_6l_col')}
for variant,names in expected.items():
    details[variant]={}
    for mode in ('delivery','motion'):
        path=a.root/f'{variant}_{mode}.urdf'; root=ET.parse(path).getroot(); collisions={x.attrib.get('name') for x in root.findall('.//collision')}
        for name in names:
            if name not in collisions: errors.append(f'{variant}/{mode}: missing {name}')
        if variant=='one_way_flap':
            joint=root.find("joint[@name='basket_one_way_flap_joint']")
            if joint is None: errors.append(f'{variant}/{mode}: flap joint missing')
            else:
                limit=joint.find('limit');
                if limit is None or float(limit.attrib['lower'])!=0 or abs(float(limit.attrib['upper'])-1.396263402)>1e-8: errors.append(f'{variant}/{mode}: flap limits wrong')
        details[variant][mode]={'urdf_sha256':sha(path),'collision_count':len(collisions)}
result={'status':'PASS' if not errors else 'FAIL','errors':errors,'criteria_sha256':sha(a.criteria),'ball_sha256':sha(a.ball),'matrix_runs':45,'variants':details}
a.output.parent.mkdir(parents=True,exist_ok=True); a.output.write_text(json.dumps(result,indent=2)+'\n'); print(json.dumps(result,indent=2)); raise SystemExit(bool(errors))
