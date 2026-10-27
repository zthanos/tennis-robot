#!/usr/bin/env python3
import argparse, hashlib, json, math, xml.etree.ElementTree as ET
from pathlib import Path

ap=argparse.ArgumentParser();ap.add_argument('--delivery',type=Path,required=True);ap.add_argument('--motion',type=Path,required=True);ap.add_argument('--criteria',type=Path,required=True);ap.add_argument('--ball',type=Path,required=True);ap.add_argument('--output',type=Path,required=True);a=ap.parse_args()
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest(); errors=[]; c=json.loads(a.criteria.read_text())
if c['status']!='FROZEN_BEFORE_FIRST_P2_46_GAZEBO_RUN':errors.append('criteria not frozen')
if c['reference_geometry']['ramp_tip_underside_clearance_mm']!=8:errors.append('reference is not 8 mm')
details={}
for label,path in [('delivery',a.delivery),('motion',a.motion)]:
 root=ET.parse(path).getroot();base=root.find("link[@name='base_link']");names={x.attrib.get('name') for x in base.findall('collision')}
 if 'intake_fixed_ramp_floor_0_col' not in names:errors.append(f'{label}: ramp missing')
 first=base.find("collision[@name='intake_fixed_ramp_floor_0_col']");o=[float(x) for x in first.find('origin').attrib['xyz'].split()];p=float(first.find('origin').attrib['rpy'].split()[1]);s=[float(x) for x in first.find('geometry/box').attrib['size'].split()]
 low=1000*(o[2]-abs(math.sin(p))*s[0]/2-abs(math.cos(p))*s[2]/2)
 if low<7.4:errors.append(f'{label}: conservative ramp AABB {low:.3f} mm')
 details[label]={'urdf_sha256':sha(path),'conservative_ramp_aabb_min_mm':low}
result={'status':'PASS' if not errors else 'FAIL','errors':errors,'criteria_sha256':sha(a.criteria),'ball_sha256':sha(a.ball),'variants':details,'matrix_runs':37}
a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2));raise SystemExit(bool(errors))
