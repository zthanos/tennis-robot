#!/usr/bin/env python3
from __future__ import annotations
import argparse, hashlib, json, xml.etree.ElementTree as ET
from pathlib import Path

def sha(p:Path)->str:return hashlib.sha256(p.read_bytes()).hexdigest()
ap=argparse.ArgumentParser();ap.add_argument('--criteria',type=Path,required=True);ap.add_argument('--ball',type=Path,required=True);ap.add_argument('--image-digest',required=True);ap.add_argument('--output',type=Path,required=True);ap.add_argument('--angles',default='35,45,55');ap.add_argument('urdfs',type=Path,nargs='+');a=ap.parse_args()
c=json.loads(a.criteria.read_text()); errors=[]
accepted={
 'FROZEN_BEFORE_FIRST_P2_51_GAZEBO_CALIBRATION_RUN',
 'FROZEN_AFTER_R125_X300_CONTACT_BEFORE_R75_RERUN',
 'FROZEN_AFTER_R75_35DEG_CONTACT_BEFORE_GEOMETRY_DIAGNOSTIC',
 'FROZEN_BEFORE_REALIZED_ANGLE_DIAGNOSTIC',
 'FROZEN_BEFORE_R75_42DEG_MATERIAL_MATRIX',
}
if c.get('status') not in accepted:errors.append('criteria status')
conditions=c['frozen_inputs'].get('conditions')
if conditions not in (3,90):errors.append('matrix size')
angles=tuple(int(x) for x in a.angles.split(','))
if len(angles)!=len(a.urdfs):errors.append('angle/URDF count')
hashes={}
for angle,p in zip(angles,a.urdfs):
 root=ET.parse(p).getroot(); names={x.get('name') for x in root.findall('.//collision') if x.get('name')}
 required={'intake_fixed_ramp_floor_p251_flat_col','basket_p251_center_floor_col','basket_p251_rear_backboard_col','fixture_camera_body_col'}
 required|={f'intake_fixed_ramp_floor_p251_{d:02d}_col' for d in range(5,angle+1,5)}
 if angle==42:required.add('intake_fixed_ramp_floor_p251_42_col')
 if not required<=names:errors.append(f'{angle} missing {sorted(required-names)}')
 hashes[str(angle)]=sha(p)
result={'status':'PASS' if not errors else 'FAIL','errors':errors,'criteria_sha256':sha(a.criteria),'urdf_sha256_by_angle':hashes,'tennis_ball_compliant_sha256':sha(a.ball),'container_image_digest':a.image_digest,'frozen_conditions':conditions}
a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2));raise SystemExit(bool(errors))
