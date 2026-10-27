#!/usr/bin/env bash
set -euo pipefail
bench=engineering/shared-flywheel-isolated-bench
root=engineering/chassis-strength-bench/results/shared_flywheel_phase2/gazebo_5c2_p247_r4/preflight
mkdir -p "$root"
mass_args=(base_mass:=7.404 base_com_x:=0.09386412749865 base_com_z:=0.13227174500270 base_ixx:=0.38652546 base_iyy:=0.35877559 base_izz:=0.54328377 base_ixz:=-0.05214756 drive_wheel_mass:=0.65 drive_wheel_ixx:=0.00154721666667 drive_wheel_iyy:=0.002348125 drive_wheel_izz:=0.00154721666667)
common=(sim_mode:=false validation_fixtures_enabled:=true five_c_enabled:=true five_c_design:=p244 five_c_p244_ramp_enabled:=true five_c_ramp_tip_clearance:=0.008 five_c_cheek_variant:=exposed_shallow five_c_basket_variant:=tall_open_drop tilt_angle_rad:=-0.052359877560)
xacro "$bench/v3_chassis_flywheel_bench.urdf.xacro" "${common[@]}" "${mass_args[@]}" > "$root/tall_open_drop_delivery.urdf"
xacro "$bench/v3_chassis_flywheel_bench.urdf.xacro" "${common[@]}" "${mass_args[@]}" five_a_enabled:=true five_a_case:=p247_full45_retention five_a_output_csv:=/tmp/p247_r4_drive.csv retention_only:=true yaw_feedback_enabled:=true tilt_hold_enabled:=true > "$root/tall_open_drop_motion.urdf"
check_urdf "$root/tall_open_drop_delivery.urdf" > "$root/tall_open_drop_delivery.check_urdf.txt"
check_urdf "$root/tall_open_drop_motion.urdf" > "$root/tall_open_drop_motion.check_urdf.txt"
python3 "$bench/analyze_p247_tall_open_static.py" > "$root/static.stdout"
python3 - "$root" "$bench/criteria_5c2_p247_r4.json" <<'PY'
import hashlib,json,sys,xml.etree.ElementTree as ET
from pathlib import Path
root,criteria=Path(sys.argv[1]),Path(sys.argv[2]); errors=[]; hashes={}
required={'intake_fixed_ramp_floor_9t_col','basket_tall_left_wall_col','basket_tall_right_wall_col','basket_tall_rear_wall_col','basket_tall_front_wall_col'}
for mode in ('delivery','motion'):
    path=root/f'tall_open_drop_{mode}.urdf'; tree=ET.parse(path); names={x.get('name') for x in tree.findall('.//collision')}
    missing=sorted(required-names)
    if missing: errors.append(f'{mode}: missing {missing}')
    if tree.find(".//joint[@name='basket_one_way_flap_joint']") is not None: errors.append(f'{mode}: unexpected flap')
    hashes[mode]=hashlib.sha256(path.read_bytes()).hexdigest()
status=json.loads(criteria.read_text()).get('status')
if status!='FROZEN_BEFORE_FIRST_P2_47_R4_GAZEBO_RUN': errors.append('criteria not frozen')
result={'status':'PASS' if not errors else 'FAIL','errors':errors,'criteria_sha256':hashlib.sha256(criteria.read_bytes()).hexdigest(),'ball_sha256':hashlib.sha256(Path('gazebo/models/tennis_ball_compliant/model.sdf').read_bytes()).hexdigest(),'urdf_sha256':hashes,'matrix_runs':15}
(root/'preflight.json').write_text(json.dumps(result,indent=2)+'\n'); print(json.dumps(result,indent=2)); raise SystemExit(bool(errors))
PY
