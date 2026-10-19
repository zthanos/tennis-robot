#!/usr/bin/env bash
set -euo pipefail
if [[ $# -ne 3 ]]; then echo "usage: $0 MODE PRELOAD REPEAT" >&2; exit 2; fi
mode="$1"; preload="$2"; repeat="$3"
case "$mode" in delivery|motion) ;; *) exit 2;; esac
case "$repeat" in 1|2|3|4|5) ;; *) exit 2;; esac
if [[ "$mode" == delivery ]]; then
  case "$preload" in 0|10|25|40|44|49|54|59) ;; *) exit 2;; esac
else
  [[ "$preload" == 45 ]] || exit 2
fi

bench=engineering/shared-flywheel-isolated-bench
out="engineering/chassis-strength-bench/results/shared_flywheel_phase2/gazebo_5c2_p246/${mode}/preload_${preload}/r${repeat}"
mkdir -p "$out"
observer="$(pwd)/$out/observer.csv"; rm -f "$observer" "$observer.done"
mass_args=(base_mass:=7.404 base_com_x:=0.09386412749865 base_com_z:=0.13227174500270 base_ixx:=0.38652546 base_iyy:=0.35877559 base_izz:=0.54328377 base_ixz:=-0.05214756 drive_wheel_mass:=0.65 drive_wheel_ixx:=0.00154721666667 drive_wheel_iyy:=0.002348125 drive_wheel_izz:=0.00154721666667)
common=(sim_mode:=false validation_fixtures_enabled:=true five_c_enabled:=true five_c_design:=p244 five_c_p244_ramp_enabled:=true five_c_ramp_tip_clearance:=0.008 five_c_cheek_variant:=exposed_shallow tilt_angle_rad:=-0.052359877560)
if [[ "$mode" == delivery ]]; then
  duration=10
  telemetry="$(pwd)/$out/telemetry.csv"; native="$(pwd)/$out/native_contacts.csv"; ball_contacts="$(pwd)/$out/ball_contacts.csv"
  rm -f "$telemetry" "$telemetry.done" "$native" "$ball_contacts"
  xacro "$bench/v3_chassis_flywheel_bench.urdf.xacro" "${common[@]}" "${mass_args[@]}" > "$out/model.urdf"
else
  duration=15
  drive="$(pwd)/$out/drive.csv"; rm -f "$drive" "$drive.done"
  xacro "$bench/v3_chassis_flywheel_bench.urdf.xacro" "${common[@]}" "${mass_args[@]}" five_a_enabled:=true five_a_case:=p246_full45_retention five_a_output_csv:="$drive" retention_only:=true yaw_feedback_enabled:=true tilt_hold_enabled:=true > "$out/model.urdf"
fi
gz sdf -p "$out/model.urdf" > "$out/model.sdf"
world_args=(--mode "$mode" --model-sdf "$out/model.sdf" --ball-model gazebo/models/tennis_ball_compliant/model.sdf --output "$out/world.sdf" --observer "$observer" --preload "$preload" --seed "$repeat" --duration "$duration")
if [[ "$mode" == delivery ]]; then world_args+=(--telemetry "$telemetry" --contacts "$native" --ball-contacts "$ball_contacts"); fi
python3 "$bench/make_5c2_p246_world.py" "${world_args[@]}" > "$out/world_build.stdout"

export GZ_CONFIG_PATH=/usr/share/gz
physics_dir="$out/physics_plugins"; mkdir -p "$physics_dir"
ln -sfn /usr/lib/x86_64-linux-gnu/gz-physics-7/engine-plugins/libgz-physics7-dartsim-plugin.so.7 "$physics_dir/libgz-physics7-dartsim-plugin.so"
export GZ_SIM_PHYSICS_ENGINE_PATH="$(pwd)/$physics_dir"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$(pwd)/$bench/build_p246:$(pwd)/$bench/build_contact_5c:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export GZ_SIM_RESOURCE_PATH="$(pwd)/gazebo/models:$(pwd)/ros2_ws/src:${GZ_SIM_RESOURCE_PATH:-}"
export GZ_PARTITION="p246_${mode}_${preload}_${repeat}_${RANDOM}"
engine=/usr/lib/x86_64-linux-gnu/gz-physics-7/engine-plugins/libgz-physics7-dartsim-plugin.so.7
pid=""
cleanup(){ if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then kill "$pid" 2>/dev/null || true; wait "$pid" 2>/dev/null || true; fi; }
trap cleanup EXIT INT TERM
timeout --signal=TERM --kill-after=3 480 /usr/bin/gz sim -r -s -v 1 --physics-engine "$engine" "$out/world.sdf" > "$out/gazebo.log" 2>&1 & pid=$!
for _ in $(seq 1 4500); do
  if [[ -f "$observer.done" ]] && { [[ "$mode" == delivery && -f "$telemetry.done" ]] || [[ "$mode" == motion && -f "$drive.done" ]]; }; then break; fi
  kill -0 "$pid" 2>/dev/null || break; sleep .1
done
[[ -f "$observer.done" ]] || { tail -100 "$out/gazebo.log" >&2; exit 1; }
cleanup; pid=""
if [[ "$mode" == delivery ]]; then
  python3 "$bench/analyze_5c2_p246_run.py" --mode delivery --observer "$observer" --preload "$preload" --telemetry "$telemetry" --ball-contacts "$ball_contacts" --native-contacts "$native" --criteria "$bench/criteria_5c2_p246.json" --output "$out/summary.json" > "$out/analysis.stdout"
else
  python3 "$bench/analyze_5c2_p246_run.py" --mode motion --observer "$observer" --preload "$preload" --drive-telemetry "$drive" --criteria "$bench/criteria_5c2_p246.json" --output "$out/summary.json" > "$out/analysis.stdout"
fi
sha256sum "$out/model.urdf" > "$out/model.urdf.sha256"
python3 - "$out/summary.json" <<'PY'
import json,sys
p=json.load(open(sys.argv[1])); print('PASS' if p['pass'] else 'FAIL')
if p['metrics'].get('forbidden_contact_count',0): raise SystemExit(3)
PY
