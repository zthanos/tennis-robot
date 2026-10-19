#!/usr/bin/env bash
set -euo pipefail
if [[ $# -ne 6 ]]; then echo "usage: $0 APPROACH LATERAL CHEEK MU SURFACE_SPEED REPEAT" >&2; exit 2; fi
approach="$1"; lateral="$2"; cheek="$3"; mu="$4"; surface_speed="$5"; repeat="$6"
case "$approach" in 0.35|0.6|0.8) ;; *) exit 2;; esac; case "$lateral" in -0.1|-0.05|0.0|0.05|0.1) ;; *) exit 2;; esac; case "$cheek" in narrow|wide) ;; *) exit 2;; esac; case "$mu" in 0.3|0.6|0.9) ;; *) exit 2;; esac; case "$surface_speed" in 2.35|2.50) ;; *) exit 2;; esac
bench=engineering/shared-flywheel-isolated-bench
tag="v_${approach/./p}/y_${lateral/-/m}"; tag="${tag/./p}/${cheek}/mu_${mu/./p}/surf_${surface_speed/./p}/r${repeat}"
out="engineering/chassis-strength-bench/results/shared_flywheel_phase2/gazebo_5c1/$tag"; mkdir -p "$out"
telemetry="$(pwd)/$out/telemetry.csv"; native="$(pwd)/$out/native_contacts.csv"; ball_contacts="$(pwd)/$out/ball_contacts.csv"; rm -f "$telemetry" "$telemetry.done" "$native" "$ball_contacts"
mass_args=(base_mass:=7.404 base_com_x:=0.09386412749865 base_com_z:=0.13227174500270 base_ixx:=0.38652546 base_iyy:=0.35877559 base_izz:=0.54328377 base_ixz:=-0.05214756 drive_wheel_mass:=0.65 drive_wheel_ixx:=0.00154721666667 drive_wheel_iyy:=0.002348125 drive_wheel_izz:=0.00154721666667)
xacro "$bench/v3_chassis_flywheel_bench.urdf.xacro" sim_mode:=false validation_fixtures_enabled:=true five_c_enabled:=true five_c_cheek_variant:="$cheek" tilt_angle_rad:=-0.052359877560 "${mass_args[@]}" > "$out/model.urdf"
gz sdf -p "$out/model.urdf" > "$out/model.sdf"
python3 "$bench/make_5c1_world.py" --model-sdf "$out/model.sdf" --ball-model gazebo/models/tennis_ball_compliant/model.sdf --output "$out/world.sdf" --telemetry "$telemetry" --contacts "$native" --ball-contacts "$ball_contacts" --approach "$approach" --lateral "$lateral" --surface-speed "$surface_speed" --tread-mu "$mu" > "$out/world_build.stdout"
export GZ_CONFIG_PATH=/usr/share/gz
physics_dir="$out/physics_plugins";mkdir -p "$physics_dir";ln -sfn /usr/lib/x86_64-linux-gnu/gz-physics-7/engine-plugins/libgz-physics7-dartsim-plugin.so.7 "$physics_dir/libgz-physics7-dartsim-plugin.so"
export GZ_SIM_PHYSICS_ENGINE_PATH="$(pwd)/$physics_dir";export GZ_SIM_SYSTEM_PLUGIN_PATH="$(pwd)/$bench/build_5b:$(pwd)/$bench/build_contact_5c:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}";export GZ_SIM_RESOURCE_PATH="$(pwd)/gazebo/models:$(pwd)/ros2_ws/src:${GZ_SIM_RESOURCE_PATH:-}";export GZ_PARTITION="p243_${$}_${RANDOM}"
engine=/usr/lib/x86_64-linux-gnu/gz-physics-7/engine-plugins/libgz-physics7-dartsim-plugin.so.7;pid="";cleanup(){ if [[ -n "$pid" ]]&&kill -0 "$pid" 2>/dev/null;then kill "$pid" 2>/dev/null||true;wait "$pid" 2>/dev/null||true;fi;};trap cleanup EXIT INT TERM
timeout --signal=TERM --kill-after=3 90 /usr/bin/gz sim -r -s -v 1 --physics-engine "$engine" "$out/world.sdf" > "$out/gazebo.log" 2>&1 & pid=$!
for _ in $(seq 1 800);do [[ -f "$telemetry.done" ]]&&break;kill -0 "$pid" 2>/dev/null||break;sleep .1;done
[[ -f "$telemetry.done" ]]||{ tail -80 "$out/gazebo.log" >&2;exit 1;};cleanup;pid=""
python3 "$bench/analyze_5c1_run.py" "$telemetry" "$ball_contacts" "$native" --criteria "$bench/criteria_5c.json" --output "$out/summary.json" > "$out/analysis.stdout"
sha256sum "$out/model.urdf" > "$out/model.urdf.sha256"
python3 - "$out/summary.json" <<'PY'
import json,sys
p=json.load(open(sys.argv[1]))
if p['metrics']['forbidden_contact_count']:
 raise SystemExit(3)
print('PASS' if p['pass'] else 'FAIL')
PY
