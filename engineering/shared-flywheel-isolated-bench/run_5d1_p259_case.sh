#!/usr/bin/env bash
set -euo pipefail
if [[ $# -ne 4 ]]; then echo "usage: $0 {15|20} {0.6|0.9} {12|16|20|24} REPEAT" >&2; exit 2; fi
angle="$1"; tread_mu="$2"; surface="$3"; repeat="$4"
case "$angle" in 15|20) ;; *) echo "invalid angle" >&2; exit 2;; esac
case "$tread_mu" in 0.6|0.9) ;; *) echo "invalid mu" >&2; exit 2;; esac
case "$surface" in 12|16|20|24) ;; *) echo "invalid surface speed" >&2; exit 2;; esac
case "$repeat" in 1|2|3) ;; *) echo "invalid repeat" >&2; exit 2;; esac
bench=engineering/shared-flywheel-isolated-bench
criteria="$bench/criteria_5d1_p259.json"
root="${P2_59_RESULT_ROOT:-engineering/chassis-strength-bench/results/shared_flywheel_phase2/gazebo_5d1_p259}"
record="${BENCH_RECORD:-false}"
case "$record" in true|false) ;; *) echo "BENCH_RECORD must be true or false" >&2; exit 2;; esac
tag="${tread_mu/./p}"
out="$root/cases/angle_${angle}/tm_${tag}/surface_${surface}/repeat_${repeat}"
mkdir -p "$out"
telemetry="$(pwd)/$out/telemetry.csv"; contacts="$(pwd)/$out/native_contacts.csv"; ball_contacts="$(pwd)/$out/ball_contacts.csv"
rm -f "$telemetry" "$telemetry.done" "$contacts" "$ball_contacts"
mass_args=(base_mass:=7.404 base_com_x:=0.09386412749865 base_com_z:=0.13227174500270 base_ixx:=0.38652546 base_iyy:=0.35877559 base_izz:=0.54328377 base_ixz:=-0.05214756 drive_wheel_mass:=0.65 drive_wheel_ixx:=0.00154721666667 drive_wheel_iyy:=0.002348125 drive_wheel_izz:=0.00154721666667)
tilt=$(python3 -c 'import math,sys; print(math.radians(float(sys.argv[1])))' "$angle")
xacro "$bench/v3_chassis_flywheel_bench.urdf.xacro" sim_mode:=false validation_fixtures_enabled:=true five_c_enabled:=true five_c_design:=p244 five_c_p244_ramp_enabled:=true five_c_ramp_tip_clearance:=0.008 five_c_cheek_variant:=exposed_shallow five_c_basket_variant:=p251_ski_42 tilt_angle_rad:="$tilt" "${mass_args[@]}" > "$out/model.urdf"
gz sdf -p "$out/model.urdf" > "$out/model.sdf"
python3 "$bench/make_5d1_p259_world.py" --model-sdf "$out/model.sdf" --ball-model gazebo/models/tennis_ball_compliant/model.sdf --output "$out/world.sdf" --telemetry "$telemetry" --contacts "$contacts" --ball-contacts "$ball_contacts" --angle-deg "$angle" --surface-speed "$surface" --tread-mu "$tread_mu"
export GZ_CONFIG_PATH=/usr/share/gz
physics_dir="$out/physics_plugins"; mkdir -p "$physics_dir"
ln -sfn /usr/lib/x86_64-linux-gnu/gz-physics-7/engine-plugins/libgz-physics7-dartsim-plugin.so.7 "$physics_dir/libgz-physics7-dartsim-plugin.so"
export GZ_SIM_PHYSICS_ENGINE_PATH="$(pwd)/$physics_dir"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$(pwd)/$bench/build_p259:$(pwd)/$bench/build_contact_5c:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export GZ_SIM_RESOURCE_PATH="$(pwd)/gazebo/models:$(pwd)/ros2_ws/src:${GZ_SIM_RESOURCE_PATH:-}"
export GZ_PARTITION="p259_${angle}_${tag}_${surface}_${repeat}_${$}_${RANDOM}"
engine=/usr/lib/x86_64-linux-gnu/gz-physics-7/engine-plugins/libgz-physics7-dartsim-plugin.so.7
gz_args=(-r -v 1 --physics-engine "$engine" -s)
if [[ "$record" == true ]]; then mkdir -p "$out/replay"; gz_args+=(--record-path "$(pwd)/$out/replay" --log-overwrite); fi
pid=""
cleanup(){ if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then kill "$pid" 2>/dev/null || true; wait "$pid" 2>/dev/null || true; fi; }
trap cleanup EXIT INT TERM
timeout --signal=TERM --kill-after=3 180 /usr/bin/gz sim "${gz_args[@]}" "$out/world.sdf" > "$out/gazebo.log" 2>&1 & pid=$!
for _ in $(seq 1 1500); do
  [[ -f "$telemetry.done" ]] && break
  kill -0 "$pid" 2>/dev/null || break
  sleep .1
done
[[ -f "$telemetry.done" ]] || { tail -100 "$out/gazebo.log" >&2; exit 1; }
cleanup; pid=""
sha256sum "$criteria" > "$out/criteria.sha256"
python3 "$bench/analyze_5d1_p259_run.py" --angle "$angle" --mu "$tread_mu" --surface "$surface" --repeat "$repeat" --telemetry "$telemetry" --contacts "$contacts" --ball-contacts "$ball_contacts" --criteria "$criteria" --output "$out/summary.json" > "$out/analysis.stdout"
cat "$out/analysis.stdout"
