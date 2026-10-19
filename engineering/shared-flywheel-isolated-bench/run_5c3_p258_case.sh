#!/usr/bin/env bash
set -euo pipefail
if [[ $# -ne 4 ]]; then
  echo "usage: $0 DX_MM DY_MM {lead_plus|lead_minus|center} {0.6|0.9}" >&2; exit 2
fi
dx_mm="$1"; dy_mm="$2"; orientation="$3"; tread_mu="$4"
case "$dx_mm" in 0|35|70|100|150) ;; *) echo "invalid dx" >&2; exit 2;; esac
case "$dy_mm" in 0|20|35|50|70|100) ;; *) echo "invalid dy" >&2; exit 2;; esac
case "$orientation" in lead_plus|lead_minus|center) ;; *) echo "invalid orientation" >&2; exit 2;; esac
case "$tread_mu" in 0.6|0.9) ;; *) echo "invalid tread mu" >&2; exit 2;; esac
if [[ "$dy_mm" == 0 && "$orientation" != center ]]; then echo "dy=0 uses center" >&2; exit 2; fi
if [[ "$dy_mm" != 0 && "$orientation" == center ]]; then echo "nonzero dy needs lead side" >&2; exit 2; fi
python3 - "$dx_mm" "$dy_mm" <<'PY'
import math,sys
if math.hypot(float(sys.argv[1]),float(sys.argv[2])) < 66.0-1e-9:
    raise SystemExit("infeasible overlapping initial ball centres")
PY

bench=engineering/shared-flywheel-isolated-bench
criteria="$bench/criteria_5c3_p258.json"
root="${P2_58_RESULT_ROOT:-engineering/chassis-strength-bench/results/shared_flywheel_phase2/gazebo_5c3_p258}"
bench_record="${BENCH_RECORD:-false}"
case "$bench_record" in true|false) ;; *) echo "BENCH_RECORD must be true or false" >&2; exit 2;; esac
tag="${tread_mu/./p}"
case_name="diag_dx${dx_mm}_dy${dy_mm}_${orientation}"
out="$root/matrix/tm_$tag/$orientation/dx_$dx_mm/dy_$dy_mm"
mkdir -p "$out"
telemetry="$(pwd)/$out/telemetry.csv"; native="$(pwd)/$out/native_contacts.csv"; observer="$(pwd)/$out/observer.csv"
rm -f "$telemetry" "$telemetry.done" "$native" "$observer" "$observer.done"

if [[ "$orientation" == lead_plus ]]; then sign=1; elif [[ "$orientation" == lead_minus ]]; then sign=-1; else sign=0; fi
lat0=$(python3 -c 'import sys; print(int(sys.argv[1])*float(sys.argv[2])/2000)' "$sign" "$dy_mm")
lat1=$(python3 -c 'import sys; print(-int(sys.argv[1])*float(sys.argv[2])/2000)' "$sign" "$dy_mm")
long1=$(python3 -c 'import sys; print(float(sys.argv[1])/1000)' "$dx_mm")
laterals=("$lat0" "$lat1"); longitudinals=(0 "$long1")

mass_args=(base_mass:=7.404 base_com_x:=0.09386412749865 base_com_z:=0.13227174500270 base_ixx:=0.38652546 base_iyy:=0.35877559 base_izz:=0.54328377 base_ixz:=-0.05214756 drive_wheel_mass:=0.65 drive_wheel_ixx:=0.00154721666667 drive_wheel_iyy:=0.002348125 drive_wheel_izz:=0.00154721666667)
xacro "$bench/v3_chassis_flywheel_bench.urdf.xacro" sim_mode:=false validation_fixtures_enabled:=true five_c_enabled:=true five_c_design:=p244 five_c_p244_ramp_enabled:=true five_c_ramp_tip_clearance:=0.008 five_c_cheek_variant:=exposed_shallow five_c_basket_variant:=p251_ski_42 tilt_angle_rad:=-0.052359877560 "${mass_args[@]}" > "$out/model.urdf"
gz sdf -p "$out/model.urdf" > "$out/model.sdf"
world_args=(--model-sdf "$out/model.sdf" --ball-model gazebo/models/tennis_ball_compliant/model.sdf --output "$out/world.sdf" --telemetry "$telemetry" --contacts "$native" --observer "$observer" --surface-speed "${P2_61_COLLECTION_SPEED:-6.0}" --tread-mu "$tread_mu" --ramp-mu 0.15 --duration 10.0 --drive-profile continuous --max-step-size "${P2_61_MAX_STEP:-0.001}")
ball_contacts=()
for i in 0 1; do
  contact="$(pwd)/$out/ball_$(printf '%02d' "$i")_contacts.csv"
  rm -f "$contact"; ball_contacts+=("$contact")
  world_args+=(--ball-contact "$contact" --longitudinal "${longitudinals[$i]}" --lateral "${laterals[$i]}")
done
python3 "$bench/make_5c3_p255_world.py" "${world_args[@]}" > "$out/world_build.stdout"

export GZ_CONFIG_PATH=/usr/share/gz
physics_dir="$out/physics_plugins"; mkdir -p "$physics_dir"
ln -sfn /usr/lib/x86_64-linux-gnu/gz-physics-7/engine-plugins/libgz-physics7-dartsim-plugin.so.7 "$physics_dir/libgz-physics7-dartsim-plugin.so"
export GZ_SIM_PHYSICS_ENGINE_PATH="$(pwd)/$physics_dir"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$(pwd)/$bench/build_p255:$(pwd)/$bench/build_contact_5c:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export GZ_SIM_RESOURCE_PATH="$(pwd)/gazebo/models:$(pwd)/ros2_ws/src:${GZ_SIM_RESOURCE_PATH:-}"
export GZ_PARTITION="p258_${dx_mm}_${dy_mm}_${orientation}_${tread_mu}_${$}_${RANDOM}"
engine=/usr/lib/x86_64-linux-gnu/gz-physics-7/engine-plugins/libgz-physics7-dartsim-plugin.so.7
gz_args=(-r -v 1 --physics-engine "$engine" -s)
if [[ "$bench_record" == true ]]; then mkdir -p "$out/replay"; gz_args+=(--record-path "$(pwd)/$out/replay" --log-overwrite); fi
pid=""
cleanup(){ if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then kill "$pid" 2>/dev/null || true; wait "$pid" 2>/dev/null || true; fi; }
trap cleanup EXIT INT TERM
timeout --signal=TERM --kill-after=3 180 /usr/bin/gz sim "${gz_args[@]}" "$out/world.sdf" > "$out/gazebo.log" 2>&1 & pid=$!
for _ in $(seq 1 1500); do
  [[ -f "$telemetry.done" && -f "$observer.done" ]] && break
  kill -0 "$pid" 2>/dev/null || break
  sleep .1
done
[[ -f "$telemetry.done" && -f "$observer.done" ]] || { tail -100 "$out/gazebo.log" >&2; exit 1; }
cleanup; pid=""
sha256sum "$out/model.urdf" > "$out/model.urdf.sha256"; sha256sum "$criteria" > "$out/criteria.sha256"
analysis_args=(--variant baseline_p240 --case "$case_name" --tread-mu "$tread_mu" --observer "$observer" --telemetry "$telemetry" --native-contacts "$native" --criteria "$criteria" --output "$out/summary.json")
for contact in "${ball_contacts[@]}"; do analysis_args+=(--ball-contact "$contact"); done
python3 "$bench/analyze_5c3_p257_run.py" "${analysis_args[@]}" > "$out/analysis.stdout"
python3 - "$out/summary.json" "$bench_record" <<'PY'
import json,sys
p=json.load(open(sys.argv[1])); m=p['metrics']
classification='PASS' if m['balls_in_basket']==2 else ('JAM' if m['jammed'] else 'REJECT')
print(f"{classification} case={p['case']} mu={p['tread_mu']} basket={m['balls_in_basket']} detector={m['d5065_detector_triggered']} record={sys.argv[2]}")
PY
