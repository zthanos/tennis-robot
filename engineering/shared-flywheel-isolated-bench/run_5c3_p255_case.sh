#!/usr/bin/env bash
set -euo pipefail
if [[ $# -ne 2 ]]; then
  echo "usage: $0 {series_gap_0|series_gap_50|series_gap_150|series_gap_300|side_by_side_y50|side_by_side_y100} {0.6|0.9}" >&2
  exit 2
fi
case_name="$1"; tread_mu="$2"
case "$tread_mu" in 0.6|0.9) ;; *) echo "tread mu must be 0.6 or 0.9" >&2; exit 2;; esac
case "$case_name" in
  series_gap_0) laterals=(0 0); longitudinals=(0 0.066) ;;
  series_gap_50) laterals=(0 0); longitudinals=(0 0.116) ;;
  series_gap_150) laterals=(0 0); longitudinals=(0 0.216) ;;
  series_gap_300) laterals=(0 0); longitudinals=(0 0.366) ;;
  side_by_side_y50) laterals=(-0.05 0.05); longitudinals=(0 0) ;;
  side_by_side_y100) laterals=(-0.1 0.1); longitudinals=(0 0) ;;
  *) echo "unknown corrected 5c-3 case: $case_name" >&2; exit 2 ;;
esac
bench=engineering/shared-flywheel-isolated-bench
criteria="$bench/criteria_5c3_p255.json"
root="${P2_55_RESULT_ROOT:-engineering/chassis-strength-bench/results/shared_flywheel_phase2/gazebo_5c3_p255}"
bench_gui="${BENCH_GUI:-false}"
bench_record="${BENCH_RECORD:-false}"
drive_profile="${P2_55_DRIVE_PROFILE:-continuous}"
case "$bench_gui" in true|false) ;; *) echo "BENCH_GUI must be true or false" >&2; exit 2;; esac
case "$bench_record" in true|false) ;; *) echo "BENCH_RECORD must be true or false" >&2; exit 2;; esac
case "$drive_profile" in continuous|stop_1s|reverse_75mm|p256_reverse_both|p256_asym_left|p256_reverse150_yaw12) ;; *) echo "invalid P2_55_DRIVE_PROFILE" >&2; exit 2;; esac
if [[ "$bench_gui" == true && "$bench_record" == true ]]; then
  echo "BENCH_RECORD is reserved for headless runs" >&2; exit 2
fi
tag(){ local value="$1"; value="${value/-/m}"; echo "${value/./p}"; }
if [[ "$bench_gui" == true ]]; then
  out="$root/live_gui/$drive_profile/tm_$(tag "$tread_mu")/$case_name"
elif [[ "$drive_profile" == continuous ]]; then
  out="$root/matrix/tm_$(tag "$tread_mu")/$case_name"
else
  out="$root/diagnostics/$drive_profile/tm_$(tag "$tread_mu")/$case_name"
fi
mkdir -p "$out"
telemetry="$(pwd)/$out/telemetry.csv"; native="$(pwd)/$out/native_contacts.csv"; observer="$(pwd)/$out/observer.csv"
rm -f "$telemetry" "$telemetry.done" "$native" "$observer" "$observer.done"
mass_args=(base_mass:=7.404 base_com_x:=0.09386412749865 base_com_z:=0.13227174500270 base_ixx:=0.38652546 base_iyy:=0.35877559 base_izz:=0.54328377 base_ixz:=-0.05214756 drive_wheel_mass:=0.65 drive_wheel_ixx:=0.00154721666667 drive_wheel_iyy:=0.002348125 drive_wheel_izz:=0.00154721666667)
xacro "$bench/v3_chassis_flywheel_bench.urdf.xacro" sim_mode:=false validation_fixtures_enabled:=true five_c_enabled:=true five_c_design:=p244 five_c_p244_ramp_enabled:=true five_c_ramp_tip_clearance:=0.008 five_c_cheek_variant:=exposed_shallow five_c_basket_variant:=p251_ski_42 tilt_angle_rad:=-0.052359877560 "${mass_args[@]}" > "$out/model.urdf"
gz sdf -p "$out/model.urdf" > "$out/model.sdf"
world_args=(--model-sdf "$out/model.sdf" --ball-model gazebo/models/tennis_ball_compliant/model.sdf --output "$out/world.sdf" --telemetry "$telemetry" --contacts "$native" --observer "$observer" --surface-speed "${P2_61_COLLECTION_SPEED:-6.0}" --tread-mu "$tread_mu" --ramp-mu 0.15 --duration 10.0 --drive-profile "$drive_profile" --max-step-size "${P2_61_MAX_STEP:-0.001}")
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
export GZ_PARTITION="p255_${case_name}_${tread_mu}_${drive_profile}_${$}_${RANDOM}"
engine=/usr/lib/x86_64-linux-gnu/gz-physics-7/engine-plugins/libgz-physics7-dartsim-plugin.so.7
gz_args=(-r -v 1 --physics-engine "$engine")
if [[ "$bench_gui" == false ]]; then gz_args+=(-s); else gz_args+=(--gui-config "$bench/5c3_side_gui.config"); fi
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
sha256sum "$out/model.urdf" > "$out/model.urdf.sha256"
if [[ "$bench_gui" == true ]]; then
  printf 'VISUAL_ONLY case=%s tread_mu=%s profile=%s; excluded from criteria\n' "$case_name" "$tread_mu" "$drive_profile"
  exit 0
fi
analysis_args=(--case "$case_name" --tread-mu "$tread_mu" --drive-profile "$drive_profile" --observer "$observer" --telemetry "$telemetry" --native-contacts "$native" --criteria "$criteria" --output "$out/summary.json")
for contact in "${ball_contacts[@]}"; do analysis_args+=(--ball-contact "$contact"); done
python3 "$bench/analyze_5c3_p255_run.py" "${analysis_args[@]}" > "$out/analysis.stdout"
python3 - "$out/summary.json" <<'PY'
import json,sys
p=json.load(open(sys.argv[1])); print(('PASS' if p['pass'] else 'FAIL')+f" case={p['case']} mu={p['tread_mu']} profile={p['drive_profile']}")
PY
