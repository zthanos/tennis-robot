#!/usr/bin/env bash
set -euo pipefail
if [[ $# -ne 3 ]]; then
  echo "usage: $0 VARIANT CASE {0.6|0.9}" >&2; exit 2
fi
variant="$1"; case_name="$2"; tread_mu="$3"
case "$variant" in
  p257_shield_m90|p257_shield_m100|p257_shield_m110|p257_stagger_open|p257_stagger_shield) ;;
  *) echo "unknown P2-57 variant: $variant" >&2; exit 2 ;;
esac
case "$tread_mu" in 0.6|0.9) ;; *) echo "tread mu must be 0.6 or 0.9" >&2; exit 2;; esac
laterals=(); longitudinals=()
case "$case_name" in
  side_y35) laterals=(-0.035 0.035); longitudinals=(0 0) ;;
  side_y50) laterals=(-0.050 0.050); longitudinals=(0 0) ;;
  side_y100) laterals=(-0.100 0.100); longitudinals=(0 0) ;;
  series_gap_0) laterals=(0 0); longitudinals=(0 0.066) ;;
  series_gap_50) laterals=(0 0); longitudinals=(0 0.116) ;;
  series_gap_150) laterals=(0 0); longitudinals=(0 0.216) ;;
  series_gap_300) laterals=(0 0); longitudinals=(0 0.366) ;;
  single_y0) laterals=(0); longitudinals=(0) ;;
  single_yp20) laterals=(0.020); longitudinals=(0) ;;
  single_ym20) laterals=(-0.020); longitudinals=(0) ;;
  single_yp40) laterals=(0.040); longitudinals=(0) ;;
  single_ym40) laterals=(-0.040); longitudinals=(0) ;;
  single_yp60) laterals=(0.060); longitudinals=(0) ;;
  single_ym60) laterals=(-0.060); longitudinals=(0) ;;
  single_yp80) laterals=(0.080); longitudinals=(0) ;;
  single_ym80) laterals=(-0.080); longitudinals=(0) ;;
  single_yp100) laterals=(0.100); longitudinals=(0) ;;
  single_ym100) laterals=(-0.100); longitudinals=(0) ;;
  *) echo "unknown P2-57 case: $case_name" >&2; exit 2 ;;
esac

bench=engineering/shared-flywheel-isolated-bench
criteria="$bench/criteria_5c3_p257.json"
root="${P2_57_RESULT_ROOT:-engineering/chassis-strength-bench/results/shared_flywheel_phase2/gazebo_5c3_p257}"
bench_gui="${BENCH_GUI:-false}"
bench_record="${BENCH_RECORD:-true}"
case "$bench_gui" in true|false) ;; *) echo "BENCH_GUI must be true or false" >&2; exit 2;; esac
case "$bench_record" in true|false) ;; *) echo "BENCH_RECORD must be true or false" >&2; exit 2;; esac
if [[ "$bench_gui" == true && "$bench_record" == true ]]; then
  echo "BENCH_RECORD is reserved for headless runs" >&2; exit 2
fi
tag="${tread_mu/./p}"
if [[ "$bench_gui" == true ]]; then
  out="$root/live_gui/$variant/tm_$tag/$case_name"
else
  out="$root/matrix/$variant/tm_$tag/$case_name"
fi
mkdir -p "$out"
telemetry="$(pwd)/$out/telemetry.csv"
native="$(pwd)/$out/native_contacts.csv"
observer="$(pwd)/$out/observer.csv"
rm -f "$telemetry" "$telemetry.done" "$native" "$observer" "$observer.done"

mass_args=(base_mass:=7.404 base_com_x:=0.09386412749865 base_com_z:=0.13227174500270 base_ixx:=0.38652546 base_iyy:=0.35877559 base_izz:=0.54328377 base_ixz:=-0.05214756 drive_wheel_mass:=0.65 drive_wheel_ixx:=0.00154721666667 drive_wheel_iyy:=0.002348125 drive_wheel_izz:=0.00154721666667)
xacro "$bench/v3_chassis_flywheel_bench.urdf.xacro" sim_mode:=false validation_fixtures_enabled:=true five_c_enabled:=true five_c_design:=p257 five_c_p244_ramp_enabled:=true five_c_ramp_tip_clearance:=0.008 five_c_cheek_variant:="$variant" five_c_basket_variant:=p251_ski_42 tilt_angle_rad:=-0.052359877560 "${mass_args[@]}" > "$out/model.urdf"
gz sdf -p "$out/model.urdf" > "$out/model.sdf"
world_args=(--model-sdf "$out/model.sdf" --ball-model gazebo/models/tennis_ball_compliant/model.sdf --output "$out/world.sdf" --telemetry "$telemetry" --contacts "$native" --observer "$observer" --surface-speed 6.0 --tread-mu "$tread_mu" --ramp-mu 0.15 --duration 10.0 --drive-profile continuous)
ball_contacts=()
for i in "${!laterals[@]}"; do
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
export GZ_PARTITION="p257_${variant}_${case_name}_${tread_mu}_${$}_${RANDOM}"
engine=/usr/lib/x86_64-linux-gnu/gz-physics-7/engine-plugins/libgz-physics7-dartsim-plugin.so.7
gz_args=(-r -v 1 --physics-engine "$engine")
if [[ "$bench_gui" == false ]]; then gz_args+=(-s); else gz_args+=(--gui-config "$bench/5c3_side_gui.config"); fi
if [[ "$bench_record" == true ]]; then
  mkdir -p "$out/replay"; gz_args+=(--record-path "$(pwd)/$out/replay" --log-overwrite)
fi
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
sha256sum "$criteria" > "$out/criteria.sha256"
if [[ "$bench_gui" == true ]]; then
  printf 'VISUAL_ONLY variant=%s case=%s tread_mu=%s; excluded from criteria\n' "$variant" "$case_name" "$tread_mu"
  exit 0
fi
analysis_args=(--variant "$variant" --case "$case_name" --tread-mu "$tread_mu" --observer "$observer" --telemetry "$telemetry" --native-contacts "$native" --criteria "$criteria" --output "$out/summary.json")
for contact in "${ball_contacts[@]}"; do analysis_args+=(--ball-contact "$contact"); done
python3 "$bench/analyze_5c3_p257_run.py" "${analysis_args[@]}" > "$out/analysis.stdout"
python3 - "$out/summary.json" <<'PY'
import json,sys
p=json.load(open(sys.argv[1])); print(('PASS' if p['pass'] else 'FAIL')+f" variant={p['variant']} case={p['case']} mu={p['tread_mu']} basket={p['metrics']['balls_in_basket']} jam={p['metrics']['jammed']}")
PY
