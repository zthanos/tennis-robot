#!/usr/bin/env bash
set -euo pipefail
[[ $# -eq 1 ]] || exit 2; dt="$1"; tag="${dt//./p}"; bench=engineering/shared-flywheel-isolated-bench; root=engineering/chassis-strength-bench/results/shared_flywheel_phase2/gazebo_p261_contact_model/rebound/dt_$tag; mkdir -p "$root"
telemetry="$(pwd)/$root/telemetry.csv"; contacts="$(pwd)/$root/contacts.csv"; rm -f "$telemetry" "$telemetry.done" "$contacts"
python3 "$bench/make_p261_rebound_world.py" --output "$root/world.sdf" --telemetry "$telemetry" --contacts "$contacts" --dt "$dt"
export GZ_CONFIG_PATH=/usr/share/gz; physics="$root/physics_plugins";mkdir -p "$physics";ln -sfn /usr/lib/x86_64-linux-gnu/gz-physics-7/engine-plugins/libgz-physics7-dartsim-plugin.so.7 "$physics/libgz-physics7-dartsim-plugin.so";export GZ_SIM_PHYSICS_ENGINE_PATH="$(pwd)/$physics";export GZ_SIM_SYSTEM_PLUGIN_PATH="$(pwd)/$bench/build_p261:$(pwd)/$bench/build_contact_5c:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}";engine=/usr/lib/x86_64-linux-gnu/gz-physics-7/engine-plugins/libgz-physics7-dartsim-plugin.so.7
timeout --signal=TERM --kill-after=3 120 gz sim -r -s -v 1 --physics-engine "$engine" "$root/world.sdf" > "$root/gazebo.log" 2>&1 & pid=$!;trap 'kill "$pid" 2>/dev/null||true' EXIT;for _ in $(seq 1 1000);do [[ -f "$telemetry.done" ]]&&break;kill -0 "$pid" 2>/dev/null||break;sleep .1;done;[[ -f "$telemetry.done" ]]||{ tail -100 "$root/gazebo.log";exit 1;};kill "$pid" 2>/dev/null||true;wait "$pid" 2>/dev/null||true;trap - EXIT
python3 "$bench/analyze_p261_rebound.py" --telemetry "$telemetry" --dt "$dt" --output "$root/summary.json"
