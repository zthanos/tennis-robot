#!/usr/bin/env bash
set -euo pipefail
[[ $# -eq 1 ]] || { echo "usage: $0 {historical|corrected}" >&2; exit 2; }
mode="$1"; case "$mode" in historical) scale=1;; corrected) scale=2;; *) exit 2;; esac
bench=engineering/shared-flywheel-isolated-bench
root=engineering/chassis-strength-bench/results/shared_flywheel_phase2/gazebo_p261_contact_model/platen/$mode
mkdir -p "$root"; contacts="$(pwd)/$root/contacts.csv"; platen="$(pwd)/$root/platen.csv"
rm -f "$contacts" "$platen" "$platen.done"
python3 "$bench/make_p261_platen_world.py" --output "$root/world.sdf" --contacts "$contacts" --platen "$platen" --scale "$scale" --damping "${P261_DAMPING:-4692.375890562493}"
export GZ_CONFIG_PATH=/usr/share/gz
physics="$root/physics_plugins"; mkdir -p "$physics"; ln -sfn /usr/lib/x86_64-linux-gnu/gz-physics-7/engine-plugins/libgz-physics7-dartsim-plugin.so.7 "$physics/libgz-physics7-dartsim-plugin.so"
export GZ_SIM_PHYSICS_ENGINE_PATH="$(pwd)/$physics"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$(pwd)/$bench/build_p261:$(pwd)/$bench/build_contact_5c:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
engine=/usr/lib/x86_64-linux-gnu/gz-physics-7/engine-plugins/libgz-physics7-dartsim-plugin.so.7
timeout --signal=TERM --kill-after=3 120 gz sim -r -s -v 1 --physics-engine "$engine" "$root/world.sdf" > "$root/gazebo.log" 2>&1 & pid=$!
trap 'kill "$pid" 2>/dev/null || true' EXIT
for _ in $(seq 1 1200); do [[ -f "$platen.done" ]] && break; kill -0 "$pid" 2>/dev/null || break; sleep .1; done
[[ -f "$platen.done" ]] || { tail -100 "$root/gazebo.log" >&2; exit 1; }
kill "$pid" 2>/dev/null || true; wait "$pid" 2>/dev/null || true; trap - EXIT
sha256sum "$bench/criteria_p261_contact_model.json" > "$root/criteria.sha256"
python3 "$bench/analyze_p261_platen.py" --contacts "$contacts" --mode "$mode" --output "$root/summary.json"
