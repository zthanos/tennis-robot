#!/usr/bin/env bash
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
out_dir="$here/stl"
openscad_bin="${OPENSCAD_BIN:-/snap/bin/openscad-nightly}"
printability_check="$here/../../check-stl-printability.sh"

parts=(base pusher spool_anchor)
outputs=(ramp_entry_launcher_base ramp_entry_launcher_pusher
         ramp_entry_launcher_spool_anchor)

mkdir -p "$out_dir"
for index in "${!parts[@]}"; do
  "$openscad_bin" --backend=manifold -o "$out_dir/${outputs[$index]}.stl" \
    -D "part=\"${parts[$index]}\"" "$here/ramp-entry-ball-launcher.scad"
  [[ -s "$out_dir/${outputs[$index]}.stl" ]] || {
    echo "Missing export: ${outputs[$index]}" >&2
    exit 1
  }
  bash "$printability_check" "$out_dir/${outputs[$index]}.stl" 1
done

echo "Ramp-entry launcher export PASS: ${#parts[@]} one-body, support-free STL"
