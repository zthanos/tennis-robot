#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
scad_file="$root_dir/body-panels-f-rear-v2.scad"
release_dir="$root_dir/stl-f-rear"
openscad_bin="${OPENSCAD_BIN:-/snap/bin/openscad-nightly}"
component_check="$root_dir/../check-stl-components.py"

if [[ "${DRY_RUN:-0}" == "1" ]]; then
  out_dir="$(mktemp -d "$root_dir/.dry-run-f-rear.XXXXXX")"
  trap 'rm -rf "$out_dir"' EXIT
else
  out_dir="$release_dir"
  mkdir -p "$out_dir"
fi

all_parts=(
  rear_lo_left rear_lo_right rear_hi_left rear_hi_right
  deck_rear_left deck_rear_right parkside_door_left parkside_door_right
  corner_stay_left corner_stay_right door_turn_button
)
parts=("${all_parts[@]}")
if (($#)); then parts=("$@"); fi

for part_name in "${parts[@]}"; do
  "$openscad_bin" -o "$out_dir/${part_name}.stl" \
    -D "part=\"${part_name}\"" "$scad_file"
  [[ -s "$out_dir/${part_name}.stl" ]] || {
    echo "Missing export: $part_name" >&2; exit 1;
  }
  python3 "$component_check" "$out_dir/${part_name}.stl" 1
done

echo "Validated ${#parts[@]} P2-86 rear-F single-body parts"
