#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
scad_file="$root_dir/rear-pod-v2.scad"
release_dir="$root_dir/stl-v2"
openscad_bin="${OPENSCAD_BIN:-/snap/bin/openscad-nightly}"
component_check="$root_dir/../check-stl-components.py"

if [[ "${DRY_RUN:-0}" == "1" ]]; then
  out_dir="$(mktemp -d "$root_dir/.dry-run-v2.XXXXXX")"
  trap 'rm -rf "$out_dir"' EXIT
else
  out_dir="$release_dir"
  mkdir -p "$out_dir"
fi

all_parts=(
  base_left base_right pi_shelf vertical_electronics_plate side_frame
  panel_posts corner_tie component_pins component_clips bts_spacers
  upright_mount_fit_coupon control_panel
)
parts=("${all_parts[@]}")
if (($#)); then
  parts=("$@")
fi
declare -A expected_components=(
  [base_left]=1 [base_right]=1 [pi_shelf]=1
  [vertical_electronics_plate]=1 [side_frame]=1 [panel_posts]=4
  [corner_tie]=1 [component_pins]=27 [component_clips]=32
  [bts_spacers]=18 [upright_mount_fit_coupon]=2 [control_panel]=1
)

for part_name in "${parts[@]}"; do
  "$openscad_bin" -o "$out_dir/${part_name}.stl" \
    -D "part=\"${part_name}\"" "$scad_file"
  [[ -s "$out_dir/${part_name}.stl" ]] || {
    echo "Missing export: $part_name" >&2; exit 1;
  }
  python3 "$component_check" "$out_dir/${part_name}.stl" \
    "${expected_components[$part_name]}"
done

# P2-75 cable parts remain active immutable release assets in stl-v2. They
# have no P2-81 geometry change and are intentionally not rewritten here.
for part_name in cable_bulkhead bulkhead_foot; do
  [[ -f "$release_dir/${part_name}.stl" ]] || {
    echo "Missing retained active asset: $release_dir/${part_name}.stl" >&2
    exit 1
  }
  python3 "$component_check" "$release_dir/${part_name}.stl" 1
done

echo "Validated ${#parts[@]} generated parts; retained cable parts present"
