#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
scad_file="$root_dir/rear-pod-v2.scad"
release_dir="$root_dir/stl-v2"
openscad_bin="${OPENSCAD_BIN:-/snap/bin/openscad-nightly}"

if [[ "${DRY_RUN:-0}" == "1" ]]; then
  out_dir="$(mktemp -d "$root_dir/.dry-run-v2.XXXXXX")"
  trap 'rm -rf "$out_dir"' EXIT
else
  out_dir="$release_dir"
  mkdir -p "$out_dir"
fi

parts=(
  base_left base_right pi_shelf vertical_electronics_plate side_frame
  panel_posts corner_tie component_pins component_clips bts_spacers
  upright_mount_fit_coupon control_panel
)

for part_name in "${parts[@]}"; do
  "$openscad_bin" -o "$out_dir/${part_name}.stl" \
    -D "part=\"${part_name}\"" "$scad_file"
  [[ -s "$out_dir/${part_name}.stl" ]] || {
    echo "Missing export: $part_name" >&2; exit 1;
  }
done

# P2-75 cable parts remain active immutable release assets in stl-v2. They
# have no P2-81 geometry change and are intentionally not rewritten here.
for part_name in cable_bulkhead bulkhead_foot; do
  [[ -f "$release_dir/${part_name}.stl" ]] || {
    echo "Missing retained active asset: $release_dir/${part_name}.stl" >&2
    exit 1
  }
done

echo "Validated ${#parts[@]} generated parts; retained cable parts present"
