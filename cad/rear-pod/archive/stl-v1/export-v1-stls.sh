#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
scad_file="$root_dir/rear-pod-v1.scad"
out_dir="$root_dir/stl-v1"
openscad_bin="${OPENSCAD_BIN:-/snap/bin/openscad-nightly}"

mkdir -p "$out_dir"

parts=(
  base_left base_right mega_shelf perf_shelf bts_shelf pi_shelf
  control_panel shelf_couplers component_pins component_clips bts_spacers
  chassis_spacers cable_bulkhead bulkhead_foot mpu6050_mount fit_coupon
)

for part_name in "${parts[@]}"; do
  "$openscad_bin" -o "$out_dir/${part_name}.stl" \
    -D "part=\"${part_name}\"" "$scad_file"
done

for length in 38 48 58 53 63 68 73 98 113 128; do
  "$openscad_bin" -o "$out_dir/pillars_${length}mm_x4.stl" \
    -D 'part="pillar"' -D "pillar_length=${length}" "$scad_file"
done
