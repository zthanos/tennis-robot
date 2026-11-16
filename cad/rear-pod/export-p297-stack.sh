#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
scad_file="$root_dir/stackable-box-p297.scad"
out_dir="$root_dir/print-stack"
openscad_bin="${OPENSCAD_BIN:-/snap/bin/openscad-nightly}"
component_check="$root_dir/../check-stl-components.py"
downward_check="$root_dir/../check-stl-downward-faces.py"
bts_height="${P297_BTS_HEIGHT_MM:-}"
seam_clearance="${P297_SEAM_CLEARANCE_MM:-0.35}"

if [[ ! "$bts_height" =~ ^[0-9]+([.][0-9]+)?$ ]] || [[ "$bts_height" == "0" ]]; then
  echo "Set P297_BTS_HEIGHT_MM to the measured BTS underside-to-heatsink-top height before exporting." >&2
  exit 2
fi

if [[ ! "$seam_clearance" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  echo "P297_SEAM_CLEARANCE_MM must be a non-negative number (default 0.35 mm per side)." >&2
  exit 2
fi

mkdir -p "$out_dir"

parts=(
  p297_base_adapter
  p297_energy_spacer
  p297_power_box
  p297_logic_box
  p297_collar
  p297_mega_carrier
  p297_perf_carrier
  p297_pi5_carrier
  p297_seam_coupon_lower
  p297_seam_coupon_upper
)

for part_name in "${parts[@]}"; do
  "$openscad_bin" -o "$out_dir/${part_name}.stl" \
    -D "part=\"${part_name}\"" -D 'show_ghosts=false' \
    -D "p297_bts_measured_h=$bts_height" \
    -D "p297_seam_clearance=$seam_clearance" "$scad_file"
  [[ -s "$out_dir/${part_name}.stl" ]] || {
    echo "Missing export: $part_name" >&2
    exit 1
  }
  python3 "$component_check" "$out_dir/${part_name}.stl" 1
  python3 "$downward_check" "$out_dir/${part_name}.stl" --max-area 40
done

"$openscad_bin" --backend=manifold --render -o "$out_dir/p297-assembly.png" \
  --imgsize=1400,1000 --viewall --autocenter \
  --camera=470,-520,390,55,0,35 -D 'part="p297_assembly"' \
  -D "p297_bts_measured_h=$bts_height" \
  -D "p297_seam_clearance=$seam_clearance" "$scad_file"
"$openscad_bin" --backend=manifold --render -o "$out_dir/p297-rear-service.png" \
  --imgsize=1400,1000 --viewall --autocenter \
  --camera=-520,-430,330,62,0,125 -D 'part="p297_assembly"' \
  -D "p297_bts_measured_h=$bts_height" \
  -D "p297_seam_clearance=$seam_clearance" "$scad_file"
"$openscad_bin" --backend=manifold --render -o "$out_dir/p297-corner-before.png" \
  --imgsize=1200,900 --viewall --autocenter \
  --camera=90,-110,80,4,4,12 -D 'part="p297_corner_before"' \
  -D "p297_seam_clearance=$seam_clearance" "$scad_file"
"$openscad_bin" --backend=manifold --render -o "$out_dir/p297-corner-after-section.png" \
  --imgsize=1200,900 --viewall --autocenter \
  --camera=90,-110,75,0,0,18 -D 'part="p297_corner_after_section"' \
  -D "p297_seam_clearance=$seam_clearance" "$scad_file"

echo "Exported ${#parts[@]} FIT-CHECK STLs (clearance ${seam_clearance} mm/side) and four previews to $out_dir"
