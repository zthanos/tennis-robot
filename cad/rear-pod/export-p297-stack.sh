#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
scad_file="$root_dir/stackable-box-p297.scad"
out_dir="$root_dir/print-stack"
openscad_bin="${OPENSCAD_BIN:-/snap/bin/openscad-nightly}"
component_check="$root_dir/../check-stl-components.py"

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
)

for part_name in "${parts[@]}"; do
  "$openscad_bin" -o "$out_dir/${part_name}.stl" \
    -D "part=\"${part_name}\"" -D 'show_ghosts=false' "$scad_file"
  [[ -s "$out_dir/${part_name}.stl" ]] || {
    echo "Missing export: $part_name" >&2
    exit 1
  }
  python3 "$component_check" "$out_dir/${part_name}.stl" 1
done

"$openscad_bin" --backend=manifold --render -o "$out_dir/p297-assembly.png" \
  --imgsize=1400,1000 --viewall --autocenter \
  --camera=470,-520,390,55,0,35 -D 'part="p297_assembly"' "$scad_file"
"$openscad_bin" --backend=manifold --render -o "$out_dir/p297-rear-service.png" \
  --imgsize=1400,1000 --viewall --autocenter \
  --camera=-520,-430,330,62,0,125 -D 'part="p297_assembly"' "$scad_file"

echo "Exported ${#parts[@]} FIT-CHECK STLs and two previews to $out_dir"
