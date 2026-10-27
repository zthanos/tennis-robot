#!/usr/bin/env bash
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
release_dir="$here/stl-v3"
openscad_bin="${OPENSCAD_BIN:-/snap/bin/openscad-nightly}"
component_check="$here/../check-stl-components.py"
if [[ "${DRY_RUN:-0}" == "1" ]]; then
  out="$(mktemp -d "$here/.dry-run-v3.XXXXXX")"
  trap 'rm -rf "$out"' EXIT
else
  out="$release_dir"
  mkdir -p "$out"
fi

all_parts=(
  rear_motor_left rear_motor_right
  rail_extension_left rail_extension_right
  front_terminal_left front_terminal_right
  rear_crossbar_half x240_crossbar_half x300_crossbar_half
  clamp_sleeve x240_clamp_sleeve x300_clamp_sleeve
  x240_terminal_sleeve x300_terminal_sleeve
  corner_clamp_left corner_clamp_right
  torsion_side_brace
  sleeve_fit_coupon
)

# Optional part names allow a correction release to export only changed/new
# meshes without touching already printed or previously validated STLs.
parts=("${all_parts[@]}")
if (($#)); then
  parts=("$@")
fi

for name in "${parts[@]}"; do
  "$openscad_bin" -q -D "part=\"$name\"" \
    -o "$out/$name.stl" "$here/modular-chassis-v3.scad"
  [[ -s "$out/$name.stl" ]] || { echo "Missing export: $name" >&2; exit 1; }
  python3 "$component_check" "$out/$name.stl" 1
done

for view in top side; do
  camera="0,0,0,0,0,0,900"
  [[ "$view" == "side" ]] && camera="0,0,0,90,0,0,900"
  "$openscad_bin" -D 'part="assembly"' --imgsize=1600,1000 \
    --viewall --autocenter --projection=ortho --camera="$camera" \
    -o "$out/modular-chassis-v3-$view.png" "$here/modular-chassis-v3.scad"
  [[ -s "$out/modular-chassis-v3-$view.png" ]] || {
    echo "Missing render: $view" >&2; exit 1;
  }
done

echo "Exported ${#parts[@]} unique v3 meshes to $out"
