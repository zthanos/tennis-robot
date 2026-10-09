#!/usr/bin/env bash
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
out="$here/stl-v2"
openscad_bin="${OPENSCAD_BIN:-/snap/bin/openscad-nightly}"
mkdir -p "$out"

parts=(
  straight_sleeve
  corner_left
  corner_right
  crossbar_half
  gamma_left
  gamma_right
  ramp_cradle_left
  ramp_cradle_right
  electronics_tray_left
  electronics_tray_right
)

for part in "${parts[@]}"; do
  "$openscad_bin" -q \
    -D "part=\"$part\"" \
    -o "$out/$part.stl" \
    "$here/modular-chassis-v2.scad"
done

"$openscad_bin" -q \
  --imgsize=1600,1000 \
  --viewall \
  --autocenter \
  -D 'part="assembly"' \
  -o "$here/modular-chassis-v2-assembly.png" \
  "$here/modular-chassis-v2.scad"

echo "Exported ${#parts[@]} integration parts to $out"
