#!/usr/bin/env bash
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
openscad_bin="${OPENSCAD_BIN:-/snap/bin/openscad-nightly}"
cad="$here/../../../cad/modular-test-chassis"
out="$cad/stl-v2"
mkdir -p "$out"

"$openscad_bin" -q -D 'part="motor_right"' \
  -o "$out/motor_right.stl" "$cad/modular-chassis-v2.scad"
"$openscad_bin" -q -D 'part="motor_left"' \
  -o "$out/motor_left.stl" "$cad/modular-chassis-v2.scad"

echo "Exported current rear motor modules to $out (not front Gamma/motor parts)"
