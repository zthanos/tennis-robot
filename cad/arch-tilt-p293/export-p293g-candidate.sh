#!/usr/bin/env bash
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
bin="${OPENSCAD_BIN:-/snap/bin/openscad-nightly}"
mkdir -p "$here/stl-candidate-p293g"
for name in p293g_foot p293g_crown p293g_crossbar_end p293g_console_inboard p293g_console_outboard p293g_cradle p293g_float_cap p293g_nut_trunnion p293g_nut_arm; do
  "$bin" --backend=manifold -D "part=\"$name\"" -o "$here/stl-candidate-p293g/$name.stl" "$here/p293g-tilt-candidate.scad"
done
echo 'P2-93g FIT-CHECK only. Physical coupons, proof-load and full scene gate required.'
