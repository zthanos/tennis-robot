#!/usr/bin/env bash
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
bin="${OPENSCAD_BIN:-/snap/bin/openscad-nightly}"
mkdir -p "$here/print-arch" "$here/parked-tilt"
for spec in "p293g_foot:foot:y:0" "p293g_crown:crown:z:52.8" "p293g_crossbar_end:crossbar_end:y:0"; do
  IFS=: read -r part name axis height <<<"$spec"
  "$bin" --backend=manifold -D "part=\"$part\"" -o "$here/print-arch/${name}_L.stl" "$here/p293g-tilt-candidate.scad"
  "$bin" --backend=manifold -D "input_file=\"$here/print-arch/${name}_L.stl\"" \
    -D "axis=\"$axis\"" -D "bed_height=$height" -o "$here/print-arch/${name}_R.stl" "$here/p296b-mirror-print.scad"
  python3 "$here/../check-stl-components.py" "$here/print-arch/${name}_L.stl" 1
  python3 "$here/../check-stl-components.py" "$here/print-arch/${name}_R.stl" 1
done
for spec in "p293g_console_inboard:console_inboard_x1" "p293g_console_outboard:console_outboard_x1" "p293g_cradle:cradle_x1" "p293g_float_cap:float_cap_x1" "p293g_nut_trunnion:nut_trunnion_x1" "p293g_nut_arm:nut_arm_x1"; do
  part="${spec%%:*}"; name="${spec##*:}"
  "$bin" --backend=manifold -D "part=\"$part\"" -o "$here/parked-tilt/$name.stl" "$here/p293g-tilt-candidate.scad"
done
echo 'P2-96 split export: arch FIT-CHECK; tilt PARKED.'
