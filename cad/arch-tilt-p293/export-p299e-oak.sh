#!/usr/bin/env bash
# P2-99e OAK-D S2 bracket (position B, oak_pitch_deg default 25) - FIT-CHECK.
# Replaces oak_mount_x1.stl (p293d_oak_mount, 50 deg at X450).
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
openscad_bin="${OPENSCAD_BIN:-/snap/bin/openscad-nightly}"
mkdir -p "$here/print-arch"
"$openscad_bin" --backend=manifold -D 'part="p299e_oak_bracket"' \
  -o "$here/print-arch/p299e_oak_bracket.stl" "$here/p299e-oak-bracket.scad"
bash "$here/../check-stl-printability.sh" "$here/print-arch/p299e_oak_bracket.stl" 1
