#!/usr/bin/env bash
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
openscad_bin="${OPENSCAD_BIN:-/snap/bin/openscad-nightly}"

"$openscad_bin" -q -D 'part="right"' \
  -o "$here/drive-motor-inline-RIGHT-M3-validated-dual-splice.stl" \
  "$here/drive-motor-inline-module.scad"
"$openscad_bin" -q -D 'part="left"' \
  -o "$here/drive-motor-inline-LEFT-M3-validated-dual-splice.stl" \
  "$here/drive-motor-inline-module.scad"

echo "Exported LEFT/RIGHT motor modules with universal splice-lock holes"
