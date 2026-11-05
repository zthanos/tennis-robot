#!/usr/bin/env bash
# Candidate only; do not overwrite the P2-93 printable release.
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
openscad_bin="${OPENSCAD_BIN:-/snap/bin/openscad-nightly}"
mkdir -p "$here/stl-candidate-p293d"
for name in p293d_foot p293d_knee p293d_crown p293d_crossbar_end \
            p293d_mast p293d_lidar_half p293d_mast_cap p293d_oak_mount; do
  "$openscad_bin" --backend=manifold -D "part=\"$name\"" \
    -o "$here/stl-candidate-p293d/$name.stl" "$here/p293d-arch-candidate.scad"
done
for side in 1 -1; do
  name=p293e_foot_template
  if [ "$side" = -1 ]; then name=p293e_foot_template_R; fi
  "$openscad_bin" --backend=manifold -D "side=$side" \
    -o "$here/stl-candidate-p293d/$name.stl" "$here/p293e-foot-template.scad"
done
echo "CANDIDATE ONLY: the full-scene gate may still fail on unrelated/F/tilt parts."
bash "$here/../check-current-scene.sh"
