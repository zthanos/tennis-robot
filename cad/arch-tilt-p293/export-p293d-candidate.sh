#!/usr/bin/env bash
# Retained P2-93d arch parts and P2-96 coupons only.
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
openscad_bin="${OPENSCAD_BIN:-/snap/bin/openscad-nightly}"
mkdir -p "$here/print-arch"
for spec in "p293d_knee:knee_L-R_x1_each" "p293d_mast:mast_L-R_x1_each" \
            "p293d_lidar_half:lidar_half_L-R_x1_each" "p293d_mast_cap:mast_cap_L-R_x1_each" \
            "p293d_oak_mount:oak_mount_x1"; do
  part="${spec%%:*}"; name="${spec##*:}"
  "$openscad_bin" --backend=manifold -D "part=\"$part\"" \
    -o "$here/print-arch/$name.stl" "$here/p293d-arch-candidate.scad"
done
for name in coupon_foot_seats coupon_knee_pins; do
  "$openscad_bin" --backend=manifold -D "part=\"$name\"" \
    -o "$here/print-arch/$name.stl" "$here/p296-coupons.scad"
  python3 "$here/../check-stl-components.py" "$here/print-arch/$name.stl" 1
done
echo "P2-96 retained arch + one-body coupons exported."
