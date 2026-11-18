#!/usr/bin/env bash
# Retained P2-93d arch parts and P2-96 coupons only.
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
openscad_bin="${OPENSCAD_BIN:-/snap/bin/openscad-nightly}"
printability_check="$here/../check-stl-printability.sh"
right_source=""
trap '[[ -z "$right_source" ]] || rm -f -- "$right_source"' EXIT
mkdir -p "$here/print-arch"
for spec in "p293d_knee:knee:z:20" "p293d_mast:mast:y:0" \
            "p293d_lidar_half:lidar_half:y:0" "p293d_mast_cap:mast_cap:y:0"; do
  IFS=: read -r part name axis height <<<"$spec"
  "$openscad_bin" --backend=manifold -D "part=\"$part\"" \
    -D 'side_mark="L"' \
    -o "$here/print-arch/${name}_L.stl" "$here/p293d-arch-candidate.scad"
  right_source="$(mktemp "$here/.right-marked.XXXXXX.stl")"
  rm -f "$right_source"
  "$openscad_bin" --backend=manifold -D "part=\"$part\"" \
    -D 'side_mark="R"' -o "$right_source" "$here/p293d-arch-candidate.scad"
  "$openscad_bin" --backend=manifold -D "input_file=\"$right_source\"" \
    -D "axis=\"$axis\"" -D "bed_height=$height" -o "$here/print-arch/${name}_R.stl" "$here/p296b-mirror-print.scad"
  rm -f "$right_source"
  right_source=""
  bash "$printability_check" "$here/print-arch/${name}_L.stl" 1
  bash "$printability_check" "$here/print-arch/${name}_R.stl" 1
done
"$openscad_bin" --backend=manifold -D 'part="p293d_oak_mount"' \
  -o "$here/print-arch/oak_mount_x1.stl" "$here/p293d-arch-candidate.scad"
bash "$printability_check" "$here/print-arch/oak_mount_x1.stl" 1
for name in coupon_foot_seats coupon_knee_pins; do
  "$openscad_bin" --backend=manifold -D "part=\"$name\"" \
    -o "$here/print-arch/$name.stl" "$here/p296-coupons.scad"
  bash "$printability_check" "$here/print-arch/$name.stl" 1
done
python3 "$here/verify-p296b-mirrors.py" "$here/print-arch"
echo "P2-96 retained arch + one-body coupons exported."
