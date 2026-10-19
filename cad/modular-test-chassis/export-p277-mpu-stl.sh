#!/usr/bin/env bash
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source_scad="$here/p277-torsion-node.scad"
release_dir="$here/stl-p277"
openscad_bin="${OPENSCAD_BIN:-/snap/bin/openscad-nightly}"

if [[ "${DRY_RUN:-0}" == "1" ]]; then
  out_dir="$(mktemp -d "$here/.dry-run-p277.XXXXXX")"
  trap 'rm -rf "$out_dir"' EXIT
else
  out_dir="$release_dir"
  mkdir -p "$out_dir"
fi

"$openscad_bin" -o "$out_dir/mpu6050_rail_side_mount.stl" \
  -D 'part="mpu6050_rail_side_mount"' "$source_scad"
[[ -s "$out_dir/mpu6050_rail_side_mount.stl" ]] || {
  echo "Missing MPU export" >&2; exit 1;
}

echo "Validated active P2-77 MPU mount"
