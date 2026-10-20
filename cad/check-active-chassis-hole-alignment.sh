#!/usr/bin/env bash
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
openscad_bin="${OPENSCAD_BIN:-/snap/bin/openscad-nightly}"
src="$here/active-chassis-hole-alignment.scad"
out="$(mktemp -d "$here/.hole-alignment.XXXXXX")"
trap 'rm -rf "$out"' EXIT

interfaces=("$@")
if ((${#interfaces[@]} == 0)); then
  interfaces=(pod_base corner_ties corner_stays ski_brackets mpu
              perimeter_side_rear perimeter_side_mid
              perimeter_x240 perimeter_x300 perimeter_map all)
fi

for name in "${interfaces[@]}"; do
  mesh="$out/$name.stl"
  log="$out/$name.log"
  status=0
  "$openscad_bin" -o "$mesh" -D "interface=\"$name\"" "$src" \
    >"$log" 2>&1 || status=$?
  if [[ -s "$mesh" ]]; then
    cat "$log" >&2
    echo "CHASSIS_HOLE_ALIGNMENT FAIL: $name intersects frame material" >&2
    exit 1
  fi
  if ((status != 0)) && ! grep -qi "top level object is empty" "$log"; then
    cat "$log" >&2
    echo "CHASSIS_HOLE_ALIGNMENT ERROR: $name (status $status)" >&2
    exit 1
  fi
  echo "CHASSIS_HOLE_ALIGNMENT PASS_EMPTY: $name"
done
