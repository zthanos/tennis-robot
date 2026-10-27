#!/usr/bin/env bash
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
openscad_bin="${OPENSCAD_BIN:-/snap/bin/openscad-nightly}"
out="$(mktemp -d "$here/.qa.XXXXXX")"
trap 'rm -rf "$out"' EXIT

empty_checks=(
  arch_frame_forbidden arch_basket arch_basket_removal arch_ski arch_rear_pod
  arch_moving arch_f tilt_f tilt_basket tilt_ski tilt_rear_pod tilt_arch
  fixed_tilt_vs_moving hole_axes printed_joint_axes fastener_tool_paths
  arch_install_path as5600_install_path
)

for name in "${empty_checks[@]}"; do
  mesh="$out/$name.stl"; log="$out/$name.log"; status=0
  "$openscad_bin" --backend=Manifold -o "$mesh" -D "check=\"$name\"" \
    "$here/p293-qa.scad" >"$log" 2>&1 || status=$?
  if [[ -s "$mesh" ]]; then
    volume="$(python3 "$here/p293-stl-volume.py" "$mesh")"
    awk -v v="$volume" 'BEGIN{exit !(v>0.05)}' && {
      cat "$log" >&2; echo "P2-93 QA FAIL_INTERSECTION: $name volume=${volume}mm3" >&2; exit 1;
    }
  fi
  if ((status!=0)) && ! grep -qi "top level object is empty" "$log"; then
    cat "$log" >&2; echo "P2-93 QA ERROR: $name" >&2; exit 1
  fi
  echo "P2-93 QA PASS_EMPTY: $name"
done

# Known/required conflict: the temporary console must intersect F.
console="$out/console_f.stl"
"$openscad_bin" --backend=Manifold -o "$console" -D 'check="console_f"' "$here/p293-qa.scad" >/dev/null 2>&1 || true
[[ -s "$console" ]] || { echo "P2-93 QA FAIL: expected drive-test console/F conflict absent" >&2; exit 1; }
console_volume="$(python3 "$here/p293-stl-volume.py" "$console")"
awk -v v="$console_volume" 'BEGIN{exit !(v>0.05)}' || {
  echo "P2-93 QA FAIL: console/F contact has no solid volume" >&2; exit 1;
}
echo "P2-93 QA EXPECTED_INTERSECTION: console_f"

echo "P2-93 exact-solid regression PASS"
