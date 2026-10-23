#!/usr/bin/env bash
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
openscad_bin="${OPENSCAD_BIN:-/snap/bin/openscad-nightly}"
src="$here/p288-assemblability.scad"
out="$(mktemp -d "$here/.assemblability.XXXXXX")"
trap 'rm -rf "$out"' EXIT

checks=(side_frame_front_driver side_frame_rear_wrench
        side_frame_front_nut side_frame_rear_driver dock_mount_driver
        parkside_lock_driver ugreen_driver)

for name in "${checks[@]}"; do
  mesh="$out/$name.stl"
  log="$out/$name.log"
  status=0
  "$openscad_bin" -o "$mesh" -D "check=\"$name\"" "$src" >"$log" 2>&1 || status=$?
  if [[ -s "$mesh" ]]; then
    cat "$log" >&2
    echo "ASSEMBLABILITY FAIL: $name" >&2
    exit 1
  fi
  if ((status != 0)) && ! grep -qi "top level object is empty" "$log"; then
    cat "$log" >&2
    echo "ASSEMBLABILITY ERROR: $name (status $status)" >&2
    exit 1
  fi
  echo "ASSEMBLABILITY PASS_EMPTY: $name"
done

for selector in qa_p288_reservation_overflow qa_p288_ugreen_device_collision; do
  mesh="$out/$selector.stl"
  log="$out/$selector.log"
  status=0
  "$openscad_bin" -o "$mesh" -D "part=\"$selector\"" \
    "$here/rear-pod/rear-pod-v2.scad" >"$log" 2>&1 || status=$?
  if [[ -s "$mesh" ]]; then
    cat "$log" >&2
    echo "ASSEMBLABILITY FAIL: $selector" >&2
    exit 1
  fi
  if ((status != 0)) && ! grep -qi "top level object is empty" "$log"; then
    cat "$log" >&2
    echo "ASSEMBLABILITY ERROR: $selector (status $status)" >&2
    exit 1
  fi
  echo "ASSEMBLABILITY PASS_EMPTY: $selector"
done

for selector in docks_pod_frame docks_panels_closed docks_panels_service \
                ugreen_surroundings ugreen_panels new_parts_ground; do
  mesh="$out/integration-$selector.stl"
  log="$out/integration-$selector.log"
  status=0
  "$openscad_bin" -o "$mesh" -D "check=\"$selector\"" \
    "$here/p288-integration-qa.scad" >"$log" 2>&1 || status=$?
  if [[ -s "$mesh" ]]; then
    cat "$log" >&2
    echo "ASSEMBLABILITY FAIL: integration-$selector" >&2
    exit 1
  fi
  if ((status != 0)) && ! grep -qi "top level object is empty" "$log"; then
    cat "$log" >&2
    echo "ASSEMBLABILITY ERROR: integration-$selector (status $status)" >&2
    exit 1
  fi
  echo "ASSEMBLABILITY PASS_EMPTY: integration-$selector"
done

# P2-90: swept solids for every declared removable object/service sequence.
for selector in ugreen parkside carrier pi; do
  mesh="$out/insertion-$selector.stl"
  log="$out/insertion-$selector.log"
  status=0
  "$openscad_bin" -o "$mesh" -D "check=\"$selector\"" \
    "$here/p290-insertion-qa.scad" >"$log" 2>&1 || status=$?
  if [[ -s "$mesh" ]]; then
    cat "$log" >&2
    echo "INSERTION_PATH FAIL: $selector" >&2
    exit 1
  fi
  if ((status != 0)) && ! grep -qi "top level object is empty" "$log"; then
    cat "$log" >&2
    echo "INSERTION_PATH ERROR: $selector (status $status)" >&2
    exit 1
  fi
  echo "INSERTION_PATH PASS_EMPTY: $selector"
done

# P2-90b: installation sweep for every rear-pod part in declared order.
for selector in corner_ties bases plate side_frames posts panel docks holder \
                clamp_caps corner_stays; do
  mesh="$out/part-install-$selector.stl"
  log="$out/part-install-$selector.log"
  status=0
  "$openscad_bin" -o "$mesh" -D "check=\"$selector\"" \
    "$here/p290b-part-installation-qa.scad" >"$log" 2>&1 || status=$?
  if [[ -s "$mesh" ]]; then
    cat "$log" >&2
    echo "PART_INSTALL_PATH FAIL: $selector" >&2
    exit 1
  fi
  if ((status != 0)) && ! grep -qi "top level object is empty" "$log"; then
    cat "$log" >&2
    echo "PART_INSTALL_PATH ERROR: $selector (status $status)" >&2
    exit 1
  fi
  echo "PART_INSTALL_PATH PASS_EMPTY: $selector"
done

# Every chassis-sharing screw also passes the permanent thin-axis probe.
bash "$here/check-active-chassis-hole-alignment.sh" \
  pod_base corner_ties corner_stays ski_brackets mpu all

if [[ "${1:-all}" != "p288" ]]; then
  bash "$here/modular-test-chassis/p285-qa.sh"
  bash "$here/body-panels/qa-p286-rear-f.sh"
fi

echo "ASSEMBLABILITY PASS: modeled hardware and >=60 mm tool approaches"
