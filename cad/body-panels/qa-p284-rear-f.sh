#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
openscad_bin="${OPENSCAD_BIN:-/snap/bin/openscad-nightly}"
scad_file="$root_dir/body-panels-f-rear-v2.scad"
pod_scad="$root_dir/../rear-pod/rear-pod-v2.scad"
qa_dir="$(mktemp -d "$root_dir/.qa-p284.XXXXXX")"
trap 'rm -rf "$qa_dir"' EXIT

expect_empty() {
  local source=$1 selector=$2 label=$3
  local out="$qa_dir/$label-$selector.stl"
  local log="$qa_dir/$label-$selector.log"
  local status=0
  "$openscad_bin" -o "$out" -D "part=\"$selector\"" "$source" \
    >"$log" 2>&1 || status=$?
  if [[ -s "$out" ]]; then
    cat "$log" >&2
    echo "P2-84 $label exact-solid FAIL: $selector produced geometry" >&2
    exit 1
  fi
  if ((status != 0)) && ! grep -q "Current top level object is empty" "$log"; then
    cat "$log" >&2
    echo "P2-84 $label render FAIL: $selector (status $status)" >&2
    exit 1
  fi
  echo "P2-84 $label exact-solid PASS (empty): $selector"
}

empty_checks=(
  qa_skin_pod_3mm qa_chassis qa_wheels qa_moving
  qa_drawer_fixed qa_drawer_open qa_ground
)
for check in "${empty_checks[@]}"; do
  expect_empty "$scad_file" "$check" "rear-F"
done

pod_checks=(
  qa_control_bodies_collision qa_control_ugreen_collision
  qa_control_posts_penetration qa_control_carrier_collision
  qa_base_parkside_collision
)
for check in "${pod_checks[@]}"; do
  expect_empty "$pod_scad" "$check" "pod"
done

echo "P2-84 exact-solid suite passed"
