#!/usr/bin/env bash
set -euo pipefail
if [[ $# -ne 2 ]]; then
  echo "usage: $0 P256_CASE {0.6|0.9}" >&2; exit 2
fi
new_case="$1"; tread_mu="$2"
case "$new_case" in
  p256_reverse_both_y50) geometry=side_by_side_y50; profile=p256_reverse_both ;;
  p256_reverse_both_y100) geometry=side_by_side_y100; profile=p256_reverse_both ;;
  p256_asym_left_y50) geometry=side_by_side_y50; profile=p256_asym_left ;;
  p256_asym_left_y100) geometry=side_by_side_y100; profile=p256_asym_left ;;
  p256_reverse150_yaw12_y50) geometry=side_by_side_y50; profile=p256_reverse150_yaw12 ;;
  p256_reverse150_yaw12_y100) geometry=side_by_side_y100; profile=p256_reverse150_yaw12 ;;
  *) echo "unknown P2-56 case: $new_case" >&2; exit 2 ;;
esac
root=engineering/chassis-strength-bench/results/shared_flywheel_phase2/gazebo_5c3_p256
P2_55_RESULT_ROOT="$root" P2_55_DRIVE_PROFILE="$profile" \
  BENCH_GUI=false BENCH_RECORD=true \
  engineering/shared-flywheel-isolated-bench/run_5c3_p255_case.sh "$geometry" "$tread_mu"

