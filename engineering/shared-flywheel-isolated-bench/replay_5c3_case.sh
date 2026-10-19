#!/usr/bin/env bash
set -euo pipefail
if [[ $# -lt 2 || $# -gt 3 ]]; then
  echo "usage: $0 CASE {0.6|0.9} [continuous|stop_1s|reverse_75mm]" >&2; exit 2
fi
case_name="$1"; tread_mu="$2"; profile="${3:-continuous}"
case "$case_name" in
  series_gap_0|series_gap_50|series_gap_150|series_gap_300|side_by_side_y50|side_by_side_y100) ;;
  p256_reverse_both_y50|p256_reverse_both_y100|p256_asym_left_y50|p256_asym_left_y100|p256_reverse150_yaw12_y50|p256_reverse150_yaw12_y100) ;;
  p257_shield_m90__*|p257_shield_m100__*|p257_shield_m110__*|p257_stagger_open__*|p257_stagger_shield__*) ;;
  p258_dx*_dy*_lead_plus|p258_dx*_dy*_lead_minus|p258_dx*_dy*_center) ;;
  p261_side_by_side_y50|p261_p258_dx*_dy*_lead_plus|p261_p258_dx*_dy*_lead_minus) ;;
  *) exit 2;;
esac
case "$tread_mu" in 0.6|0.9) ;; *) exit 2;; esac
case "$profile" in continuous|stop_1s|reverse_75mm) ;; *) exit 2;; esac
bench=engineering/shared-flywheel-isolated-bench; tag="${tread_mu/./p}"
base="$(pwd)/engineering/chassis-strength-bench/results/shared_flywheel_phase2/gazebo_5c3_p255"
if [[ "$case_name" == p261_side_by_side_y50 ]]; then
  base="$(pwd)/engineering/chassis-strength-bench/results/shared_flywheel_phase2/gazebo_5c3_p261_p255"
  case_name=side_by_side_y50
elif [[ "$case_name" == p261_p258_dx*_dy*_* ]]; then
  token="${case_name#p261_p258_dx}"; dx="${token%%_dy*}"
  token="${token#*_dy}"; dy="${token%%_*}"; orientation="${token#*_}"
  base="$(pwd)/engineering/chassis-strength-bench/results/shared_flywheel_phase2/gazebo_5c3_p261_p258"
  replay="$base/matrix/tm_${tag}/$orientation/dx_$dx/dy_$dy/replay"
elif [[ "$case_name" == p258_dx*_dy*_* ]]; then
  token="${case_name#p258_dx}"; dx="${token%%_dy*}"
  token="${token#*_dy}"; dy="${token%%_*}"; orientation="${token#*_}"
  case "$dx" in 0|35|70|100|150) ;; *) exit 2;; esac
  case "$dy" in 0|20|35|50|70|100) ;; *) exit 2;; esac
  case "$orientation" in lead_plus|lead_minus|center) ;; *) exit 2;; esac
  base="$(pwd)/engineering/chassis-strength-bench/results/shared_flywheel_phase2/gazebo_5c3_p258"
  replay="$base/matrix/tm_${tag}/$orientation/dx_$dx/dy_$dy/replay"
elif [[ "$case_name" == p257_*__* ]]; then
  variant="${case_name%%__*}"
  case_name="${case_name#*__}"
  case "$case_name" in
    side_y35|side_y50|side_y100|series_gap_0|series_gap_50|series_gap_150|series_gap_300|single_y0|single_ym20|single_yp20|single_ym40|single_yp40|single_ym60|single_yp60|single_ym80|single_yp80|single_ym100|single_yp100) ;;
    *) echo "unknown P2-57 replay case" >&2; exit 2 ;;
  esac
  base="$(pwd)/engineering/chassis-strength-bench/results/shared_flywheel_phase2/gazebo_5c3_p257"
  replay="$base/matrix/$variant/tm_${tag}/$case_name/replay"
elif [[ "$case_name" == p256_* ]]; then
  base="$(pwd)/engineering/chassis-strength-bench/results/shared_flywheel_phase2/gazebo_5c3_p256"
  case "$case_name" in
    p256_reverse_both_y50) profile=p256_reverse_both; case_name=side_by_side_y50 ;;
    p256_reverse_both_y100) profile=p256_reverse_both; case_name=side_by_side_y100 ;;
    p256_asym_left_y50) profile=p256_asym_left; case_name=side_by_side_y50 ;;
    p256_asym_left_y100) profile=p256_asym_left; case_name=side_by_side_y100 ;;
    p256_reverse150_yaw12_y50) profile=p256_reverse150_yaw12; case_name=side_by_side_y50 ;;
    p256_reverse150_yaw12_y100) profile=p256_reverse150_yaw12; case_name=side_by_side_y100 ;;
  esac
fi
if [[ -z "${replay:-}" ]]; then
  if [[ "$profile" == continuous ]]; then replay="$base/matrix/tm_${tag}/$case_name/replay"; else replay="$base/diagnostics/$profile/tm_${tag}/$case_name/replay"; fi
fi
[[ -f "$replay/state.tlog" ]] || { echo "missing replay log: $replay/state.tlog" >&2; exit 1; }
ROS_DOMAIN_ID=88 GAZEBO_HEADLESS=false docker/jazzy/run.sh exec "gz sim -v 2 --gui-config $bench/5c3_side_gui.config --playback $replay"
