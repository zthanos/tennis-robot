#!/usr/bin/env bash
set -euo pipefail
parallelism="${P2_47_PARALLELISM:-3}"
variants="${P2_47_VARIANTS:-high_hood one_way_flap lip_cover}"
include_motion="${P2_47_INCLUDE_MOTION:-1}"
bench=engineering/shared-flywheel-isolated-bench
root=engineering/chassis-strength-bench/results/shared_flywheel_phase2/gazebo_5c2_p247_r3
mkdir -p "$root"; matrix="$root/matrix.tsv"; : > "$matrix"
for variant in $variants; do
  for preload in 0 25 40 44; do for repeat in 1 2 3; do printf '%s\tdelivery\t%s\t%s\n' "$variant" "$preload" "$repeat" >> "$matrix"; done; done
  if [[ "$include_motion" == 1 ]]; then for repeat in 1 2 3; do printf '%s\tmotion\t45\t%s\n' "$variant" "$repeat" >> "$matrix"; done; fi
done
export bench root
run_one(){
  local variant="$1" mode="$2" preload="$3" repeat="$4" out="$PWD/$root/$variant/$mode/preload_${preload}/r${repeat}"
  [[ -s "$out/summary.json" ]] && return 0
  "$bench/run_5c2_p247_case.sh" "$variant" "$mode" "$preload" "$repeat" >/dev/null
}
export -f run_one
xargs -P "$parallelism" -n 4 bash -c 'run_one "$@"' _ < "$matrix"
if [[ "${P2_47_VARIANTS:-}" == "" || "${P2_47_FINAL_AGGREGATE:-0}" == 1 ]]; then
  python3 "$bench/aggregate_5c2_p247_results.py" --root "$root" --matrix "$matrix" --output "$root/summary.json" --csv "$root/condition_matrix.csv"
fi
