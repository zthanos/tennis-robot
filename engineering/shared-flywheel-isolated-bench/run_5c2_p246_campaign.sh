#!/usr/bin/env bash
set -euo pipefail
parallelism="${P2_46_PARALLELISM:-3}"
bench=engineering/shared-flywheel-isolated-bench
root=engineering/chassis-strength-bench/results/shared_flywheel_phase2/gazebo_5c2_p246
mkdir -p "$root"; matrix="$root/matrix.tsv"; : > "$matrix"
for preload in 0 10 25 40; do for repeat in 1 2 3 4 5; do printf 'delivery\t%s\t%s\n' "$preload" "$repeat" >> "$matrix"; done; done
for preload in 44 49 54 59; do for repeat in 1 2 3; do printf 'delivery\t%s\t%s\n' "$preload" "$repeat" >> "$matrix"; done; done
for repeat in 1 2 3 4 5; do printf 'motion\t45\t%s\n' "$repeat" >> "$matrix"; done
export bench root
run_one(){
  local mode="$1" preload="$2" repeat="$3" out="$PWD/$root/$mode/preload_${preload}/r${repeat}"
  [[ -s "$out/summary.json" ]] && return 0
  "$bench/run_5c2_p246_case.sh" "$mode" "$preload" "$repeat" >/dev/null
}
export -f run_one
xargs -P "$parallelism" -n 3 bash -c 'run_one "$@"' _ < "$matrix"
python3 "$bench/aggregate_5c2_p246_results.py" --root "$root" --matrix "$matrix" --output "$root/summary.json" --csv "$root/condition_matrix.csv"
