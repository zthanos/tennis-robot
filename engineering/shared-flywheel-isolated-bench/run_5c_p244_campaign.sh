#!/usr/bin/env bash
set -euo pipefail
parallelism="${P2_44_PARALLELISM:-5}"
bench=engineering/shared-flywheel-isolated-bench
root=engineering/chassis-strength-bench/results/shared_flywheel_phase2/gazebo_5c_p244_r2
mkdir -p "$root"
matrix="$root/matrix.tsv"; : > "$matrix"
for cheek in exposed_short exposed_shallow exposed_shallow_lowfriction; do
  for lateral in -0.1 -0.05 0.0 0.05 0.1; do
    for repeat in 1 2 3 4 5; do
      printf '%s\t%s\t%s\n' "$cheek" "$lateral" "$repeat" >> "$matrix"
    done
  done
done
export bench root
run_one(){
  local cheek="$1" lateral="$2" repeat="$3"
  local tag="${cheek}/y_${lateral/-/m}"; tag="${tag/./p}/r${repeat}"
  [[ -s "$PWD/$root/$tag/summary.json" ]] && return 0
  "$bench/run_5c_p244_case.sh" "$cheek" "$lateral" "$repeat" >/dev/null
}
export -f run_one
xargs -P "$parallelism" -n 3 bash -c 'run_one "$@"' _ < "$matrix"
python3 "$bench/aggregate_5c_p244_results.py" --root "$root" --matrix "$matrix" --output "$root/summary.json" --csv "$root/condition_matrix.csv"
