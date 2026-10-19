#!/usr/bin/env bash
set -euo pipefail
parallelism="${P2_45_PARALLELISM:-5}"
bench=engineering/shared-flywheel-isolated-bench
root=engineering/chassis-strength-bench/results/shared_flywheel_phase2/gazebo_5c_p245
mkdir -p "$root"
matrix="$root/matrix.tsv"; : > "$matrix"
for tip in 3 5 8; do
  for cheek in exposed_short exposed_shallow exposed_shallow_lowfriction; do
    for lateral in -0.05 0.0 0.05; do
      for repeat in 1 2 3 4 5; do
        printf '%s\t%s\t%s\t%s\n' "$tip" "$cheek" "$lateral" "$repeat" >> "$matrix"
      done
    done
  done
done
export bench root
run_one(){
  local tip="$1" cheek="$2" lateral="$3" repeat="$4"
  local tag="tip_${tip}mm/${cheek}/y_${lateral/-/m}"; tag="${tag/./p}/r${repeat}"
  [[ -s "$PWD/$root/$tag/summary.json" ]] && return 0
  "$bench/run_5c_p245_case.sh" "$tip" "$cheek" "$lateral" "$repeat" >/dev/null
}
export -f run_one
xargs -P "$parallelism" -n 4 bash -c 'run_one "$@"' _ < "$matrix"
python3 "$bench/aggregate_5c_p245_results.py" --root "$root" --matrix "$matrix" --output "$root/summary.json" --csv "$root/condition_matrix.csv"
