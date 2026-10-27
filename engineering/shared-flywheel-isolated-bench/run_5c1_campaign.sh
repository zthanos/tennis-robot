#!/usr/bin/env bash
set -euo pipefail
parallelism="${P2_43_PARALLELISM:-6}"
bench=engineering/shared-flywheel-isolated-bench
root=engineering/chassis-strength-bench/results/shared_flywheel_phase2/gazebo_5c1
mkdir -p "$root"
matrix="$root/matrix.tsv"; : > "$matrix"
for v in 0.35 0.6 0.8; do for y in -0.1 -0.05 0.0 0.05 0.1; do for cheek in narrow wide; do for mu in 0.3 0.6 0.9; do for surface in 2.35 2.50; do for repeat in 1 2 3 4 5; do
  printf '%s\t%s\t%s\t%s\t%s\t%s\n' "$v" "$y" "$cheek" "$mu" "$surface" "$repeat" >> "$matrix"
done;done;done;done;done;done
export bench root
run_one(){
  local v="$1" y="$2" cheek="$3" mu="$4" surface="$5" repeat="$6"
  local tag="v_${v/./p}/y_${y/-/m}"; tag="${tag/./p}/${cheek}/mu_${mu/./p}/surf_${surface/./p}/r${repeat}"
  local summary="$PWD/$root/$tag/summary.json"
  [[ -s "$summary" ]] && return 0
  "$bench/run_5c1_case.sh" "$v" "$y" "$cheek" "$mu" "$surface" "$repeat" >/dev/null
}
export -f run_one
xargs -P "$parallelism" -n 6 bash -c 'run_one "$@"' _ < "$matrix"
python3 "$bench/aggregate_5c1_results.py" --root "$root" --matrix "$matrix" --criteria "$bench/criteria_5c.json" --output "$root/summary.json" --csv "$root/condition_matrix.csv"
