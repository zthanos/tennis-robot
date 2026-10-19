#!/usr/bin/env bash
set -euo pipefail
b=engineering/shared-flywheel-isolated-bench
for mu in 0.6 0.9;do for speed in 12 16 20 24;do for feed in 0 2;do BENCH_RECORD=false "$b/run_5d1_p261_case.sh" "$mu" "$speed" "$feed";done;done;done
python3 "$b/aggregate_5d1_p261.py"
