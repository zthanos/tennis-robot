#!/usr/bin/env bash
# Gazebo intake campaign S1-S5 on the CORRECTED frozen-CAD intake.
#
# Gazebo owns ramp wedge/plough dynamics with ground friction, cheek centering,
# capture half-width, jam/reject modes and 6-DOF multi-contact. It does NOT own
# exit velocity, tyre deflection, wheel droop, torque or current: velocity
# commands here are an ideal velocity source (D4).
#
# Stages gate each other (task section 9). S1 must show the ball reaching the
# nip before S2-S5 can measure anything.
#
#   S1  ramp dynamics, Phase 3 (wheels + ramp, no cheeks), centred
#   S2  capture half-width, Phase 4 (full intake), lateral offset sweep
#   S5  wheel-speed sweep, Phase 4, centred
#
# Every run is one headless simulation. Results land under
# runtime/intake_campaign/<stage>/ and are compiled by
# scripts/analyze_intake_gazebo_campaign.py.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"

OUT_ROOT="${INTAKE_CAMPAIGN_OUT:-$ROOT/runtime/intake_campaign}"
SWEEP="$ROOT/scripts/sim_debug/run_native_intake_sweep.sh"

# The compact variant is the only one carrying the frozen CAD cheeks, ramp and
# bridge. The launcher is off: it is not part of this measurement.
export ROBOT_PACKAGING_VARIANT=compact
export ROBOT_ENABLE_FLYWHEEL=false
export INTAKE_SWEEP_REPEATS="${INTAKE_SWEEP_REPEATS:-1}"

run_case() {
    local stage="$1" name="$2"; shift 2
    local dir="$OUT_ROOT/$stage/$name"
    if [ -f "$dir/summary.csv" ] && [ "${INTAKE_CAMPAIGN_FORCE:-false}" != "true" ]; then
        echo "[$stage/$name] already present, skipping"
        return 0
    fi
    mkdir -p "$dir"
    echo "=== [$stage/$name] $* ==="
    env "$@" INTAKE_SWEEP_OUT_DIR="$dir" bash "$SWEEP" > "$dir/sweep.log" 2>&1 \
        && echo "[$stage/$name] ok" \
        || echo "[$stage/$name] FAILED (see $dir/sweep.log)"
}

# --- S1: does the ball climb, stall, or get ploughed ahead? ---------------
for drive in 0.35 0.60 0.80; do
    for mu in 0.3 0.6 0.9; do
        run_case S1 "drive${drive}_mu${mu}_rampmu0.40" \
            INTAKE_SWEEP_PHASE=ramp \
            INTAKE_SWEEP_DRIVE_SPEEDS="$drive" \
            INTAKE_SWEEP_TREAD_MUS="$mu" \
            INTAKE_RAMP_MU=0.40 \
            INTAKE_SWEEP_WHEEL_SPEEDS=25.0
    done
done
for rampmu in 0.20 0.60; do
    run_case S1 "drive0.80_mu0.6_rampmu${rampmu}" \
        INTAKE_SWEEP_PHASE=ramp \
        INTAKE_SWEEP_DRIVE_SPEEDS=0.80 \
        INTAKE_SWEEP_TREAD_MUS=0.6 \
        INTAKE_RAMP_MU="$rampmu" \
        INTAKE_SWEEP_WHEEL_SPEEDS=25.0
done

# --- S2: capture half-width with the cheeks fitted ------------------------
for lateral in 0.000 0.020 0.040 0.060 0.080; do
    run_case S2 "lat${lateral}_drive0.80" \
        INTAKE_SWEEP_PHASE=full \
        INTAKE_SWEEP_DRIVE_SPEEDS=0.80 \
        INTAKE_SWEEP_TREAD_MUS=0.6 \
        INTAKE_RAMP_MU=0.40 \
        INTAKE_SWEEP_WHEEL_SPEEDS=25.0 \
        INTAKE_SWEEP_BALL_LATERAL_OFFSETS="$lateral"
done

# --- S5: wheel-speed sweep (LOW/NOMINAL/HIGH of 26.3 rad/s) ---------------
for wheel in 14.5 19.7 25.0; do
    run_case S5 "wheel${wheel}_drive0.80" \
        INTAKE_SWEEP_PHASE=full \
        INTAKE_SWEEP_DRIVE_SPEEDS=0.80 \
        INTAKE_SWEEP_TREAD_MUS=0.6 \
        INTAKE_RAMP_MU=0.40 \
        INTAKE_SWEEP_WHEEL_SPEEDS="$wheel"
done

echo "campaign complete: $OUT_ROOT"
