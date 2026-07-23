# C2 native-q100 calibration amendment coverage report

## Coverage result

- Gate: **PASS**
- Associated target evidence rows: 270 (30 for each of 9 range×quality bins).
- Existing q90/q98 rows were retained; only `r0_q100`, `r1_q100` and
  `r2_q100` were captured in this amendment.
- Every q100 row has `injected_missing_pixel_ratio=0.0` and
  `depth_roi_quality=1.0`.
- Target association, timestamp-aligned GT/TF, measured-range readiness and
  target outlier policy passed for each native trial.

| Native trial | measured target range (m) | target samples | target outlier rate |
| --- | ---: | ---: | ---: |
| r0_q100 | 1.021663 | 30 | 0.0% |
| r1_q100 | 1.591317 | 30 | 0.0% |
| r2_q100 | 2.979881 | 30 | 0.0% |

## Fit result

The full 3×3 evidence set passed per-axis conservation in every declared bin.
It produced the separate candidate artifact
`calibration_artifacts/gazebo/range_depth_quality_diagonal_v1-gazebo-v2.json`
with depth-quality domain `[0.8979591836734694, 1.0]`. The v1 artifact was
neither replaced nor loaded.

Producer activation of v2 is intentionally outside this amendment and needs a
separate review/gate.
