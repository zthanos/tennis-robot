# Superseded flywheel-launcher studies

Checkpoint date: 2026-08-26

These files preserve the traceability of earlier flywheel investigations. They are historical, non-authoritative artifacts and must not be used as current mechanical, simulation or manufacturing inputs.

The superseding architecture is the provisionally frozen standalone dual direct-drive D5065 launcher documented in [`standalone-flywheel-launcher.md`](../../../mechanism/standalone-flywheel-launcher.md).

## Archived files

- [`flywheel-launcher-exploration-el.md`](flywheel-launcher-exploration-el.md) — `SUPERSEDED_MOTOR_MOUNT_CONCEPT`, `ANALYSIS_ONLY`. Broad v0 launcher/complete-robot packaging exploration containing multiple wheel orientations, pitch mechanisms, guards and feed concepts. It is background only; the current standalone datums and direct-panel motor arrangement supersede it.
- [`flywheel-launcher-physics-bench-stop-report.md`](flywheel-launcher-physics-bench-stop-report.md) — `FAILED_PRELIMINARY_CAPABILITY_MODEL`. Pre-calibration stop report produced before the authoritative compliant-ball contact law, corrected standalone reconstruction and native capability controller existed.
- [`flywheel-launcher-capability-bench-mechanical-stop-report.md`](flywheel-launcher-capability-bench-mechanical-stop-report.md) — `PRE_EXIT_CORRIDOR_CORRECTION`, `SUPERSEDED_WHEEL_INTERFACE`. Gate-A stop based on the earlier incomplete wheel/hub and exit geometry. It is superseded by the provisional wheel-candidate Gate A, exit-corridor audit and completed capability campaign.
- [`flywheel_launcher_capability_gate_results.json`](config/flywheel_launcher_capability_gate_results.json) — machine-readable snapshot belonging to the archived mechanical stop; not the current capability result.
- [`plot_flywheel_capability_stop_evidence.py`](../../../../scripts/archive/flywheel-launcher/plot_flywheel_capability_stop_evidence.py) — `ANALYSIS_ONLY`. Reproduction helper retained with the stopped preliminary capability evidence; it is not part of the active campaign toolchain.

Historical scientific and engineering statements are intentionally preserved unchanged. A later result may supersede their decision status without rewriting what was measured or known at the time.

## Current authoritative and supporting artifacts

- standalone entry point: [`standalone-flywheel-launcher.md`](../../../mechanism/standalone-flywheel-launcher.md)
- active CAD datum: [`launcher-envelope.scad`](../../../../cad/flywheel-launcher-v0/launcher-envelope.scad)
- standalone Xacro: [`flywheel_launcher_module.urdf.xacro`](../../../../ros2_ws/src/tennis_robot/urdf/components/flywheel_launcher_module.urdf.xacro)
- checkpoint configuration: [`flywheel_launcher_checkpoint.json`](../../../../config/flywheel_launcher_checkpoint.json)
- provisional mechanical Gate A: [`flywheel-wheel-candidate-provisional-gate-a.md`](../../../mechanism/flywheel-wheel-candidate-provisional-gate-a.md)
- corrected exit corridor: [`flywheel-launcher-post-nip-exit-corridor-audit.md`](../../../mechanism/flywheel-launcher-post-nip-exit-corridor-audit.md)
- completed capability campaign: [`flywheel-launcher-capability-validation-report.md`](../../../mechanism/flywheel-launcher-capability-validation-report.md)
- energy-transfer diagnosis: [`flywheel-energy-transfer-root-cause-report.md`](../../../mechanism/flywheel-energy-transfer-root-cause-report.md)
- calibrated ball evidence: [`tennis-ball-compliance-calibration`](../../../mechanism/tennis-ball-compliance-calibration/)

## Audit boundary

The checkpoint did not move complete-robot compact/intake packaging studies. They are a separate, currently modified integration workstream and are not alternative definitions of the standalone launcher. Current standalone documentation and tests do not depend on them.

Two unreferenced generated comparison images (`flywheel-capability-low-energy-traces.png` and `flywheel-capability-measured-stages.png`) were removed rather than archived. Their authoritative plotted evidence remains reproducible from the committed campaign data and plot scripts.

## Terminology/reference audit

- old 0.40 kg wheel and 40 mm wheel references: `VALID_HISTORICAL_REFERENCE` in the staged reconstruction/direct-drive evidence; both carry explicit supersession banners;
- physical barrel wording: `CURRENT_VALID_REFERENCE` only where the current reports explicitly reject the CAD keep-out cylinder as a physical barrel;
- old over/under orientation and pitch/bracket concepts: `VALID_HISTORICAL_REFERENCE` in this archive or in the separately scoped complete-robot CAD narrative, not standalone inputs;
- obsolete 198 mm shaft, external bearing/coupler and printed-hub concepts: no occurrence in the checkpoint's active standalone document/configuration/toolchain;
- pre-relief collision and stopped capability results: `VALID_HISTORICAL_REFERENCE` only in this archive; current active evidence uses the corrected corridor and completed campaign;
- stale active references to the former report paths: none found by repository search.
