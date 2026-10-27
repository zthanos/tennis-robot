# Intake architecture archive and audit ledger

This directory preserves superseded intake engineering work. It is historical
evidence only and must not be used as the current physical intake definition.

The current physical architecture is the fixed-motor, direct-drive,
compliant-tyre intake defined by
[`standalone-intake-fixed-motor-compliant-tyre.md`](../../../mechanism/standalone-intake-fixed-motor-compliant-tyre.md)
and
[`standalone_intake_fixed_motor_checkpoint.json`](../../../../config/standalone_intake_fixed_motor_checkpoint.json).
The motors are rigidly mounted; compliance comes from the pneumatic tyres.
The current 35-degree wheel→motor axes lie in the longitudinal X-Z plane and
are parallel in front view. Archived reports may show the superseded splayed
Y-Z/V arrangement; that orientation is historical evidence, not current CAD.

## Pre-move audit ledger

This ledger was written before the archive moves. Every inspected intake-related
artifact is assigned exactly one classification. `ARCHIVE` destinations are the
paths established by this cleanup; `KEEP` means the artifact remains active.

| Artifact or bounded group | Classification | Action | Reason |
|---|---|---|---|
| `docs/mechanism/standalone-intake-fixed-motor-compliant-tyre.md` | `CURRENT_AUTHORITATIVE` | KEEP | Current physical architecture and open validation gates. |
| `config/standalone_intake_fixed_motor_checkpoint.json` | `CURRENT_AUTHORITATIVE` | KEEP | Machine-readable current physical contract. |
| `cad/standalone-intake-fixed-motor/standalone-intake-fixed-motor-compliant-tyre.scad` | `CURRENT_AUTHORITATIVE` | KEEP | Current standalone architecture CAD checkpoint. |
| `tests/test_standalone_intake_fixed_motor_checkpoint.py` | `CURRENT_AUTHORITATIVE` | KEEP | Regression enforcement for the current checkpoint. |
| `cad/collector-intake-v1/option-a/` and `cad/collector-intake-v1/motor-fit-gauge/` | `CURRENT_SUPPORTING_EVIDENCE` | KEEP; repair documentation/export list | Fixed motor-mount and fit-check evidence; stale bearing/hub claims are not architecture. |
| `config/compact_mechanical_contract.json` and `config/compact_cad_measurements.json` | `CURRENT_SUPPORTING_EVIDENCE` | KEEP | Axis/rest-envelope and integration evidence; not the physical compliance definition. |
| `docs/mechanism/intake-concept-decision-el.md` | `CURRENT_SUPPORTING_EVIDENCE` | KEEP; repair links | Decision evidence rejecting the single-wide-roller concept. |
| `docs/hardware/ordered-parts.md`, `docs/hardware/prototype-purchase-list-el.md`, and `docs/hardware/chassis-layout-4wd-dual-intake-el.md` | `CURRENT_SUPPORTING_EVIDENCE` | KEEP; repair architecture/BOM wording | Ownership, purchasing, and chassis integration evidence. |
| Intake orientation/metrology evidence referenced by the current report, including tennis-ball calibration and compact CAD/URDF alignment records | `CURRENT_SUPPORTING_EVIDENCE` | KEEP | Supports axis, envelope, and metrology gates without defining another architecture. |
| Intake Xacro/SDF generated models, URDF generator, ros2-control mapping, native simulation probes/sweeps, and their simulation tests | `CURRENT_SIMULATION_SURROGATE` | KEEP; label `NOT_PHYSICAL_INTAKE_ARCHITECTURE` | Active simulation actuation/contact surrogate only. |
| `docs/mechanism/compact-handoff-regression-repair.md` and `docs/mechanism/compact-mechanical-reconstruction-report.md` | `CURRENT_SIMULATION_SURROGATE` | KEEP; label surrogate | Useful model/telemetry evidence, not a physical intake definition. |
| `docs/mechanism/standalone-intake-guide-definition-report.md`, `config/standalone_intake_guide_definition.json`, and `cad/standalone-intake-guide/standalone-intake-guide-definition.scad` | `HISTORICAL_SUPERSEDED` | ARCHIVE | Size-9 guide/contact proposal used the wrong compliance architecture. |
| `docs/mechanism/standalone-intake-validation-report.md`, `config/standalone_intake_checkpoint.json`, and `tests/test_standalone_intake_checkpoint.py` | `HISTORICAL_SUPERSEDED` | ARCHIVE | Earlier translating/carriage standalone checkpoint. |
| `docs/mechanism/dual-wheel-intake-design-el.md`, `docs/mechanism/intake-bench-sweep-report-el.md`, and `docs/mechanism/intake-debug-log-el.md` | `HISTORICAL_SUPERSEDED` | ARCHIVE | Earlier carriage and single-roller simulation/design records. |
| Root `cad/collector-intake-v1/intake-structure*`, `export-structure-stls.sh`, and root `stl/` outputs | `HISTORICAL_SUPERSEDED` | ARCHIVE | Pre-Option-A straight-cheek/aluminium-rail study. |
| Compact moving-pod/carriage stop reports, pod concept report/data, basket launch reevaluation, and corrected packaging resolution report | `HISTORICAL_SUPERSEDED` | ARCHIVE | Records the superseded translating-pod physical premise. |
| Compact pod/corrected-packaging CAD and analysis sources still imported by launcher integration tooling | `HISTORICAL_SUPERSEDED` | KEEP; add historical warning | Kept only to preserve the existing launcher analysis dependency chain; not an active intake definition. |
| `cad/archive/single-motor-geared-intake/` and `docs/archive/mechanical/concept-a-single-wide-roller-summary.md` | `HISTORICAL_SUPERSEDED` | KEEP IN ARCHIVE | Already correctly archived historical concepts. |
| Runtime output, Python caches, ROS/Gazebo build/install/log trees, and generated previews not selected as evidence | `GENERATED_TRANSIENT` | DO NOT ARCHIVE | Reproducible or ephemeral outputs are not engineering history. |
| Flywheel launcher geometry, basket/bin geometry, drivetrain, perception, navigation, and unrelated documentation/tests | `UNRELATED_KEEP` | KEEP | Outside this intake architecture cleanup. |

## Archive rules

- Historical files retain their original engineering conclusions. Only archive
  banners and links may be repaired during this cleanup.
- A file in this archive cannot override the current authoritative report or
  checkpoint.
- Active simulation carriage geometry is explicitly a surrogate and is not
  evidence that the physical motors translate.
- Physical architecture remains validation-pending until tyre-deflection,
  metrology, and bridge-opening gates in the current report are closed.

## Archived set

- Standalone translating checkpoints: `standalone-intake-validation-report.md`,
  `standalone-intake-guide-definition-report.md`, `config/`, and `tests/`.
- Earlier intake design/simulation history: `dual-wheel-intake-design-el.md`,
  `intake-bench-sweep-report-el.md`, and `intake-debug-log-el.md`.
- Compact moving-pod history: the five `compact-*` reports plus
  `compact-intake-pod-concept-study-measurements.json`.
- Superseded CAD: [`cad/archive/intake/`](../../../../cad/archive/intake/).

These were superseded because they either model compliance as motor/wheel
translation, retain the earlier single-top-roller concept, or define a remote
shaft/bearing stack absent from the selected direct-drive architecture.
