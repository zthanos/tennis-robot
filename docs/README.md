# Tennis Robot Documentation

Active source-of-truth documents, grouped by topic. When a decision changes,
update the relevant baseline document first, then adjust CAD/software/tests.

## Structure

```text
docs/
  collection-route/   continuous collect_route: spec, design, plan, debug log, Nav2 controller
  perception/         OAK-D perception + measurement-covariance calibration
    covariance/       Gazebo covariance calibration reports (c2 / c2-v2 / c2-v3)
  survey/             court knowledge model + LiDAR court survey (v2) + SLAM
  mechanism/          intake / funnel / basket / Concept-A build + intake debug log
  hardware/           BOM, wiring, cut lists, purchase lists, dual-boot, sensor contract
  process/            architecture guide, delivery contract, validation, board, telemetry
  images/             diagrams referenced by the build plans
  archive/  research/  superseded notes and broad exploratory research (background only)
```

## collection-route/
- `collection-route-rules-el.md` - active specification for the continuous collection route.
- `collection-route-design-el.md` - technical design.
- `collection-route-implementation-plan-el.md` - phased implementation plan.
- `collection-route-phase7-acceptance-plan-el.md` - Gazebo acceptance scenarios (S1-S8).
- `collection-route-debug-log-el.md` - **running debug log** (every collect_route change lands here).
- `collection-nav2-controller-{rules,design,implementation-plan}-el.md` - C++ CollectionFollowPath controller.
- `collector-controller-cheat-sheet.md`, `collector-wiring-reference-el.md` - collector bring-up.
- `mission-dashboard-plan-el.md` - mission-control dashboard evolution plan.

## perception/
- `perception-oakd-sim-el.md` - simulated OAK-D pipeline / `BallDetectionArray` contract.
- `oak-d-adapter-contract.md` - OAK-D adapter contract.
- `perception-measurement-covariance-calibration-{rules,design,implementation-plan}-el.md`.
- `covariance/` - Gazebo perception-covariance calibration reports and scenario.

## survey/
- `court-knowledge-model-specification.md` - environment knowledge model + definition of done.
- `court-survey-v2-spec-el.md` - **active, as-built** LiDAR court survey spec.
- `slam-mapping-el.md` - SLAM mapping notes.

## mechanism/
- `intake-debug-log-el.md` - **running debug log** for the intake/collector mechanism.
- `dual-wheel-intake-design-el.md`, `intake-concept-decision-el.md`, `intake-bench-sweep-report-el.md`.
- `basket-bin-redesign-spec-el.md`, `concept-a-funnel-lift-wheel-plan.md`.

## hardware/
- Purchase/build: `prototype-purchase-list-el.md`, `hardware-bom-el.md`, `ordered-parts.md`, `plywood-cut-list.md`.
- Wiring/setup: `motion-perfboard-wiring-el.md`, `ros2-control-migration-el.md`, `ubuntu-dual-boot-handoff.md`.
- Contracts: `sensor-topic-contract-el.md`, `hardware-glossary.md`.
- `../arduino/collector/` - Arduino bring-up sketches (collector motor, encoder, IR beam).

## process/
- `architecture-implementation-guide-el.md` - source-of-truth implementation guide.
- `feature-delivery-contract-el.md` - spec/design/plan workflow for substantial features.
- `pi-deployment-plan-el.md` - Raspberry Pi deployment + Humble→Jazzy migration plan.
- `validation-plan-el.md`, `project-board-plan-el.md`, `telemetry-architecture-el.md`.

## Archive

`docs/archive/` and `docs/research/` hold superseded plans and broad exploratory
research. Treat them as background only.
