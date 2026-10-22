# Tennis Robot Documentation

Active source-of-truth documents, grouped by topic. When a decision changes,
update the relevant baseline document first, then adjust CAD/software/tests.

**Start here:** [`current-design-el.md`](current-design-el.md) is the concise
current architecture. [`decisions/decision-log-el.md`](decisions/decision-log-el.md)
records why earlier architectures changed. Physical print status is owned by
[`../cad/ACTIVE-PRINT-SET.md`](../cad/ACTIVE-PRINT-SET.md).

## Structure

```text
docs/
  collection-route/   continuous collect_route: spec, design, plan, debug log, Nav2 controller
  perception/         OAK-D perception + measurement-covariance calibration
    covariance/       Gazebo covariance calibration reports (c2 / c2-v2 / c2-v3)
  survey/             court knowledge model + LiDAR court survey (v2) + SLAM
  mechanism/          shared rotating flywheel, basket, ski and P2 evidence
  hardware/           active 4WD layout, BOM, wiring, purchase lists, sensor contract
  process/            architecture guide, delivery contract, validation, board, telemetry
  images/             diagrams referenced by the build plans
  archive/  research/  superseded notes and broad exploratory research (background only)
```

## collection-route/
- `collection-route-rules-el.md` - active specification for the continuous collection route.
- `collection-route-design-el.md` - technical design.
- `collection-route-implementation-plan-el.md` - phased implementation plan.
- `collection-route-optimization-plan-el.md` - constraint register and measured plan for the shortest safe full route.
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
- `shared-flywheel-intake-launcher-debug-log-el.md` - πλήρες P2 history και
  source evidence για το κοινό περιστρεφόμενο flywheel.
- `p273-bolted-ski-final-el.md`, `outboard-selector-rails-p274-el.md`,
  `p285-ski-bracket-el.md` - ενεργός βατήρας/rails/στήριξη.
- `rear-cage-p280-el.md` - μελλοντικό fully printed PETG concept, όχι release.
- Τα `standalone-*`, `compact-*`, `intake-concept-decision-el.md` και τα
  παλιά flywheel capability reports είναι ιστορικά evidence και φέρουν banner
  προς το `current-design-el.md`.
- `../archive/mechanism/intake/README.md` - superseded intake reports, CAD/data
  traceability, and the classification ledger.
- `basket-bin-redesign-spec-el.md`.

## hardware/
- Purchase/build: `prototype-purchase-list-el.md`, `hardware-bom-el.md`,
  `ordered-parts.md` και το active CAD manifest.
- `chassis-layout-4wd-dual-intake-el.md` και
  `modular-open-front-test-chassis-el.md` είναι ιστορικά predecessors του
  modular v3/P2-78 chassis.
- Wiring/setup: `unified-keyed-perfboard-wiring-el.md` (new Rev C netlist;
  physical solder-side routing still pending), `ros2-control-migration-el.md`,
  `ubuntu-dual-boot-handoff.md`. Older perfboard wiring is in `../archive/hardware/`.
- Contracts: `sensor-topic-contract-el.md`, `hardware-glossary.md`.
- `../arduino/collector/` - Arduino bring-up sketches (collector motor, encoder, IR beam).

## process/
- `architecture-implementation-guide-el.md` - source-of-truth implementation guide.
- `feature-delivery-contract-el.md` - spec/design/plan workflow for substantial features.
- `pi-deployment-plan-el.md` - Raspberry Pi deployment/Jazzy baseline.
- `validation-plan-el.md`, `project-board-plan-el.md`, `telemetry-architecture-el.md`.

## Archive

`docs/archive/` and `docs/research/` hold superseded plans and broad exploratory
research. Treat them as background only.
