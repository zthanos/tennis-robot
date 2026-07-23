# Tennis Robot Build Documents

This folder keeps the active source-of-truth documents for the first physical
prototype.

## Active Baseline

Use these files for purchasing, cutting, CAD alignment, and software assumptions:

- `feature-delivery-contract-el.md` - mandatory specification, design, and implementation-plan workflow for substantial new features.
- `prototype-purchase-list-el.md` - current purchase list for the first build.
- `plywood-cut-list.md` - plywood cut list and assembly instructions.
- `architecture-implementation-guide-el.md` - source-of-truth implementation guide; treats current code as legacy reference for the next Webots rebuild.
- `collection-route-rules-el.md` - active specification for the planned continuous collection route.
- `collection-route-design-el.md` - active technical design for implementing the continuous collection route.
- `collection-route-implementation-plan-el.md` - active phased implementation plan for the continuous collection route rewrite.
- `concept-a-funnel-lift-wheel-plan.md` - active Concept A build plan.
- `mission-dashboard-plan-el.md` - active mission-control dashboard evolution plan.
- `collector-wiring-reference-el.md` - active bench wiring reference for the first Arduino Nano + TB6612FNG collector prototype.
- `intake-concept-decision-el.md` - current intake concept decision: stop tuning the fragile single top-roller scoop path and move the next concept work to dual-wheel / dual-roller intake.
- `motion-perfboard-wiring-el.md` - active perfboard wiring reference for the Arduino Mega + 2x BTS7960 4WD motion prototype.
- `court-knowledge-model-specification.md` - active environment knowledge model specification and definition of done.
- `court-survey-v2-spec-el.md` - **active, as-built** court survey spec (LiDAR occupancy -> Court Knowledge Model). Replaces the old perimeter / Nav2-explore / FSM-fix survey docs.
- `project-board-plan-el.md` - proposed GitHub Project board workstreams, labels, and initial cards.
- `validation-plan-el.md` - baseline validation and implementation plan from simulation to real court tests.
- `ubuntu-dual-boot-handoff.md` - current simulation/collector checkpoint and
  first-run checklist for moving development from WSL 2 to native Ubuntu.
- `images/` - diagrams referenced by the active build plan.
- `../arduino/collector/` - Arduino Nano bring-up sketches for collector motor, encoder, and IR break beam tests.

## Archive

Older exploratory notes, broad hardware research, generated mechanical design
documents, and superseded collection/search plans live in:

```text
docs/research/
docs/archive/
```

Treat archived documents as background only. If a decision changes, update the
active baseline documents first, then adjust CAD/software to match.
