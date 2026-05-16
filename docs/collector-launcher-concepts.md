# Collector And Launcher Concepts

Last checked: 2026-05-16

This document compares early mechanical concepts for collecting tennis balls and, later, launching them for practice. Costs are rough prototype estimates and exclude shipping, VAT/import duties, machining, 3D printing, and failed experiments.

## Concept Sketches

### Collector Intake Options

![Collector intake concept sketches](images/collector-concepts.png)

### Pneumatic Collection And Launch Concept

![Pneumatic collection and launch concept sketch](images/pneumatic-collection-launch-concept.png)

## Important Ball Constraint

An ITF tennis ball diameter is approximately 65.4-68.6 mm. For any tube-based concept, use generous clearance:

- minimum tube internal diameter: about 90 mm
- preferred tube internal diameter: 100 mm
- avoid tight bends
- keep the path short, smooth, and serviceable

Sources: ITF ball specification references and current retail pipe examples show common 100 mm PVC/ventilation tube is inexpensive and widely available. Example 100 mm flexible PVC ventilation tube: [Karwei](https://www.karwei.nl/assortiment/sanivesk-buis-flexibel-pvc-wit-100-mm-3-meter/p/B118002). Example tennis machine throwing wheel replacement: [Tennis Warehouse Australia](https://www.tenniswarehouse.com.au/spinfire-throwing-wheel.html).

## Concept Summary

| Concept | Collection | Launch | Prototype cost | Complexity | Best use | Verdict |
|---|---|---|---:|---|---|---|
| A. Funnel + lift wheel + flywheel barrel | Mechanical funnel guides ball into a lift wheel/roller | Hopper feeds a rear/top dual-flywheel launch barrel | US $330-$780 | Medium-high | First full architecture | Best direction, build collector first |
| B. Brush roller intake + flywheel barrel | Front brush/roller sweeps balls into a bin | Hopper feeds a rear/top dual-flywheel launch barrel | US $310-$750 | Medium | Simple pickup tests plus normal launcher | Good quick prototype |
| C. Scoop + conveyor + flywheel barrel | Low scoop feeds belt/elevator into hopper | Conveyor/hopper feeds a rear/top dual-flywheel launch barrel | US $370-$900 | High | Higher capacity collection | Strong later option |
| D. Pneumatic collection + pneumatic launch | Vacuum/airflow pulls balls through tube | Diverter sends balls to launch chamber | US $250-$700+ | High | Bench experiment, not first robot | Interesting but risky |
| E. Mechanical collector + flywheel launcher | Any collector above | Dual counter-rotating flywheels | US $250-$600+ launcher only | Medium-high | Reliable training shots | Best long-term launcher |

## Full Robot Architecture Note

The first collector sketches intentionally focused on pickup geometry and do not show the launch tube/barrel clearly. A complete tennis robot needs three mechanical zones:

1. front collector/intake
2. middle hopper/feed gate
3. launch module with a short barrel or guide tube

For mechanical collector concepts A-C, the launcher should be treated as a separate rear/top module. The ball path is:

```text
front intake -> hopper/bin -> feed gate -> dual flywheels -> short launch barrel
```

The launch barrel is not a long pressure tube. It is a short guarded guide after the flywheels that sets exit direction and keeps fingers away from moving parts.

## A. Funnel + Lift Wheel + Flywheel Barrel

Estimated prototype cost:

- collector only: US $80-$180
- full collector + launcher prototype: US $330-$780

Likely parts:

- 3D printed or sheet plastic funnel guides: US $10-$40
- rubber lift wheel or roller: US $10-$40
- small DC gear motor: US $15-$50
- simple motor driver: US $10-$30
- brackets, bearings, fasteners: US $20-$60

Launcher module:

- dual throwing wheels: US $100-$220
- two launch motors: US $60-$200
- motor drivers: US $40-$120
- feed gate and short launch barrel: US $40-$120
- guards, frame, fasteners: US $40-$140

Pros:

- forgiving if the robot approaches slightly off-center
- keeps the front of the base mechanically simple
- easy to simulate as collision guides plus one driven wheel
- good match for the current `bearing_rad` and `distance_m` approach logic

Cons:

- may struggle with balls against fences or court edges
- wheel pressure/gap needs tuning
- still needs a hopper/bin design

Recommended role: first mobile collection prototype.

Full robot role: best first complete architecture if we keep the launcher modular and build it after pickup works.

## B. Brush Roller Intake + Flywheel Barrel

Estimated prototype cost:

- collector only: US $60-$150
- full collector + launcher prototype: US $310-$750

Likely parts:

- soft brush roller or foam roller: US $15-$50
- DC gear motor: US $15-$50
- motor driver: US $10-$30
- front tray/bin and brackets: US $20-$50

Launcher module: same dual-flywheel barrel module as Concept A.

Pros:

- mechanically simple
- cheap to test
- works well if the robot can center itself in front of the ball
- easy to repair

Cons:

- can push balls away if height, speed, or roller softness is wrong
- less controlled than funnel/lift geometry
- may collect court debris

Recommended role: quick bench prototype if we want fastest mechanical test.

Full robot role: fastest full prototype, but pickup may be less reliable than funnel/lift.

## C. Low Scoop + Conveyor Belt + Flywheel Barrel

Estimated prototype cost:

- collector only: US $120-$300
- full collector + launcher prototype: US $370-$900

Likely parts:

- low scoop: US $10-$40
- small belt or printed conveyor: US $30-$100
- motor and pulleys: US $30-$80
- frame, bearings, tensioner, brackets: US $50-$120

Launcher module: same dual-flywheel barrel module as Concept A, potentially fed more cleanly by the conveyor/hopper.

Pros:

- naturally moves balls upward into storage
- scalable to multiple balls
- easier to integrate with a hopper

Cons:

- more moving parts
- belt alignment and debris handling can become annoying
- front geometry may require a larger chassis

Recommended role: second-generation collector if the first MVP works.

Full robot role: best capacity path, but too complex for first mechanical build.

## D. Pneumatic Collection + Pneumatic Launch

Estimated prototype cost: US $250-$700+

Likely parts:

- 90-100 mm smooth tube and bends: US $20-$80
- high-flow blower/vacuum: US $50-$200
- separator/hopper: US $30-$100
- diverter gate/servo/actuator: US $20-$80
- launch chamber or pressure path: US $80-$250+
- seals, access doors, filters, mounts: US $50-$150

Pros:

- same tube can theoretically serve transport and launch routing
- includes a visible launch tube/path from the start
- fewer ground-contact pickup mechanisms
- can move balls to a high hopper without a conveyor
- interesting system for experiments

Cons:

- collection wants high airflow and low pressure; launch wants high speed/energy and repeatability
- same path needs diverters, seals, and jam access
- noisy and power-hungry
- tennis ball fuzz increases friction and dust
- bends can stall or jam balls
- pneumatic launch is harder to tune than flywheels

Recommended role: bench experiment only. Do not make it the first mobile robot mechanism.

## E. Mechanical Collector + Dual Flywheel Launcher

Estimated prototype cost: US $250-$600+ for launcher module

Likely parts:

- two throwing wheels: US $100-$220 pair, depending source
- two motors: US $60-$200
- motor drivers: US $40-$120
- feed gate/wheel and short launch barrel: US $40-$120
- frame, guards, bearings, fasteners: US $50-$150

Pros:

- proven tennis ball machine architecture
- repeatable speed and spin
- easier to tune shot velocity than air launch
- launcher can be developed as a separate module after collection

Cons:

- rotating wheels need guarding and safety interlocks
- more electrical power draw
- spin/velocity calibration needed

Recommended role: best long-term launcher path.

## Current Recommendation

Build the project in this order:

1. Simulate and test `scan -> align -> approach -> stop_near_ball`.
2. Model Concept A: funnel + lift wheel.
3. Add the hopper/feed gate interface in the model, even before the launcher works.
4. Add a placeholder launch module with dual flywheels and a short barrel/tube.
5. Treat the pneumatic tube idea as a bench-only side experiment.

The pneumatic concept is worth sketching and possibly bench-testing, but it should not drive the first base choice. The base should first support a reliable front collector with enough room for a hopper and service access.
