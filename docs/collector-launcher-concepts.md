# Collector And Launcher Concepts

Last checked: 2026-05-16

This document compares early mechanical concepts for collecting tennis balls and, later, launching them for practice. Costs are rough prototype estimates and exclude shipping, VAT/import duties, machining, 3D printing, and failed experiments.

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
| A. Funnel + lift wheel | Mechanical funnel guides ball into a lift wheel/roller | Separate launcher later | US $80-$180 | Medium | First mobile collector MVP | Best first collector concept |
| B. Brush roller intake | Front brush/roller sweeps balls into a bin | Separate launcher later | US $60-$150 | Low-medium | Simple pickup tests | Good quick prototype |
| C. Scoop + conveyor | Low scoop feeds belt/elevator into hopper | Separate launcher later | US $120-$300 | Medium-high | Higher capacity collection | Strong later option |
| D. Pneumatic collection + pneumatic launch | Vacuum/airflow pulls balls through tube | Diverter sends balls to launch chamber | US $250-$700+ | High | Bench experiment, not first robot | Interesting but risky |
| E. Mechanical collector + flywheel launcher | Any collector above | Dual counter-rotating flywheels | US $250-$600+ launcher only | Medium-high | Reliable training shots | Best long-term launcher |

## A. Funnel + Lift Wheel

Estimated prototype cost: US $80-$180

Likely parts:

- 3D printed or sheet plastic funnel guides: US $10-$40
- rubber lift wheel or roller: US $10-$40
- small DC gear motor: US $15-$50
- simple motor driver: US $10-$30
- brackets, bearings, fasteners: US $20-$60

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

## B. Brush Roller Intake

Estimated prototype cost: US $60-$150

Likely parts:

- soft brush roller or foam roller: US $15-$50
- DC gear motor: US $15-$50
- motor driver: US $10-$30
- front tray/bin and brackets: US $20-$50

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

## C. Low Scoop + Conveyor Belt

Estimated prototype cost: US $120-$300

Likely parts:

- low scoop: US $10-$40
- small belt or printed conveyor: US $30-$100
- motor and pulleys: US $30-$80
- frame, bearings, tensioner, brackets: US $50-$120

Pros:

- naturally moves balls upward into storage
- scalable to multiple balls
- easier to integrate with a hopper

Cons:

- more moving parts
- belt alignment and debris handling can become annoying
- front geometry may require a larger chassis

Recommended role: second-generation collector if the first MVP works.

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
- feed gate/wheel: US $20-$80
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
3. Keep the launcher separate from the collector for now.
4. Use dual flywheels for launch when we reach the serving phase.
5. Treat the pneumatic tube idea as a bench-only side experiment.

The pneumatic concept is worth sketching and possibly bench-testing, but it should not drive the first base choice. The base should first support a reliable front collector with enough room for a hopper and service access.
