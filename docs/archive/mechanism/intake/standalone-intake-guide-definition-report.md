# SUPERSEDED — standalone intake guide and direct-drive interface definition

> **Artifact status: `SUPERSEDED_WRONG_COMPLIANCE_ARCHITECTURE`.** This
> historical study assumed that each motor/wheel assembly translated on a
> compliant guide. That is not the intended physical machine. The current
> architecture fixes both FIT0186 motors and both wheel centres to the bridge;
> compliance comes from the tennis ball and the Trencher tyre/open-cell insert.
> Rails, moving pods, return springs, pod stops, and moving-motor cable loops in
> this document are not current design requirements and must not be promoted to
> manufacturing CAD or simulation evidence. See
> [`standalone-intake-fixed-motor-compliant-tyre.md`](../../../mechanism/standalone-intake-fixed-motor-compliant-tyre.md).

Date: 2026-08-26
Scope: standalone intake only
Decision: **historical analysis superseded; wrong physical compliance architecture**

Machine-readable definition: [`config/standalone_intake_guide_definition.json`](config/standalone_intake_guide_definition.json)
Analysis-only CAD: [`standalone-intake-guide-definition.scad`](../../../../cad/archive/intake/standalone-guide-definition/standalone-intake-guide-definition.scad)

## Executive decision

The retained intake is two independently compliant, mirrored, direct-drive
pods. Each FIT0186, purchased `14-00012630` adapter, Raid interface, and wheel
moves outward as one rigid pod over 0–8 mm. Both wheel-to-motor axes remain
`(0, side*sin(35 deg), cos(35 deg))`, so both motors are outboard. No remote
shaft, coupler, external wheel bearing, or printed torque hub is introduced.

A new standalone guide study compared three genuinely different families. A
pair of size-9-class miniature profile rails per pod is recommended. The rails
are 80 mm long, parallel to travel, with their centre-lines 70 mm apart. One
approximately 30 mm long carriage per rail gives a 70 x 30 mm anti-racking
support rectangle. Twin symmetric compression springs and twin symmetric stop
pads avoid applying an intentional yaw moment. This is a defined architecture
and is suitable for analysis CAD, but supplier rail ratings, the physical
spring, the fixed support, and the printed/machined interfaces still require a
structural screen and physical validation.

The direct-drive stack is **not physically defined**. The adapter bore depth,
internal stop, set-screw relationship, actual Raid hex pocket/seating, and
wheel retention are among the missing physical datums. Per the STOP rule, no
engagement, seating, retention, or authoritative manufacturing geometry is
inferred. Dynamic ball-capture validation remains prohibited.

## Protected scope and evidence rules

This study does not alter or evaluate the basket, launcher, complete bridge,
chassis, or compact-robot packaging. Candidate A is neither a source nor a
default: dimensions below were selected afresh for the isolated load path.

Every interface datum is classified as `MEASURED_FROM_HARDWARE`,
`MANUFACTURER_SPEC`, `PURCHASED_PART_SPEC`, or `MISSING`. Approximate ruler
measurements are retained as measured evidence but are not treated as exact
stack-release dimensions. Analysis allocations and derived loads are clearly
identified as provisional rather than masquerading as part measurements.

## Hardware metrology checklist

Measure both intake instances separately and record tool, resolution, date,
part marking, operator, three repeated readings, mean, range, and a photograph
showing the datum. Do not copy one motor, adapter, or Raid insert to the other.

### A. FIT0186 motor — checklist for each motor

| Datum | How to measure | Current value | Class | Release relevance |
|---|---|---:|---|---|
| exact model/label | photograph label and purchase identity | FIT0186 / GB37Y3530-12V-251R expected | MANUFACTURER_SPEC | confirm both variants match |
| shaft diameter | micrometer at 3 axial and 2 angular positions | nominal 6 mm; received part reported 6 mm | MEASURED_FROM_HARDWARE | exact readings missing |
| D-flat depth and remaining thickness | micrometer over round and flat; calculate only from those readings | — | MISSING | adapter compatibility |
| D-flat axial length and start from tip | depth gauge/caliper | — | MISSING | set-screw landing |
| shaft projection from motor face | depth gauge from cleaned face, excluding any pilot | approximately 20 mm reported | MEASURED_FROM_HARDWARE | exact value missing |
| shaft shoulder/axial stop | profile photograph and axial dimensions | — | MISSING | adapter seating/retention |
| motor-face datum/pilot | measure face flatness, pilot OD/height if present | — | MISSING | pod bracket datum |
| mounting pattern | verify six holes, PCD/adjacent pitch, clocking to cable | six M3; adjacent centres 15.5 mm | MANUFACTURER_SPEC | physical verification pending |
| usable thread depth | bottom gently with depth probe; subtract any chamfer | maximum 3 mm | MANUFACTURER_SPEC | physical verification pending |
| body diameter | micrometer/caliper at gearbox and motor can | approximately 30 mm | MEASURED_FROM_HARDWARE | record both exact envelopes |
| body length | face datum to rear-most rigid body | approximately 70 mm | MEASURED_FROM_HARDWARE | record both exact envelopes |
| encoder/cable exit envelope | photograph, connector size/pose, lead diameter and natural exit direction | — | MISSING | mount and cable loop |
| motor mass | scale including attached leads, state lead length | 205 g | MANUFACTURER_SPEC | measure each pod item |
| shaft radial/axial play | indicator at a stated lever arm; no hand estimate | — | MISSING | side-load baseline |

### B. `14-00012630` adapter — checklist for each allocated adapter

| Datum | How to measure | Current value | Class | Release relevance |
|---|---|---:|---|---|
| part identity | bag/receipt photograph and unique ID | `14-00012630` | PURCHASED_PART_SPEC | traceability |
| total length | caliper between axial end faces | nominal 30 mm | PURCHASED_PART_SPEC | exact hardware reading missing |
| 6 mm bore diameter and depth | pin gauges plus depth pin; inspect blind/through | nominal 6 mm bore | PURCHASED_PART_SPEC | depth is MISSING |
| bore type | photograph/light test: round, D, split clamp, or keyed | — | MISSING | D-shaft compatibility |
| clamp/set-screw geometry | thread, quantity, tip, axis, centre from end, protrusion, key access | — | MISSING | torque and service |
| D-shaft compatibility | blue/marker contact trial on the actual shaft without torque | — | MISSING | cannot infer from 6 mm label |
| internal stop/shoulder | depth/profile inspection | — | MISSING | axial seating |
| 12 mm hex across flats | micrometer across three opposed face pairs | nominal 12 mm AF | PURCHASED_PART_SPEC | exact reading missing |
| hex axial length | caliper/profile image | — | MISSING | wheel engagement |
| axial retention features | list shoulder, flange, thread, clip groove, or none | — | MISSING | wheel retention |
| concentricity/runout | rotate on verified shaft; indicator at hex and seating face | — | MISSING | target to be set after measurement |
| usable shaft engagement | derive only after bore stop and screw landing are measured | — | MISSING | stack release blocker |
| wheel-side seating datum | identify shoulder/end face and measure squareness/offset | — | MISSING | stack release blocker |
| mass and maximum OD | scale and caliper including screw protrusion | — | MISSING | pod mass and clearance |

### C. Pro-Line Raid wheel interface — checklist for each wheel

| Datum | How to measure | Current value | Class | Release relevance |
|---|---|---:|---|---|
| exact wheel/insert identity | photograph wheel, package, insert markings and installed orientation | PRO117010 expected | PURCHASED_PART_SPEC | actual delivered part missing |
| installed hex variant | verify with 12 mm gauge; record narrow/wide insert and orientation | supplied 12/14 mm options; 12 mm intended | MANUFACTURER_SPEC | installed selection MISSING |
| hex pocket AF and depth | pin/hex gauge and depth gauge at all six flats | nominal 12 mm | MANUFACTURER_SPEC | actual depth MISSING |
| wheel seating face | identify insert/wheel/adapter contact face and axial offset | — | MISSING | stack release blocker |
| axial retention interface | measure centre screw/nut/thread/counterbore and tool access | — | MISSING | stack release blocker |
| centre clearance | gauge through full wheel/insert stack | — | MISSING | adapter interference |
| internal adapter interference | section view or depth-probe map; trial at zero torque | — | MISSING | stack release blocker |
| removable insert attachment | verify six fasteners, thread and protrusion | Raid 6x30 interface with six M3 fasteners | MANUFACTURER_SPEC | physical verification pending |
| actual wheel mass | scale complete installed wheel and insert | — | MISSING | pod mass |
| actual overall width | caliper across tread without compressing it | nominal 73 mm | MANUFACTURER_SPEC | exact hardware reading missing |
| actual diameter | diameter tape or two-plane height measurement, no tread compression | nominal 124 mm | MANUFACTURER_SPEC | exact hardware reading missing |
| radial/face runout | rotate on verified datum; indicator on hub and tread separately | — | MISSING | balance and bearing load |

## Direct-drive stack STOP disposition

The intended order is retained:

```text
FIT0186 motor face
  -> native 6 mm D-shaft
  -> 14-00012630 adapter
  -> 12 mm male hex
  -> installed Raid 12 mm interface and wheel
```

No exact installed stack can yet be released:

- shaft engagement needs exact shaft projection, flat extent, adapter bore
  depth/stop, and set-screw landing;
- adapter seating needs its motor-side internal stop and wheel-side shoulder;
- axial retention needs the adapter retention method and the Raid centre
  fastener/nut/thread stack;
- wheel engagement needs installed hex pocket depth, insert orientation,
  seating face, centre clearance, and internal-interference map.

Adapter orientation, wheel seating, and removal order therefore remain
undetermined. A safe provisional service rule is only: isolate power, support
the wheel and pod, photograph/mark orientation, release the measured
wheel-retainer first, remove the wheel axially without prying on the motor
shaft, then release the measured adapter clamp/set screw. This sequence must be
replaced by a part-specific work instruction after metrology.

```text
INTAKE_SHAFT_ENGAGEMENT_DEFINED = false
INTAKE_ADAPTER_SEATING_DEFINED = false
INTAKE_AXIAL_RETENTION_DEFINED = false
INTAKE_WHEEL_HEX_INTERFACE_DEFINED = false
```

## New guide architecture study

All candidates use two mirrored but independent guides. Travel is lateral:
left `+Y`, right `-Y`. The motor, adapter, wheel, bracket, guide followers,
spring moving seats, stop tongue, and moving cable anchor translate together.

### Candidate 1 — dual round shafts with four dry plain bushings

- **Fixed parts:** two 8 mm stainless shafts, four end supports, local backing
  plate, spring and stop anchors.
- **Moving parts:** printed/machined pod plate with four replaceable flanged
  polymer bushings, motor bracket, twin spring seats and stop tongue.
- **Geometry:** 60 mm shaft length; 70 mm shaft centre spacing in X; two 24 mm
  long bushings per shaft, their centres 30 mm apart along Y; 8 mm travel.
- **Anti-racking:** four bearing zones form a 70 x 30 mm rectangle. The second
  shaft support uses transverse assembly slots, then clamps after alignment.
- **Moving mass estimate:** 0.54–0.78 kg, dominated by the unmeasured wheel.
- **Risk:** low debris sensitivity and no recirculating elements, but four
  plain bearings can bind if shaft parallelism or printed bore coaxiality is
  poor. Reamed/machined bearing carriers are preferred over printed bores.
- **Stiffness/service:** moderate-high; shafts and bushings are individually
  replaceable, but four end supports and eight support fasteners add bulk.
- **Spring/stops/cable:** twin springs and twin stop pads centred at X +/-25
  mm; rear U-loop above the motor.
- **Failure modes:** shaft misalignment, bushing creep/wear, shaft-end support
  rotation, debris scoring, and asymmetric bushing drag.

This is not Candidate A: it is a new standalone four-bearing layout with a
different support rectangle and no inherited bridge allocation. It is credible
but rejected because its manufactured coaxiality burden is higher than the
selected profile-rail datum.

### Candidate 2 — captured box/dovetail guide with adjustable gibs

- **Fixed parts:** 80 mm hard-anodised aluminium male box rail, end brackets,
  replaceable inner/outer stop pads.
- **Moving parts:** 45 mm long U-carriage with two UHMW-PE side gibs and one
  adjustable top gib, pod plate, spring seats and stop tongue.
- **Geometry:** 40 mm rail width, 18 mm captured depth, 45 mm bearing length,
  0.15–0.25 mm running clearance after adjustment, 8 mm travel.
- **Anti-racking:** broad 40 x 45 mm bearing footprint and opposing gibs
  constrain yaw/pitch/roll; two spaced gib screws set clearance.
- **Moving mass estimate:** 0.56–0.80 kg.
- **Risk:** best gross-debris shedding and inexpensive manufacture, but highest
  print/material sensitivity; thermal growth, gib creep, or one over-tightened
  screw can create stick-slip. A printed rail is not acceptable as the wear
  datum.
- **Stiffness/service:** moderate; wipe-clean open ends, replaceable gibs, 10–12
  guide fasteners depending on gib retention.
- **Spring/stops/cable:** same symmetric architecture as Candidate 1.
- **Failure modes:** gib loosening, creep, local wear steps at the 0 mm rest,
  debris wedging, and preload-dependent friction masquerading as spring force.

This candidate is viable for a low-cost prototype but is not selected because
repeatable low friction over only 8 mm is unusually sensitive to gib setup.

### Candidate 3 — dual size-9-class profile rails (selected)

- **Fixed parts:** two 80 mm profile rails on one machined or accurately
  shimmed local metal rail bed, eight M3 rail screws, twin spring anchors, and
  twin replaceable stop pads.
- **Moving parts:** one approximately 30 mm carriage per rail, 4 mm aluminium
  or adequately screened printed pod plate, motor saddle, spring seats, stop
  tongue, and moving cable anchor.
- **Geometry:** rails parallel to Y; rail centres X = +/-35 mm; carriage
  effective contact length approximately 30 mm; 8 mm travel. At rest the
  carriage centres are 32 mm from the inner rail ends and move to 40 mm at the
  outer stop, leaving at least 25 mm nominal rail coverage beyond each block
  end in the analysis envelope.
- **Anti-racking:** two preloaded/clearance-controlled carriages form a 70 x 30
  mm support rectangle. Rail separation carries yaw/roll couples; each
  carriage's longitudinal bearing rows carry pitch. The datum rail mounts in
  round holes; the follower rail is aligned through transverse slots to <=0.05
  mm parallelism over 80 mm and <=0.10 mm coplanarity before final torque.
- **Moving mass estimate:** 0.54–0.78 kg; use 0.66 kg nominal only for the
  first-pass load screen.
- **Friction/contamination:** lowest predictable friction but poorer loose-dirt
  tolerance than the plain-bearing concepts. Fit end seals, mount rails with
  openings downward/outward where possible, add non-contact shields, and keep
  both rail ends accessible for dry wiping. No exposed grease trap faces the
  ball path.
- **Print sensitivity/stiffness:** low if the rail bed is metal and carriage
  bosses are post-machined or shimmed; unacceptable if two raw printed faces
  alone establish coplanarity. Highest stiffness of the three, subject to the
  selected supplier's moment/load rating.
- **Fasteners/service:** approximately 8 rail screws, 8 block-to-plate screws,
  6 motor screws, 4 stop screws, and 4 spring-anchor fasteners per pod. A pod
  removes upward after unplugging at the fixed connector and releasing the
  block screws; rail alignment remains intact.
- **Failure modes:** rail brinelling from stop impact, contamination, loss of
  parallelism, block screw loosening, seal drag mismatch, corrosion, and
  supplier carriage moment rating below the pitch screen.

Candidate 3 is selected because it provides the most explicit six-degree
constraint, the best repeatability and stiffness, a serviceable replaceable
datum, and the least dependence on printed sliding fits. Exact rail brand and
rating remain a procurement/structural gate; “size 9” is an analysis envelope,
not permission to substitute any look-alike rail.

In the standalone analysis allocation, all left fixed-guide solids remain at
Y >= +100 mm and all right fixed-guide solids at Y <= -100 mm, leaving 200 mm
between the opposite fixed-guide half-spaces. Moving blocks remain still farther
outboard, and springs, stops, and cable envelopes never cross Y = 0 throughout
0–8 mm. The two tread envelopes intentionally face the ball corridor; no guide,
spring, stop, motor, adapter, or cable volume is assigned to the opposite pod's
guide half-space. This is an isolated packaging statement only and does not
reopen complete-robot integration.

## Spring and preload derivation

The centered geometric closure is `66 - 56 = 10 mm`. With symmetric pods:

```text
ball diametral deformation delta = 10 mm - 2*x_pod
```

The accepted calibrated loading law has `K = 107309.294 N/m^1.5` and preload
compression `d0 = 2.761219 mm`. For this guide sizing study, the incremental
force above the calibrated contact datum is:

```text
F_ball(delta) = K * ((d0 + delta)^1.5 - d0^1.5)
```

This preserves the calibrated nonlinear slope while making zero added nip
deformation equal zero incremental guide load. It is a quasi-static sizing
calculation, not a new ball fit and not a dynamic result.

Each pod uses two symmetric compression springs. The **combined per-pod**
provisional range is 0.75–1.25 N/mm with 3–5 N total rest preload. Therefore:

| Combined pod spring case | Rest | At 4 mm | At 8 mm | Equilibrium pod motion | Residual ball deformation | Passage force |
|---|---:|---:|---:|---:|---:|---:|
| soft | 3.0 N | 6.0 N | 9.0 N | 4.640 mm | 0.720 mm | 6.47 N |
| nominal | 4.0 N | 8.0 N | 12.0 N | 4.533 mm | 0.934 mm | 8.53 N |
| firm | 5.0 N | 10.0 N | 15.0 N | 4.432 mm | 1.136 mm | 10.54 N |

Nominal analysis CAD shows two 0.50 N/mm springs per pod (1.00 N/mm combined),
2 N preload each, 31 mm nominal free length, 27 mm installed rest length, and
19 mm installed length at the 8 mm stop. A selected spring must have <=17 mm
solid height, >=12 mm safe working compression from free length, suitable
fatigue life, and rate/preload tolerance that keeps the combined system inside
the range. These are selection requirements; the spring remains
`PROVISIONAL_HARDWARE_SELECTION_PENDING`.

The expected 6.5–10.5 N passage reaction and 15 N maximum spring reaction are
not high shaft stresses by themselves, but the Raid overhang is missing. Motor
bearing radial/axial ratings are also missing, so “not excessive” cannot yet be
proved. The physical stack must minimize wheel-centre overhang and the bench
must measure bearing temperature/play and shaft runout.

## Anti-racking definition and load path

The selected guide does not rely on a single short surface:

```text
travel direction Y
rail centres: X = -35, +35 mm       (70 mm transverse spacing)
carriage effective length: 30 mm    (longitudinal support)
support rectangle: 70 x 30 mm
spring axes: X = -25, +25 mm
stop contacts: X = -25, +25 mm
```

Yaw about Z is reacted by differential lateral load at the 70 mm rail spacing.
Roll about Y is reacted by the same transverse separation and carriage bearing
rows. Pitch about X is reacted by each carriage's approximately 30 mm
longitudinal contact length; supplier pitch-moment capacity must exceed the
screen below. Twin springs, twin stops, equal shims, and a centred motor saddle
avoid designed-in asymmetric loading. During assembly, move the un-sprung pod
through the full stroke while the follower-rail screws are snug, align, torque,
then verify breakaway force in both directions. A >20% left/right rail drag or
any position-dependent stick-slip fails assembly.

## Cable service definition

Motor power, encoder, and any sensor leads terminate in one sleeved moving
bundle; no bare lead crosses the joint. Until cable manufacturer data are
available, use a conservative **30 mm minimum dynamic bend radius** and do not
use cable smaller than its published moving-life radius.

- Moving strain relief: motor/pod rear, within 20 mm of the motor exit, with no
  load at the solder/connector joint.
- Fixed strain relief/connector: local fixed guide, at least 70 mm outward of
  the moving anchor's rest position and above the rail contamination plane.
- Loop: one planar U-loop, at least 60 mm inside diameter, with at least 25 mm
  additional developed slack beyond the anchor-to-anchor straight distance.
- Swept reservation: 80 mm along Y x 25 mm along X x 70 mm along Z per pod,
  outboard of the motor; the analysis CAD shows this translucent envelope.
- Separation: power and encoder conductors may share a rated flex sleeve but
  retain the motor/encoder manufacturer's shielding and twist requirements.
- Test: cycle the unpowered pod 0–8–0 mm 100 times. Cable-only force measured
  at the pod must stay below 0.3 N and must not change the 0 mm return datum by
  more than 0.1 mm.

The loop is anchored at both ends, cannot touch the wheel, spring, stop, rail,
or opposite pod, and is not credited toward preload or return force.

## Hard stops

**Inner stop (0.00 mm):** two fixed M4-mounted replaceable acetal pads at
X = +/-25 mm contact two machined/aluminium carriage faces. Shims establish the
nominal wheel-gap datum. Both pads must touch within 0.05 mm; the springs hold
the pod against them. Wear pads, not rail seals or carriage ends, own the rest
datum.

**Outer stop (8.00 mm):** two independent fixed stop screws with locknuts and
replaceable 70A polyurethane impact washers contact matching carriage faces at
X = +/-25 mm. Set both with gauge blocks to 8.00 +/-0.10 mm from the inner
datum. A secondary metal catch at 8.5 mm prevents block run-off if a polymer
washer fails. The spring never serves as the travel limiter.

Stops and springs are outside the opposite pod's swept half-space. A failed
spring returns no guaranteed preload, but the retained rails and metal outer
catch prevent wheel/pod release; the bench controller must then remain off.

## First-pass mass and load screen

The wheel, adapter, exact bracket, and rail blocks are unmeasured, so mass is a
range, not a released inertia: FIT0186 0.205 kg; wheel 0.18–0.32 kg provisional;
adapter/Raid hardware 0.02–0.06 kg; pod plate/saddle/blocks/fasteners
0.135–0.195 kg. Total moving mass is 0.54–0.78 kg, nominal 0.66 kg.

- Quasi-static passage normal load: 6.47–10.54 N per pod; full-stroke spring
  load: 9–15 N.
- Motor stall torque bound: 1.77 N m; wheel-radius tangential-force bound:
  28.55 N. This is a motor bound, not a demonstrated operating load.
- With a provisional 100 mm wheel-contact-to-rail-plane offset, 15 N creates
  1.5 N m pitch moment. Split across two blocks, a 30 mm effective bearing
  length corresponds to opposing row reactions of about 25 N per row (50 N
  summed magnitude) in each block. A conservative simultaneous 15 N normal and
  28.55 N tangential bound gives 32.25 N resultant guide shear. If the full
  tangential bound also acts at the 100 mm offset, the combined moment bound is
  4.36 N m; split between blocks it corresponds to about 72.6 N per opposing
  row (145 N summed magnitude) in each block. A purchased rail must exceed the
  applicable force and moment cases with an explicit safety factor; no
  anonymous rail is released.
- If the measured wheel-centre overhang from motor face is 25–55 mm, a 15 N
  side load gives 0.375–0.825 N m shaft bending moment. A simple 6 mm solid
  circular-section screen gives approximately 17.7–38.9 MPa bending stress.
  The shaft material, stress concentration, bearing span/rating, combined
  torque, fatigue, and actual overhang remain unknown, so this does not release
  the motor bearings or adapter.
- Adapter torque is bounded by the 1.77 N m motor stall torque plus the above
  side reaction. Clamp/set-screw capacity cannot be screened before bore,
  screw, engagement, and material metrology.
- A 0.66 kg pod moving at the current 1.0 m/s simulation joint limit would
  carry 0.33 J and is not an acceptable stop-design condition. The physical
  standalone controller must limit outward speed to 0.15 m/s for first tests;
  then energy is 0.0074 J. With a provisional 1 mm polyurethane stop stroke,
  the constant-deceleration average impact force is about 7.4 N, plus the
  spring force. Peak force and rail suitability require a drop/impact bench
  check and supplier dynamic rating.

This is a first-pass screen only. The guide requires supplier-rating review,
fastener/plate checks, and a physical breakaway, deflection, stop, contamination,
and fatigue test.

## Analysis CAD status

The CAD shows the selected standalone guide at 0, 4, and 8 mm, including both
mirrored pods, 35 degree axes, motor/adapter/wheel envelopes, two rails and
blocks per pod, twin springs, paired stops, and cable-loop reservation. Missing
purchased interfaces are translucent analysis envelopes. It deliberately
contains no bore, D-flat, screw landing, Raid seating, wheel retention, final
motor bracket holes, or released fits. It is not manufacturing CAD and does
not modify any complete-robot artifact.

## Superseded final classifications

The historical conclusions below are void for the current physical intake.
There is no guide to select, screen, validate, or promote. The current fixed
motor/compliant-tyre checkpoint owns all forward status.

```text
INTAKE_DIRECT_DRIVE_STACK_PHYSICALLY_DEFINED = false
INTAKE_SHAFT_ENGAGEMENT_DEFINED = false
INTAKE_ADAPTER_SEATING_DEFINED = false
INTAKE_AXIAL_RETENTION_DEFINED = false
INTAKE_WHEEL_HEX_INTERFACE_DEFINED = false

RECOMMENDED_INTAKE_GUIDE_ARCHITECTURE = NONE_SUPERSEDED_WRONG_COMPLIANCE_ARCHITECTURE
INTAKE_GUIDE_ARCHITECTURE_DEFINED = false
INTAKE_GUIDE_ARCHITECTURE_UNRESOLVED = false
INTAKE_GUIDE_ARCHITECTURE_READY_FOR_CAD = false
INTAKE_GUIDE_STRUCTURAL_SCREEN_REQUIRED = false
INTAKE_GUIDE_PHYSICAL_VALIDATION_PENDING = false

INTAKE_GUIDE_ANTI_RACKING_DEFINED = false
INTAKE_GUIDE_SPRING_PRELOAD_DEFINED = false
INTAKE_GUIDE_SPRING_HARDWARE_SELECTED = false
INTAKE_GUIDE_HARD_STOPS_DEFINED = false
INTAKE_GUIDE_CABLE_SERVICE_DEFINED = false

INTAKE_GUIDE_READY_FOR_AUTHORITATIVE_CAD = false
INTAKE_READY_FOR_DYNAMIC_STANDALONE_BENCH = false
DYNAMIC_BALL_CAPTURE_VALIDATION_AUTHORIZED = false
INTAKE_PREVIOUS_GUIDE_STUDY_SUPERSEDED = true
```

The exact direct-drive STOP datums are the motor shaft/flat/shoulder, adapter
bore/stop/clamp/hex/seating, and installed Raid pocket/seating/clearance/
retention measurements listed above. When those are recorded, select rated
rails and springs, update the load screen with actual mass and overhang, then
create authoritative standalone manufacturing CAD. Do not simulate the ball
again before those gates close.
