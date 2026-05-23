# Plywood Cut List For First Chassis

Material:

```text
Birch marine plywood sheet: 1000 x 700 x 21 mm
```

This cut list matches the first CAD chassis reference:

```text
cad/3d-printable-base/base_mounting_plate.scad
```

For the full physical-store shopping checklist, including motors, wheels,
collector parts, fasteners, and safety electronics, see:

```text
docs/prototype-purchase-list-el.md
```

Ask the shop to keep the cuts square and to account for saw kerf, usually
about 3-5 mm per cut. Dimensions below are finished part dimensions.

## Required Cuts

| Qty | Finished size, mm | Part | Notes |
|---:|---|---|---|
| 1 | 760 x 430 | Main base plate | Bottom chassis plate. Mark front side before drilling. |
| 2 | 760 x 45 | Long side rails | Mount on edge along left/right sides for stiffness. |
| 3 | 390 x 45 | Cross rails | Mount on edge: rear motor area, middle battery area, front collector area. |
| 2 | 180 x 90 | Collector upright plates | Side/back support for collector lift wheel and hopper mouth. |
| 2 | 220 x 80 | Hopper/back-plate supports | Trim/drill after collector height is confirmed. |
| 2 | 120 x 80 | Motor pod reinforcement plates | Under or above the motor pod bolt area. |
| 2 | 100 x 80 | Front caster reinforcement plates | Under or above caster mounts. |
| 2 | 130 x 60 | Stabilizer bracket blocks | Front stabilizer / anti-tip mounting area. |
| 2 | 100 x 60 | Handle socket backing plates | Rear handle/socket reinforcement. |

## Optional Spare Cuts

Use remaining material for test brackets and shims:

| Qty | Finished size, mm | Use |
|---:|---|---|
| 4 | 150 x 45 | Short brackets / spacer rails |
| 4 | 90 x 45 | Corner blocks / test mounts |
| 6 | 60 x 40 | Shims, sensor brackets, temporary stops |

## Suggested Layout On 1000 x 700 Sheet

One practical nesting is:

```text
Top strip:
760 x 430 main base

Remaining side area / bottom strips:
2 x 760 x 45 long rails
3 x 390 x 45 cross rails
collector uprights, hopper supports, backing plates, stabilizer blocks
```

If the shop can optimize nesting, give them the required cuts and let them
arrange the sheet. Keep the grain direction along the long dimension for the
760 mm base and long rails if possible.

## Assembly Notes

1. Seal all cut edges before outdoor testing.
2. Pre-drill pilot holes before screwing into plywood edges.
3. Use washers under bolt heads on all module mounts.
4. Use threaded inserts, T-nuts, or through-bolts where parts will be removed
   often, especially collector and motor pods.
5. Do not glue the collector uprights at first. Bolt them so the lift-wheel
   gap and hopper height can still change.
6. If 21 mm feels heavy after the first rolling test, keep this base as the
   rugged prototype and later transfer the final geometry to 12 mm plywood plus
   aluminum angle.

## Assembly Instructions

### Adhesive And Fastener Strategy

Use glue only for wooden reinforcement parts that should become permanent.
Keep all mechanical modules removable.

| Joint | Recommended fastening | Why |
|---|---|---|
| Long side rails to base | D4 wood glue + 3.5x30 or 4x30 wood screws | Makes the plywood plate much stiffer. |
| Cross rails to base | D4 wood glue + 3.5x30 or 4x30 wood screws | Adds torsional stiffness and supports module loads. |
| Small backing plates | D4 wood glue + screws or clamps while curing | Spreads wheel/caster/collector loads. |
| Motor pods | M5 bolts + washers + T-nuts or threaded inserts | Must be removable and adjustable. |
| Front caster mounts | M5 bolts + washers + T-nuts or threaded inserts | Must be removable for height/level tuning. |
| Collector module | M5 bolts through slots + washers | Needs adjustment for lip height and wheel gap. |
| Electronics trays | M4 screws + standoffs | Serviceable wiring and board access. |
| Battery straps | Bolts/screws + strap slots | Battery must be removable for charging/service. |
| Handle sockets | M5 bolts + backing plates | High pull load; avoid relying on glue only. |
| Stabilizers | M5 bolts + washers | May need tuning after rolling tests. |

Wood glue:

```text
D3 wood glue: acceptable if the robot stays mostly dry.
D4 wood glue: preferred for outdoor/humidity resistance.
Polyurethane glue: strong and moisture-resistant, but expands while curing.
```

Small brad nails or pins can help hold rails while glue cures, but do not count
on nails as the main structural fastener. Screws are better because they clamp
the rail firmly into the base.

### Build Order

1. Mark the main base plate.
   - Mark `FRONT`, `REAR`, and the centerline.
   - Keep the 760 mm dimension front-to-back and 430 mm left-to-right.

2. Dry-fit the long side rails.
   - Place the two 760 x 45 mm rails on edge along the left and right sides.
   - Keep them inset enough that motor pods and caster brackets still sit flat.
   - Clamp them first, before glue.

3. Dry-fit the cross rails.
   - Use one cross rail near the rear motor pod area.
   - Use one cross rail around the battery/electronics area.
   - Use one cross rail near the front collector/caster area.
   - Check that none of the rails blocks planned bolt holes.

4. Glue and screw only the wooden rails.
   - Apply a thin continuous line of D4 wood glue.
   - Clamp the rail to the base.
   - Pre-drill pilot holes.
   - Add screws every 120-160 mm.
   - Wipe excess glue before it cures.

5. Add backing plates.
   - Glue/screw motor pod reinforcement plates around the wheel mount zones.
   - Glue/screw caster reinforcement plates around the front caster zones.
   - Add handle backing plates at the rear.
   - Let glue cure fully before mounting moving parts.

6. Drill module mounting holes.
   - Use `base_mounting_plate.scad` as the hole reference.
   - Start with smaller pilot holes.
   - Open module holes to M5 clearance only after checking alignment.
   - Use washers on both sides where possible.

7. Mount wheel system first.
   - Bolt motor pods loosely.
   - Bolt front caster mounts.
   - Place the base on a flat floor and check that all wheels touch correctly.
   - Tighten only after the base rolls straight.

8. Add battery and electronics.
   - Keep the battery low and near the center.
   - Use straps, not glue.
   - Leave the battery bay open to the left/right side so the battery can slide
     out after the top strap or clamp is removed.
   - Do not glue side blocks that trap the battery permanently.
   - Keep electronics serviceable and away from ball/wheel debris.

9. Mount the collector.
   - Bolt collector through slots with washers.
   - Start with the front lip around 5-12 mm above the floor.
   - Keep the lift-wheel/back-plate adjustment reachable.
   - Hand-feed balls before driving the robot.

10. Add handle and stabilizers.
    - Bolt handle sockets through backing plates.
    - Add stabilizers after wheel height is known.
    - Confirm the stabilizers do not scrape during normal collection.

11. Final sealing.
    - Sand sharp edges.
    - Seal exposed plywood edges.
    - Re-check all fasteners after the first vibration/rolling test.
