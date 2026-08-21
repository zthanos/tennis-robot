# Collector intake v1 CAD

## Active design

The agreed collector is **Option A**. Its source, parameters, previews, export
script and generated STLs are kept together in [`option-a/`](option-a/).

Use only:

```text
cad/collector-intake-v1/option-a/
```

for the curved-cheek, plywood-bridge, 124 x 73 mm RC-wheel intake. See
[`option-a/README.md`](option-a/README.md) for the STL inventory, quantities and
the interfaces that remain provisional until physical measurement.

## Motor fit gauge

The root-level `motor-fit-gauge.scad/.stl` is a measurement coupon for the
approximately 70 mm long motor, approximately 30 mm body and measured 5 mm
D-shaft. It is not a torque-transmitting part.

## Pre-Option-A files — do not mix

The following root-level files and `stl/` exports belong to the earlier
straight-cheek/aluminium-rail study:

- `intake-structure.scad`;
- `export-structure-stls.sh`;
- `stl/cheek_front.stl`, `cheek_rear.stl`, `cheek_joiner.stl`;
- `stl/ramp.stl`, `rail_saddle.stl`, `rail_cap.stl`.

They remain only as development history and are **not parts of Option A**.
Running their export script does not update the Option A print set.
