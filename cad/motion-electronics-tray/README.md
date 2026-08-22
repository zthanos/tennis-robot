# Motion electronics service tray

Parametric, open-air carrier for the first physical motion prototype:

- Arduino Mega 2560 Rev3, with USB and barrel jack facing the service edge;
- 80 x 120 mm motion perfboard, installed landscape;
- two 50 x 50 mm IBT-2 / BTS7960 drivers;
- reserved universal bays for the future 12V/40A motor-power relay and
  fuse/distribution hardware.

The tray is `240 x 180 mm`, so it prints as one part on the Bambu Lab P2S
`256 x 256 mm` build plate. It is deliberately open: the BTS7960 heatsinks,
motor terminals and power wiring must not be enclosed.

## Confirmed and provisional dimensions

Confirmed:

- P2S build volume: `256 x 256 x 256 mm`;
- Mega Rev3 PCB: `101.6 x 53.3 mm`, official mounting pattern;
- perfboard PCB: `80 x 120 mm`;
- perfboard hole pattern: `73.66 x 111.76 mm` portrait, derived from the
  photographed 2.54 mm grid and supplied measurements;
- BTS7960 envelope: `50 x 50 x 43 mm`.

The BTS7960 mounting-hole pattern is **not confirmed**. Each corner therefore
uses a 7 mm two-axis cross-slot around a nominal 3.25 mm PCB inset. This accepts
approximately `40.5-46.5 mm` spacing. Measure the real drivers before printing;
if their holes fall outside that range, edit `driver_nominal_inset` and
`driver_adjust_span` in `params.scad`.

The relay and fuse/distribution bays are intentionally universal slots. Replace
them with fixed hole patterns only after those exact parts arrive.

## Files

- `params.scad` — board envelopes, positions, mounting patterns and tolerances.
- `tray.scad` — printable tray with standoffs, vents, slots, labels and cable
  separation ribs.
- `assembly.scad` — non-printing reference mock-up of all installed boards and
  future hardware envelopes.
- `fit-check.scad` — 0.6 mm full-footprint mounting template; print this before
  the structural tray.

## Render and export

From the repository root:

```bash
docker compose --profile cad run --rm openscad \
  openscad -o cad/motion-electronics-tray/assembly.png \
  --imgsize=1600,1000 --viewall --autocenter \
  cad/motion-electronics-tray/assembly.scad

docker compose --profile cad run --rm openscad \
  openscad -o cad/motion-electronics-tray/motion-electronics-tray.stl \
  --export-format binstl cad/motion-electronics-tray/tray.scad

docker compose --profile cad run --rm openscad \
  openscad -o cad/motion-electronics-tray/motion-electronics-fit-check.stl \
  --export-format binstl cad/motion-electronics-tray/fit-check.scad
```

## Print and assembly

- Material: PETG/PETG-HF for the first court prototype; ASA is suitable after
  fit is confirmed. PLA is only for indoor fit checks.
- Suggested start: 0.4 mm nozzle, 0.20 mm layers, 4 walls, 5 top/bottom layers,
  25-35% gyroid infill. Print flat without supports.
- Use M3 fasteners and washers for all PCBs. Do not let a washer or printed boss
  contact solder joints or copper reinforcement.
- Use nylon or metal spacers if the supplied printed clearance is insufficient.
- Orient both BTS7960 power terminals toward the outer/service edge and keep
  B+/B-/M+/M- wiring in the lower power zone.
- The printed tray is a mechanical carrier, not an electrical insulator to be
  trusted by itself. Preserve fusing, strain relief, common ground and the
  physical E-stop relay chain described in
  `docs/hardware/motion-perfboard-wiring-el.md`.

## Fit-check gate before final print

1. Print `motion-electronics-fit-check.stl` at 0.20 mm layer height (three
   layers) before committing to the structural tray.
2. Confirm all four perfboard and Mega holes without forcing the PCBs.
3. Measure both BTS7960 hole patterns and verify an M3 screw can sit centered in
   every cross-slot with a washer fully supported by the boss.
4. Confirm the Mega USB cable clears the tray edge and chassis surroundings.
5. Dry-place the future relay and fuse holders before replacing universal slots
   with final holes.
