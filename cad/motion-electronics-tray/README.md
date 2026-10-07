# Motion electronics service tray

> **Legacy mechanical layout — do not print as the final electronics tray.**
> It still has one L298N intake bay and only two BTS7960 bays. The active
> electrical plan uses two BTS7960 for intake, with two additional BTS7960
> planned for drive. The tray CAD and generated STL/PNGs must be revised and
> rechecked against the physical four-driver layout before assembly.

Parametric, open-air carrier for the physical motion/intake prototype:

- Arduino Mega 2560 Rev3 in its measured 60 x 110 mm acrylic enclosure;
- 80 x 120 mm common motion/intake perfboard, installed portrait;
- one measured 53 x 60 mm L298N intake driver;
- two measured 50 x 50 mm HW-039 / BTS7960 motion drivers.

The tray is `180 x 240 mm`, so it prints as one part on the Bambu Lab P2S
`256 x 256 mm` build plate. It is deliberately open: the BTS7960 heatsinks,
motor terminals and power wiring must not be enclosed.

## Confirmed and provisional dimensions

Measured from the user's physical parts:

- P2S build volume: `256 x 256 x 256 mm`;
- Mega acrylic case: `60 x 110 mm`, using its two upper and two middle holes;
- perfboard: `80 x 120 mm`, measured `75 x 115 mm` hole pattern;
- L298N: `53 x 60 mm`, measured `46 x 53 mm` hole pattern;
- each BTS7960: `50 x 50 mm`, measured `40 x 40 mm` hole pattern;
- all measured PCB/enclosure holes: `4 mm` diameter.

The supplied measurements were made with a ruler, so every component uses
two-axis M3 cross-slots for fit tolerance. Print the fit-check before the full
tray.

The added L298N occupies the former relay bay. The physical E-stop relay, fuse
and high-current distribution must be installed on a separate power/safety
carrier; they are not optional electrical components.

## Files

- `params.scad` — board envelopes, positions, mounting patterns and tolerances.
- `tray.scad` — printable tray with standoffs, vents, slots, labels and cable
  separation ribs.
- `assembly.scad` — non-printing reference mock-up of all installed boards.
- `fit-check.scad` — 0.6 mm full-footprint mounting template; print this before
  the structural tray.
- `component-retaining-pins.scad` — separate grooved pins and 12 mm C-clips
  sized for the perfboard, Mega case, L298N and raised BTS7960 mounts.
- `bts-spacers.scad` — eight required 20 mm spacers plus two spares for the two
  BTS7960 boards.
- `vertical-service-stand.scad` — separate one-piece tabletop fixture that
  holds the existing 180 x 240 mm tray vertically for wiring and bench tests.
  It uses the tray's two lower M5 slots, supports the tray edge on ledges and
  provides cable-tie slots for a temporary screw/WAGO distribution block.
- `vertical-service-stand-clips.scad` — **rejected; do not print**. Although the
  clip envelope clears the lower PCB footprints, the populated continuous tray
  blocks the lateral installation path to the two stand uprights.
- `vertical-service-stand-edge-clip-fit-test.scad` — superseded before print by
  the unified retainer below; retained only as design history.
- `vertical-service-stand-unified-retainer-fit-test.scad` — one full-width U
  concept, **superseded before print** by the front-loaded retainer.
- `vertical-service-stand-front-retainer-fit-test.scad` — current fit-test:
  slides in from the front, has two cut-outs around the black support blocks,
  bolts through the stand's x=68/142 centre slots and bears continuously on the
  lower 10 mm of the green tray, below every driver PCB.

## Render and export

From the repository root:

```bash
docker compose --profile cad run --rm openscad \
  openscad -o cad/motion-electronics-tray/assembly.png \
  --imgsize=1200,1600 --viewall --autocenter \
  cad/motion-electronics-tray/assembly.scad

docker compose --profile cad run --rm openscad \
  openscad -o cad/motion-electronics-tray/motion-electronics-tray.stl \
  --export-format binstl cad/motion-electronics-tray/tray.scad

docker compose --profile cad run --rm openscad \
  openscad -o cad/motion-electronics-tray/motion-electronics-fit-check.stl \
  --export-format binstl cad/motion-electronics-tray/fit-check.scad

docker compose --profile cad run --rm openscad \
  openscad -o cad/motion-electronics-tray/component-retaining-pins.stl \
  --export-format binstl \
  cad/motion-electronics-tray/component-retaining-pins.scad

docker compose --profile cad run --rm openscad \
  openscad -o cad/motion-electronics-tray/vertical-service-stand.stl \
  --export-format binstl \
  cad/motion-electronics-tray/vertical-service-stand.scad

docker compose --profile cad run --rm openscad \
  openscad -o cad/motion-electronics-tray/vertical-service-stand-front-retainer-fit-test.stl \
  --export-format binstl \
  cad/motion-electronics-tray/vertical-service-stand-front-retainer-fit-test.scad

```

## Print and assembly

- Material: PETG/PETG-HF for the first court prototype; ASA is suitable after
  fit is confirmed. PLA is only for indoor fit checks.
- Suggested start: 0.4 mm nozzle, 0.20 mm layers, 4 walls, 5 top/bottom layers,
  25-35% gyroid infill. Print flat without supports.
- Use M3 fasteners and washers for all PCBs. Do not let a washer or printed boss
  contact solder joints or copper reinforcement.
- The printable pin/C-clip retainers are intended for indoor prototype fit
  checks. Insert the 2.8 mm pin from above, then slide the 12 mm PETG C-clip
  sideways into its 2.2 mm groove below the tray. The clip bridges the complete
  adjustment slot. Replace them with M3 nylon or metal fasteners before
  sustained vibration testing.
- Each BTS7960 board uses four separate 20 mm spacers above the tray's existing
  8 mm bosses. Its dedicated pins span the complete 28 mm support stack and
  leave only the required groove/clip projection below the tray; keep the pin
  heads at the four corner holes, clear of each central heatsink. A physical
  trial with an additional 10 mm projected about 13 mm and was rejected.
- Printed-fit correction: the first 3.2 mm pins would not enter the physical
  holes and the 2.2 mm revision was too thin for the wide slots. The retained
  design uses a 2.8 mm shaft plus a separate 12 mm C-clip below the tray.
- Use nylon or metal spacers if the supplied printed clearance is insufficient.
- Orient both BTS7960 power terminals toward the outer/service edge and keep
  B+/B-/M+/M- wiring in the lower power zone.
- Print the vertical service stand flat on its 210 x 130 mm base. Bolt the
  existing tray through its two lower M5 slots with washers and locknuts; the
  tray edge must rest on both ledges before tightening. Secure the supply lead
  through a rear strain-relief slot and mount a proper distribution block in
  the four front cable-tie slots. Crocodile clips are not load-bearing wiring.
- If the M5 bolts are inaccessible after printing, do not use the rejected
  side-entry clips. Dry-fit the populated tray first and design a front-loaded
  retainer around the physically accessible lower slots/stand recesses.
- The printed tray is a mechanical carrier, not an electrical insulator to be
  trusted by itself. Preserve fusing, strain relief, common ground and the
  physical E-stop relay chain described in
  `docs/hardware/motion-perfboard-wiring-el.md`.

## Fit-check gate before final print

1. Print `motion-electronics-fit-check.stl` at 0.20 mm layer height (three
   layers) before committing to the structural tray.
2. Confirm the perfboard and acrylic Mega-case holes without forcing either.
3. Confirm the L298N and both BTS7960 boards sit flat and every M3 screw can be
   centred in its cross-slot with a washer fully supported by the boss.
4. Confirm the Mega USB cable and all driver terminals remain accessible.
5. Keep the separate relay/fuse carrier adjacent to the lower power zone.
