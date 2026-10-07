# Perfboard-end 3+2 connector — first fit CAD

For **each** of four BTS7960 logic harnesses:

- one 1×3 female header for `RPWM`, `LPWM`, `EN`, with a matching 1×3 male
  header on the perfboard;
- one 1×2 female header for logic `5V`, `GND`, with a matching 1×2 male
  header on the perfboard.

This CAD makes two small cable-end clamshells and two **different keyed board
shrouds**. The 2-pin plug has a centre key on one side. The 3-pin plug has
an offset key on the opposite side. The geometry is intended to reject a
2-pin plug in the 3-pin shroud and 180° rotation of either plug; verify these
cases with the first print before connecting power. This is
mechanical guidance **only while each shroud is fixed to the perfboard**. Bare
male pin strips are not polarised.

The two cable-shell halves are held by two narrow cable ties in the outside
grooves. The headers are real metal parts, not printed contacts. Directly
solder the small signal wires to the rear tails, insulate every joint and
provide a cable stop/strain relief in the rear chamber. This is a fit
prototype, not a sealed or vibration-qualified connector.

## Dimensional assumptions and first test

- Pin pitch and spacing: 2.54 mm.
- The *outside plastic body* of the actual 1×2 and 1×3 female headers has **not
  been measured**. Default body lengths are `contacts × 2.54 + 2.84 mm`, and
  body width/height are 2.7/8.5 mm. All are editable in
  `perfboard-end-v0.scad`.
- The board shroud has no bottom: it goes around the male header's plastic
  base, then is retained on the perfboard after a dry fit. The final mounting
  method must tolerate vibration; do not glue until orientation and clearance
  have been checked with the **actual board**.
- Print **one** 2-pin and **one** 3-pin set first, not all four. With power
  disconnected, check header-body fit, full insertion depth, cable strain
  relief, and that all wrong pairings and 180° rotations are rejected.
- After mounting the shrouds, verify with a continuity meter that 2-pin `5V`
  and `GND`, and 3-pin `RPWM`, `LPWM`, `EN`, are on the intended pins. Do not
  hot-plug while power is on.

Export examples:

```sh
openscad -o 2p-plug-lower-v0.stl -D 'contacts=2' -D 'part="plug_lower"' perfboard-end-v0.scad
openscad -o 3p-board-shroud-v0.stl -D 'contacts=3' -D 'part="board_shroud"' perfboard-end-v0.scad
```

These CAD parts do not replace electrical strain relief, insulation, an
emergency stop, or the separate high-current motor wiring.
