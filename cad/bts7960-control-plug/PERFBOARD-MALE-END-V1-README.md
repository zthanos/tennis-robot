# Perfboard-side jumper holders — first 2+4 fit prototype

This variant is for the **male ends** of the six M–F jumper wires already
held at their female ends by the driver-side v3 shell. It replaces the older
`perfboard-end-v0.scad` concept, which held *female* ends and used 3+2.

For each driver, the six male tips are grouped into two separate keyed plugs:

- `contacts=2`: logic `5V`, `GND`; two female sockets soldered to perfboard.
- `contacts=4`: `RPWM`, `LPWM`, `R_EN`, `L_EN`; four female sockets soldered
  to perfboard. The two EN sockets connect to the **same EN net on the board**,
  not directly to 5 V.

In CAD coordinates, contact 1 is at the negative-x end and has a small
recessed dot on the upper (`+y`) half. Do not infer electrical order solely
from a loose plug's viewed face: verify each jumper end against the labelled
perfboard sockets with a continuity meter before applying power. Wire
colours alone are not a pinout.

Each printed cable plug has a lower and upper half and a shallow external
groove for a small cable tie. The cable path is open at the split so the male
jumper ends can be laid in place without cutting or re-crimping them. The
plastic male-end housings sit behind a 1.2 mm front lip, leaving their metal
tips exposed. **Actual pin exposure and engagement depth must be checked**
with the real female header before this is used electrically.

The matching 2- and 4-position printed board collars have different keyways:
the power key is centred on one side; the control key is offset on the
opposite side. A bare 2-position plug can still enter part of a bare
4-position female header. The mechanical key works **only after the correct
collar is securely fixed around each perfboard header**. This first collar
has no built-in mounting feature because the perfboard layout and nearby
components have not been measured. Do not rely on loose collars for polarity
protection or vibration resistance.

## Dimensions still to confirm

The generic Dupont housing reference used here gives 2.54 mm pitch, 2.54 mm
body width and 14 mm length, but the exact Grobotronics jumper ends were not
dimensioned in their accessible product information. The CAD therefore uses
2.54 × 2.54 × 14 mm for each male-end plastic housing, with 0.50 mm total
clearance around a packed row. All values are editable in
`perfboard-male-end-v1.scad`.

Print **one 2-pin and one 4-pin cable plug plus their collars first**, not
four sets. With all power disconnected, check:

1. The 1P male plastic ends lie in their assigned positions, do not slide
   backward, and remain aligned when the halves close with a cable tie.
2. The exposed metal tips enter the soldered female headers fully, without
   hitting the printed front lip or flexing the perfboard.
3. The correctly oriented plug enters only its own collar, cannot be
   reversed, and the 2-pin plug cannot enter the 4-pin collar.
4. The board collars can be fixed in their correct positions without touching
   adjacent solder joints, traces or components.
5. Continuity, EN commoning, 5 V/GND polarity, and isolation between power
   and signals are correct.

This is logic wiring only, not a motor-current connector. Prototype Dupont
contacts and printed cable holders are not a substitute for a proven locking
connector in a vibration-heavy installation.
