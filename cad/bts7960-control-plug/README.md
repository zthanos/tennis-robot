# BTS7960 logic plug — fit prototypes

This is a **prototype for one of the four driver logic plugs**, not a finished
polarised connector. It uses one standard 2×4 female PCB header at 2.54 mm
pitch, with five signal wires soldered to its rear tails. The printed halves
retain the *plastic header body* and provide space for insulated solder joints.
There is no perfboard and no motor-current path through this part.

For the separate male-to-female jumper-wire approach, see
[`JUMPER-V2-README.md`](JUMPER-V2-README.md) and `plug-v2-jumpers.scad`.
That is a **new, unverified fit variant**; it does not replace the v1 files.
The subsequent photographed 3+3 gap prompted a wider **v3 fit test**; see
[`JUMPER-V3-README.md`](JUMPER-V3-README.md) and `plug-v3-jumpers.scad`.
For the opposite, perfboard-side ends of those same M–F jumpers, see the
2+4 male-end prototype in
[`PERFBOARD-MALE-END-V1-README.md`](PERFBOARD-MALE-END-V1-README.md).

`plug-v0.scad` and its three `*-v0.stl` files are the first print, with M2
fasteners. **`plug-v1.scad` is the current fit revision** after the user
confirmed that the gauge fits the driver: its complete driver-side nose is
1 mm shorter at each x end, and its two fastener holes suit M3 bolts with
*external* nuts. The approx. 13 mm header leaves only about 0.55 mm of printed
material at each x side of that nose, so this is a **fragile fit trial**, not
yet a vibration-ready part. Do not force the female header into the cavity.
The rear screw flanges stay 25 mm wide to preserve wall thickness around M3.
Their external x-z corners are rounded with a 2.5 mm radius; the header cavity,
pin grid, M3 hole positions and narrow driver-side nose are unchanged.
The y-positive (upper) half has a recessed `VCC` orientation mark. When wiring,
place the `VCC / R_IS / R_EN / RPWM` row on that marked side and align the mark
with the driver's own VCC silkscreen before plugging in.

## Electrical arrangement

| Driver contact | Harness |
|---|---|
| VCC | Logic 5 V |
| GND | Logic GND |
| RPWM | PWM line |
| LPWM | PWM line |
| R_EN + L_EN | Both bridged on insulated rear tails to one EN line |
| R_IS + L_IS | Physical socket positions present, electrically unconnected |

The bridge is between the two EN contacts only. Do **not** bridge either EN to
5 V. This CAD does not include the optional EN pull-down; whether the actual
modules need an additional driver-side resistor has not been established.
Insulate each rear tail and verify the completed harness with a continuity
meter before connecting the Mega or applying motor power.

## Print / fit sequence

1. Print the v1 `part="fit_gauge"` first. With *all power disconnected*, slide it
   gently over the IBT-2 2×4 male header. Do not force the pins. The gauge
   checks only pin spacing and aperture clearance.
2. The user's approximate overall size for the complete 2×4 header is
   **13 × 4.5 mm**. These values are the starting `header_x` and `header_y` in
   `plug-v1.scad`, with 0.4 mm total clearance. `header_z=8.5 mm` is still
   provisional; check the fit of the actual female header before use.
3. Print `part="lower"` and `part="upper"`. Their exported STLs are already
   oriented with the flat split face on the build plate. Fit the header,
   solder and insulate the five-wire harness,
   and close the halves with two M3 through-bolts and external nuts. Because
   the rear shell is 15 mm thick, M3×20 is a practical starting length; check
   the actual nut and washer stack before tightening. Use a sleeved
   cable bundle and a cable tie or other stop inside the rear chamber so
   pulling on the harness cannot load the solder joints.
4. Check physical clearance around the real driver board and verify every
   signal and absence of shorts with a meter. Do not attach the housing by
   the wires or apply force to the driver pins.

The front nose is narrow to reduce clashes with components near the header;
the screw flanges begin farther back. `cable_exit_d` assumes the five wires
are bundled in a sleeve. Adjust it to the actual bundle. PETG is preferable
for a final part near a warm driver; this first gauge may be printed in PLA.

**Not yet safe against reverse insertion:** the unshrouded 2×4 pin grid is
180° symmetric. Two unused IS positions, the VCC mark, and an asymmetric feature on
the cable half alone cannot key it. A matching driver-side guide must be
designed and fit-checked before this becomes a robot-ready connector.
