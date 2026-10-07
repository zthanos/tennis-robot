# New unified motion + intake perfboard — mechanical study

This is a **new layout study** for one 120 × 80 mm, 42 × 30-hole perfboard.
It is not a soldering plan or a replacement electrical netlist. Its purpose
is to test whether the four driver harnesses can use integrated keyed 2+4
ports in one faceplate, while keeping the Mega, six encoders, IR, USER, IMU
and 5 V selection accessible. The throw/launcher hardware is not included;
its motor controllers and logic interface have not been finalised.
The proposed connectors expose **107 electrical positions** in total; the
previous consolidated IDC layout had 103. This count does not imply that
there is enough solder-side routing space.

The cable plugs must be the **2-pin power** and **4-pin control** versions in
`cad/bts7960-control-plug/perfboard-male-end-v1.scad`. Female header strips
are soldered on the new perfboard under the eight raised faceplate openings.
The faceplate itself supplies the keyways, so no eight loose board collars
are needed. Both the cover and its mounting must remain fixed for the keys
to protect against swapped or reversed plugs.

## Proposed component-side positions

Grid convention: C1 at left, C42 at right, R1 at bottom, R30 at top.

- `M-L`: 2-pin power C10–C11/R8; 4-pin control C17–C20/R8.
- `M-R`: 2-pin power C27–C28/R8; 4-pin control C34–C37/R8.
- `I-L`: 2-pin power C10–C11/R18; 4-pin control C17–C20/R18.
- `I-R`: 2-pin power C27–C28/R18; 4-pin control C34–C37/R18.
- Six 1×4 encoder headers, all on R26, start at C7, C13, C19, C25, C31
  and C37; provisional labels `LF`, `LR`, `RF`, `RR`, `IL`, `IR`.
- Mega IDC 2×17: C3–C4/R6–R22 (same position as the earlier concept).
- IR IDC 2×5: C15–C19/R2–R3; USER IDC 2×3: C24–C26/R2–R3;
  IMU IDC 2×2: C31–C32/R2–R3.
- Power selector 1×3: C36–C38/R3; regulated 5 V input 1×2:
  C41–C42/R3.

For every power pair, in increasing component-side column order, the first
socket is **logic 5 V**, the second is **logic GND**. For every control group,
in increasing column order: `RPWM`, `LPWM`, `R_EN`, `L_EN`. The two EN sockets of each
driver must share that driver's EN net on the board; they must not be tied
directly to 5 V. None of these sockets carries motor current or 12 V.

The integrated 2-pin keyway is centred on its +Y side; the 4-pin keyway is
offset on its −Y side. The CAD matches the cable-plug prototype geometry.
The narrowest planar separation between adjacent driver-port outer walls is
about **5.25 mm**. This is a mechanical clearance calculation, not a
confirmation of wiring space or vibration resistance.
Across all nominal header envelopes and keyed ports in the study, the
smallest computed planar gap is about **3.98 mm** (control port to IR opening);
no envelopes overlap. Purchased header dimensions may change that result.

## Print and verification sequence

1. Print `keyed-2plus4-port-fit-coupon.stl` first. It has exactly the same
   2+4 spacing and keys as one pair in the full cover.
2. With everything unpowered, test both cable plugs together against real
   female headers held in the actual board grid. Check complete pin insertion,
   body clearance, no force on solder tails, key rejection of wrong/reversed
   plugs and cable bend/strain relief.
3. Only after the coupon passes, check the full faceplate against all planned
   components and mounting holes. `unified-keyed-faceplate-study.stl` is a
   mechanical prototype, **not yet approved for soldering or operation**.
4. Draw and audit a new point-to-point netlist for these exact grid positions
   and the four BTS7960/encoder groups. The old
   `unified-motion-intake-perfboard-wiring-el.md` assigns different header
   positions and must not be used to solder this new layout.
5. If the full netlist, solder-side wire crossings, service access or
   mounting fail despite the planar fit, split motion and intake into two
   boards. Do not move the throw circuit onto this board by assumption.

`unified-keyed-faceplate-study.scad` is parametric and retains the existing
120 × 80 mm board outline and 115 × 75 mm corner mounting pattern. The
generic female-header height in the visual mockup is 8.5 mm and needs a
physical check. The faceplate is 1.2 mm thick; each integrated collar reaches
7.5 mm above board level.
