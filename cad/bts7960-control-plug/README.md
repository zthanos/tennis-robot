# BTS7960 logic plugs — current fit prototypes

Only the following generated files in this directory match the current
four-wire BTS control harness and the Rev C perfboard connector plan:

- **Driver side:** `plug-v3-jumpers.scad`, `plug-lower-v3.stl`,
  `plug-upper-v3.stl`, `plug-dummy-v3.stl`, `plug-fit-gauge-v3.stl`.
  See `JUMPER-V3-README.md`. This 2×4 end is visually marked but **not
  mechanically keyed** against 180° reversal; check the real driver labels.
- **Perfboard cable side:** `perfboard-male-end-v2-m3.scad`, the four
  `perfboard-{2p,4p}-plug-{lower,upper}-v2-m3.stl` halves, and
  `perfboard-4p-plug-v2-m3.png`. See `PERFBOARD-MALE-END-V2-M3.md`.
  These ends mate with `../motion-electronics-tray/unified-keyed-faceplate-v2`.

The active v3/v2 SCAD files include `plug-v2-jumpers.scad` and
`perfboard-male-end-v1.scad` respectively. Those **source dependencies stay
here**, even though their old v2/v1 STL and PNG exports are archived.

All superseded print exports, old 3+2 concepts, old cable-tie perfboard
shells and their supporting notes are in [`archive/`](archive/README.md).
Do not print an archived STL for the new perfboard. None of these prototype
connectors is yet vibration-qualified. With all power disconnected, verify
pin order, full metal-pin engagement, contact retention and absence of shorts
before any electrical test. No motor current runs through these parts.
