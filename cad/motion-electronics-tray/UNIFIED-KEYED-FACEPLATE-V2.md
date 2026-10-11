# Raised faceplate v2 — fit-test print

This revision keeps the verified 120 × 80 mm outside, four M3 hole positions
(115 × 75 mm centres), and the connector grid from the first study. It is
still a mechanical prototype, **not a soldering netlist**.

## What changed

- The eight keyed 2+4 BTS7960 port collars now rise 12.3 mm from the cover
  underside, up from 7.5 mm. The mating cable-plug nose is nominally 18.3 mm,
  leaving 6.0 mm of possible insertion before its wider shoulder meets the
  collar. That arithmetic does **not** prove metal-pin engagement or retention.
- A separate one-piece 8.5 mm-high perimeter skirt joins the four corner
  M3 spacer pads. Its 1.6 mm outer wall is open inside for the perfboard
  components. Cover, skirt, and board share the same four M3 screws.
- The 8.5 mm height is the *assumed* distance from the component face of the
  perfboard to the top of the soldered female header. A real header could be
  different. Change `skirt_h` in `unified-keyed-faceplate-v2.scad` to its
  measured height before relying on the final assembly. The cover underside
  must not press on the headers or their solder joints.

## Print and dry-fit

Print `keyed-2plus4-port-fit-coupon-v2.stl` first to check the taller port with
the actual 2-pin and 4-pin cable plugs. Print the full
`unified-keyed-faceplate-v2.stl` and `unified-keyed-skirt-v2.stl` only if that
fit passes. Print both large parts flat with the open face up; the skirt is a
separate piece so the full cover needs no large underside support structure.

With all power disconnected, stack board → skirt → cover. Check that the
skirt's corner pads sit on bare/insulated board areas without crushing wires
or solder joints; the perimeter wall must also clear anything mounted close
to the board edge. Insert all intended plugs fully, check that pins engage and
continuity is correct, and confirm no plug can be forced into the wrong port
or backwards. Do not tighten M3 screws until the real header height is
confirmed. Screw length must cover board + skirt + cover + washers/nut; select
it from the actual hardware stack.

The loose **male Dupont jumper ends** are held outside the perfboard by the
two-piece `perfboard-male-end-v2-m3.scad` plug bodies (separate 2-pin and
4-pin lower/upper STLs). Two through-M3 screws and nuts clamp each plug's
rear lugs so its halves remain together even when unplugged. The front keyed
nose is unchanged from v1. The original cable-tie groove is omitted from v2.
Fit of the purchased jumper plastic, actual screw hardware and pin protrusion
still needs to be checked physically.

The proposed electrical pin allocation is now in
`docs/hardware/unified-keyed-perfboard-wiring-el.md`. The solder-side physical
routing remains unverified; do not use an archived perfboard netlist for this
new board.
