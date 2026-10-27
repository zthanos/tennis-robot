# Dual outboard Γ brackets — prototype release notes

Date: 2026-09-07

This is an alternate physical support study. It does not change the frozen
intake-wheel, motor, ramp, basket, launcher or Xacro geometry. The removable
cheek fastening interface on this prototype bracket has a physical-fit
correction described below.

## Files

- Source: `dual-outboard-gamma-brackets.scad`
- Left print: `stl/gamma_bracket_left_prototype.stl`
- Right print: `stl/gamma_bracket_right_prototype.stl`
- Assembly check: `gamma-brackets-assembly-check.png`
- Two-piece cheek-base cable crossbar: `gamma-cheek-cable-crossbar.scad`,
  `stl/gamma_cheek_crossbar_left.stl` and
  `stl/gamma_cheek_crossbar_right.stl`;
- Removable cable cover: `stl/gamma_cheek_crossbar_cover.stl`;
- Corrected wheel-first compact ramp:
  `stl/intake_handoff_ramp_wheel_first.stl` (40.7 x 188 x 53 mm);
- Fractured-foot repair shoes: `stl/gamma_base_repair_shoe_left.stl` and
  `stl/gamma_base_repair_shoe_right.stl` (129 x 70 x 42 mm each);
- Reinforced complete replacements: `stl/gamma_bracket_left_reinforced_v2.stl`
  and `stl/gamma_bracket_right_reinforced_v2.stl` (185 x 176 x 149 mm each).
- Removable front-IR hangers: `stl/gamma_front_ir_hanger_left.stl` and
  `stl/gamma_front_ir_hanger_right.stl` (142 x 119 x 45 mm print envelopes).

The two STL files are mirrored parts and must be printed separately.

The optional crossbar is split into two printable parts and bolts on the upper
face of the two cheek-platform strips using both existing M5 holes on each
side. The existing cheek flange remains below the strips, so this does not move
the cheek datum or modify either Γ STL. Its installed end-to-end span
is 284 mm; the two centre pieces overlap by 36 mm and lock with one transverse
M3 bolt. The open-top duct has separate motor-power and encoder paths. The left
print is 34 x 142 x 14 mm and the right print, including its 36 mm tongue, is
34 x 178 x 14 mm. Both fit easily on the P2S bed and are oriented to avoid tall
generated supports. The new 4 mm mounting tab requires cheek bolts with about
4 mm of additional usable length. Both cable paths enter at the outer ends and
open into a 60 mm centre breakout bay (Y=-24..+36 mm); the M3 joint lock is
offset near its edge at Y=+34 mm so it does not occupy the main cable-exit area.
The internal divider also stops 15 mm before each outer entrance, providing an
undivided pocket for the approximately 13 mm-wide six-pin motor connector.

The removable cover is a single 24.6 x 238 x 7.4 mm print. It leaves 3 mm of
the duct open at each outer entrance and uses five independent snap segments
per side. Two rounded 6 x 48 mm openings at the centre align with the separated
motor-power and IR/signal lanes. The transverse M3 joint remains accessible
from the side. The cover is not part of the structural load path.

## Fractured-foot repair and reinforced v2

The first Γ prototype concentrated its load at a face-only wall/foot junction,
and its M5 holes sat at the edges of the open base rails. The temporary repair
shoe replaces that foot with a full 125 x 70 x 12 mm bearing plate. The old
lower header seats 4 mm into an 8.8 mm open-top channel at its original Z=60 mm
datum. Two M4 bolts at X=430/450 mm and Z=76 mm pass through the socket above
the old header (whose top is Z=72 mm), acting as cross-pins. Exact CSG against
the original printed left STL is empty, so the repair requires no new holes in
the surviving Γ. Each of the four M5 chassis holes retains its 5.5 mm bore in
the full plate and now has a 15 mm vertical tool/washer well through any socket
gusset above it; an inset 14.6 mm CSG access probe is empty at all four holes.
Use M4 x 30 bolts with washers and locknuts; clean the printed 4.5 mm passages
with a drill by hand if needed.

The complete reinforced-v2 Γ uses a full 125 x 70 x 10 mm foot, embeds the
outer wall through the complete foot thickness, adds an opposing load-return
diagonal, and broadens the rear root. Motor, wheel, cheek and chassis-hole
datums are unchanged. Exact CSG intersection with both motor/wheel envelopes
remains empty. The repair shoe is for supervised prototype testing, not a
strength certification or a substitute for inspecting the remaining print for
cracks and layer separation.

## Removable front-IR hangers

Each mirrored hanger drops onto a clear section of the already printed Γ upper
header with two inverted J-hooks. It needs neither holes in the Γ nor a clamp
fastener: gravity locates this lightly loaded sensor carrier, while the outer
hook lips prevent it moving sideways off the header. Its narrow spine passes
through the measured gap between the two motor-support ribs. A triangulated
drop places the provisional optical centre at X=460, Y=+/-110 and Z=40 mm,
immediately behind the cheeks and forward of the calculated wheel envelope.
The sensor shelf assumes the existing provisional 20 x 16 x 16 mm module
envelope and provides two retaining-tie slots plus an outer backstop. Three
small J-hooks route the cable upward and an upper slot accepts strain relief.
The print orientation lays the broad side frame on the bed, giving a 45 mm
maximum height. Exact CSG is empty against both original printed prototype Γ
STLs and against both wheel, motor and cheek envelopes. Confirm the real sensor
body and optical-centre position before treating the bracket as final hardware.

The current physical-test ramp comes directly from the compact wheel-first
geometry, with the -100 mm packaging shift baked into its coordinates. Its
envelope is X=319.65..360.35 mm, Y=-94..94 mm and Z=0..53 mm. A centred ball
contacts the wheels about 11.36 mm before reaching its 1.5 mm lip. Do not use
the 100 mm-long Option-A ramp for this assembly: its lip touches the ball about
48.6 mm before the wheels and can plough it forward. The corrected STL is
one connected manifold body and has zero exact intersection volume with either
wheel. It uses the clean unrelieved body; the earlier relieved compact export
left four disconnected wall fragments and is not a physical-print file. The
`stl/` directory contains only this ramp for the current Γ assembly. Historical
Option-A and ROS simulation meshes stay outside this print directory because
they belong to older reproducible studies. The corrected part contains no
physical mounting holes; its final attachment to the robot base must be
verified in the physical assembly.

## Retained interfaces

- wheel centres: compact ground `[370, +/-90, 70]` mm;
- common motor/wheel axis: 35 degrees in the X-Z plane;
- existing v6 motor-mount bearing plane: Z=190 mm;
- existing v6 motor-mount holes: four 4.5 mm holes, 34 x 28 mm pattern;
- existing compact cheek bearing plane: Z=150 mm;
- prototype cheek holes: X=475/495 mm, Y=+/-132 mm (10 mm forward from the
  earlier X=465/485 positions);
- chassis top: Z=52 mm;
- complete chassis foot and every base hole: X <= 460 mm.

The new base-hole pattern is provisional: four 5.5 mm clearance holes at
X=345/440 mm and Y=+/-188/232 mm. Verify it on the real chassis before
drilling.

## Geometry checks

- each STL is manifold (`OpenSCAD Status: NoError`);
- print envelope per side: 185 x 176 x 146 mm;
- exact CSG intersection with both wheel and motor envelopes: empty;
- central opening between the two motor pads: 138 mm, versus a 66 mm ball;
- the two cheek supports remain outside the existing ball corridor.
- each cheek strip extends 22 mm inward past its M5 hole centre (10 mm longer
  than the previous prototype), providing clearance for the removable cheek
  base beside the frame support;

## P2S PLA estimate

Measured with Bambu Studio 2.8.2, Bambu Lab P2S 0.4 mm nozzle, Bambu PLA Basic,
0.20 mm Standard, two walls and 20% sparse infill:

- one side, without generated support: 40.591 m filament;
- one side, calculated filament volume: 97.633 cm3;
- one side, estimated PLA mass at 1.24 g/cm3: 121.1 g;
- one side, estimated print time: 3 h 35 min 32 s;
- both sides, before support: about 242 g and 7 h 11 min;
- conservative allowance with tree supports: about 260-290 g and 8-9 h for
  the pair.

The CLI reports a floating-cantilever warning for the horizontal bearing pads.
Enable tree(auto) support at 30 degrees, as in the existing prototype profile,
and inspect the preview before printing. Do not print both parts on one plate;
use one part per plate.

## Preliminary structural screen

This is a sizing screen, not FEA or strength certification.

Per side, a deliberately conservative hand calculation uses:

- 1.0 kg supported motor/adapter/wheel mass at a 115 mm lateral arm;
- up to 107 N ball-contact envelope from the frozen compliance study;
- 1.77 N.m FIT0186 stall-torque reaction;
- combined nominal moment rounded upward and multiplied by a 3x dynamic factor;
- design screen: approximately 15 N.m per side.

At 44 mm lateral base-bolt row spacing, 15 N.m corresponds to roughly 341 N
total bolt-row couple. With four M5 fasteners, steel-fastener strength is not
the governing issue. The important physical checks are PLA layer adhesion,
washer bearing/creep, chassis stiffness and vibration loosening. Use wide
washers and locknuts, do not use printed threads, and perform the first loaded
test restrained and at low motor command.

The two 14 x 10 mm motor diagonals provide a nominal axial area of 280 mm2 per
side. Even when the complete 15 N.m screen is conservatively converted through
a 100 mm lever, nominal rib stress remains below 1 MPa. This does not capture
local stress concentration at printed joints; the diagonals and 8 mm junctions
must be inspected after every early test.

## Comparison with the original bridge

The repository baseline bridge is 18 mm plywood and intentionally has no STL.
If the alternative is a full bridge printed in PLA, these two brackets should
use substantially less filament: after removing the recorded 47 g aluminium
doublers, the Xacro's bridge mass corresponds to about 1.91 litres of plywood
at 600 kg/m3. A split PLA print of that geometry would likely remain several
times heavier than the
242 g support-free estimate here. This comparison does not make PLA equivalent
to plywood in stiffness, creep or impact resistance.
