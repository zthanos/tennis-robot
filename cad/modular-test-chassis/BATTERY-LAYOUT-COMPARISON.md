# Rear launcher: low centre battery versus side battery / L basket

2026-10-11. Architecture CAD only. No manufacturing export, force validation,
new xacro, or changes to the tested main chassis/intake/launcher parts.
Source `rear-low-launcher-study.scad`.

User battery: 198 × 170 × 166 mm laid down; 5.59 kg. Permitted battery orientation,
terminals, straps, wiring and service access still require confirmation.
The accepted launcher stays at 20° with 8 mm plates and metal spacers, yaw 180°,
nip (-70,0,175) mm. No previous FEA is approval of these new load paths.

## Selectors

- `battery_layout="center"`: battery X146..344, Y±85, centre Y0.
- `battery_layout="side_l"`: battery X176..374, Y0..170, centre Y85.
- `mount_mode="clear"` (default): preserve existing front brace and intake ramp.
- `mount_mode="target20"`: show the requested 20 mm strip-underfloor clearance;
  **intentionally unresolved** because it intersects the fixed front brace/ramp.

The fixed brace reaches Z42. Centre strips start Z43 (only 1 mm nominal gap),
battery bottom Z46/top Z212. Side battery also crosses the ramp sidewall, which
reaches Z53 in its footprint: strips start Z54, battery bottom Z57/top Z223.
These are candidate clearances, not tolerance, flex or full-load approval.

Two 20 mm wide / 3 mm thick suspended strip references span the inner chassis.
Their thickness only reserves space; it is **not** approved to carry 5.59 kg.
Attachment returns are illustrative; detailed support design is still required.

## Basket geometry

Centre mode: 5 mm gap over battery + approximately 3 mm floor. Nominal rear
inside floor surface Z220; front Z268, intake handoff surface Z35, hence front
entry rise 233 mm. Requested target20 would give rear Z197/front Z245 (210 mm
rise), but fails the existing-frame interference check.

Side mode: actual L footprint wraps behind and beside the battery bay. Floor
upper surface Z148 at X80, rising to Z168 at X374; front entry rise 133 mm.
It is a packaging hypothesis, not proof that the intake can climb that height.
Rear L leg is 86 mm externally / 80 mm internally; main side channel is 170 mm
externally / 164 mm internally. The right-angle concave corner remains a flow
and bridging risk: a rounded/diagonal flow guide and a multi-ball test are needed.
Neither layout is a solved gravity-only intake path.

L basket rim Z300 versus centre rim Z335; not equal measured capacity. Battery
must have independent retention, not basket walls. Side displacement contributes
0.47515 kg·m to the lateral mass first moment (5.59 × 0.085); whole-robot CG still
requires every other mass and empty/full basket cases. Ball load is not a stable
counterweight. Cover, LiDAR and electronics are not repositioned in this study.

## Selected geometric QA

Eight checks: battery vs filled basket volume, launcher vs battery/basket,
straight Ø76 exit vs selected frame/support solids, swept Ø66 feed balls vs
launcher/battery, funnel walls vs launcher, battery vs original chassis/intake
and launcher cradle. Empty exports exit 1 with “Current top level object is
empty”; do not accept other errors as passes.

With default clear mode, all eight selected pairs returned empty for each
layout (16 pair checks). The three structural/chassis test modules passed 23
tests. Rerun QA after changes. The first
target20 runs found real collisions; no existing brace or ramp was silently cut
away. Full assembly clashes, strips/preload/deflection, tool access, one-ball
escapement, transient trajectory and reliable ball flow remain unverified.

## Deferred xacro / simulation gate

After choosing CAD, create isolated layout xacro variants with consistent frames,
metre units, battery mass 5.59 kg, rotated geometry and inertia, metal plate and
launcher masses, basket empty/full load and realistic collision primitives.
Keep the existing simulation default unchanged. Compare wheel normal loads,
acceleration/braking, left/right turning and tipping margins under the same
commands/friction. Rigid-body Gazebo is for motion/load distribution, **not**
printed-strip strength, bolt pullout or elastic deflection; these need separate
structural screening and physical testing. No xacro was generated in this step.
