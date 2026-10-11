# Global frame comparison — ideal joints

`run_global_frame_fea.py` creates and solves three-dimensional CalculiX B32R
beam networks. BOX sections represent hollow rails and RECT sections the ribs.
This is a centreline engineering comparison, not an exact-CAD solid mesh and
not a certificate for printed parts. CalculiX B32R/BOX conventions follow the
[official manual](https://www.dhondt.de/ccx_2.20.pdf), beam section / B32R chapters.

## Common loading and supports

Units: mm, N, MPa. Rail centrelines Y=±220, Z=37; rear corner X=-165;
front drive mounts X=300. Both rear corners clamped in all six degrees of
freedom, representing an ideal test jig, not tyre support during driving.

Opposing longitudinal forces of 10 N at the front mounts plus -1300 N·mm
about Z at EACH mount represent the 130 mm wheel-to-rail offset. These are
prescribed reference loads, not the measured Gazebo joint-wrench norms.

Compare no transverse reinforcement, middle 50×30 with 6×30 ribs, and
ramp-back 25×20 with 4×20 ribs. Walls 3.2 mm; ramp-back vertical offset
Z=32 included. Rib ends are idealized at beam centrelines, not actual faces.
All joints fully bonded/rigid; no nut-pocket, sleeve slip, preload or adhesive
stiffness. Main rails remain uniform 50×30: W, local motor roofs, Gamma plates,
electronic carrier and their local stiffening are omitted in every variant.

## Evidence

27 solutions: three variants, three provisional materials and three mesh
lengths (20, 10, 5 mm). Maximum displacement change between 10 and 5 mm is
0.382%. Unreinforced PLA beam agrees with the independent moment-loaded
cantilever formula within 0.086%. These validate this simplified beam model,
not its approximation of the physical printed chassis.

At 5 mm mesh, PLA maximum front displacement is 0.408 / 0.099 / 0.196 mm
for none / middle / ramp-back. For this test the thin front brace reduces
maximum displacement approximately 52%; the middle brace approximately 76%.
The thin front brace has roughly twice the displacement of the larger middle
brace. Displacement redistribution matters: relative front X displacement is
0.00885 / 0.01214 / 0.01126 mm, so NOT every displacement component decreases.
Do not describe the result as a universal improvement under every turn/load.

Integration-point stress maxima are retained as diagnostics only. No local
hole/fillet/layer safety factor is asserted. Displacement convergence is
checked; stress convergence and nonlinear connection capacity are not.

Run:

```bash
python3 engineering/chassis-strength-bench/run_global_frame_fea.py
```

Source/result: `results/global_frame/comparison.json`; generated solver decks
and logs in ignored `fea/work/global_frame`. Next: calibrated joint compliance,
load/vertical-support sweeps and local solid-mesh studies of the small T sleeve.

## Separate Gazebo motion model

The default SDF now has one `chassis_body` and four wheel links/joints. Geometry,
aggregate mass, centre of mass and parallel-axis inertia of the former structural
links are preserved. There are no internal force sensors or structural loop.
`--legacy-structural-links` retains the old diagnostic representation only.

Motion tests at 0 and 10 kg cargo executed approximately in-place turns. This
establishes usable wheel motion/wrenches, not elasticity or internal joint loads.
The bench's command publishing uses wall-clock timing; do not interpret total
yaw as a calibrated 4-second angular-response measurement. A native shutdown
allocator warning occurred in both runs after capture. Cleanup is now
bounded; it does not alter the captured motion/force measurements. Runtime
shutdown stability remains a separate caveat.
