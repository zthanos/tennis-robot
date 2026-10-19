# Height-frame screen — current bolted plate architecture

2026-10-11. This is a **new ideal-joint structural model**, not a rerun of an
unchanged low chassis advertised as validation of its upper frame. The CAD
study, low-chassis manufacturing STLs, and standalone launcher source are not
modified by the solver.

## Geometry and materials

Source: `cad/modular-test-chassis/archive/v2-chassis/height-infrastructure-study.scad`. Whole
launcher pitch 20°, nip (410,0,298) mm. Two 8 mm plate midplanes at local Z=±43,
plate bolts at local X=±115 / Y=±145, four metal spacers Ø12 / pilot Ø3.3.
The extra upper 50×20 motor bridge has been removed; it is **not** analysed.

CalculiX quadratic B32R beams represent current 50×30 side rails, 50×20
ramp-back crossbar, 20×20 upper posts/frames, lower inclined launcher rails,
transverse members, approximate outboard roots and plate bosses. S6 quadratic
shells represent the two plates. Gmsh embeds exact bolt-pattern load/attachment
coordinates before meshing; the program fails on disconnected structural graphs.
Beam/shell common nodes are ideally rigid according to the
[CalculiX manual](https://www.dhondt.de/ccx_2.20.pdf), sections 6.2 and 7.3.

The lower plate's accepted exit relief is approximated by overlapping sampled
disks instead of the CAD's short circle-hulls. Shaft holes and exit slot retained;
small motor/plate bolt holes are **ideal attachment points**, not local notch
meshes. Polymer sections omit W webs and local roofs. Boss shape, side roots,
socket depth, preload and sleeve slip are not exact-CAD models.

PLA/PETG/PETG-CF use the existing provisional E/ν/density profiles, not measured
filament allowables. Plates assume aluminium E=69 GPa, ν=.33, 2700 kg/m³;
spacers assume steel E=210 GPa, ν=.30, 7850 kg/m³. The user confirmed the study's
8 mm metal plate and metal spacer architecture. Actual plate alloy, measured stock
thickness, spacer stock, thread engagement and fasteners remain unconfirmed. Threaded joins
are perfectly bonded: no pullout, contact, bolt tightening or local crushing.

## Loads and restraints

Every case includes weight of the modelled structure and launcher: motor 0.49 kg
and wheel 0.90 kg each, except wheel sensitivity at 1.20 kg each. Hub mass is
unknown and **excluded**, not assumed physically zero. No intake, drive hardware,
LiDAR, cover skins, electronics, basket or battery mass is included. The report's
mass is the model's mass, **not a real whole-robot weighing or slicer estimate**.

1. `gravity`: normal gravity, four ideal vertical rail supports at drive stations.
2. `turn_reference`: gravity + prescribed lateral 0.5g on modelled masses, plus
   previous opposing front ±10 N and -1300 N·mm per mount. Not measured driving.
3. `motor_torque_reference`: gravity + opposite 0.62 N·m about each motor axis.
4. `motor_torque_peak_screen`: gravity + opposite 2.635 N·m, a provisional
   motor-based screen, not a tested continuous operating or launch load.
5. `carry_rear_2g`: 2g down, only rear rail corners clamped in all six DOF.
   This is an artificial rear-only lift/jig, **not every way of carrying a robot**.
6. `wheel_mass_1p2kg`: normal gravity with heavier wheel mass sensitivity.

Wheel-support cases constrain Z at four rail nodes; only minimum X/Y restraints
remove rigid-body motion. These are not tyre friction/contact/ground models.
Equivalent motor/wheel force and CG moment are distributed over the four flange
attachment points; wheel CG, motor CG, pitch and opposite motor torques included.
Units mm/N/MPa/tonne: density and kg conversions have explicit unit tests.

## Verification and reproduction

`low_chassis_rerun.json` is the separate 45-case old lower-frame comparison.
It retains its original ideal rear-clamped load and analytical beam check;
do not compare its displacement numerically to a new flange displacement with
different loads, observation points and wheel supports.

`report.json` contains the 54-case new frame matrix: six scenarios × three
polymer profiles × meshes 20/10/5 mm. For each case retain deck/log/DAT in ignored
`fea/work/height_frame`, CAD/source/material hashes, force-balance error, masses,
flange translations and integration-point stress diagnostics. Displacement
convergence gate is 2% between the finest meshes. Stress convergence or bearing
capacity is **not** established by that gate.

```bash
python3 engineering/chassis-strength-bench/run_height_frame_fea.py
python3 engineering/chassis-strength-bench/run_global_frame_fea.py \
  --front-width-study \
  --work engineering/chassis-strength-bench/fea/work/height_frame_baseline \
  --output engineering/chassis-strength-bench/results/height_frame/low_chassis_rerun.json
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest -q tests/test_height_frame_fea.py
```

`--reuse` permits re-parsing completed runs only when regenerated deck bytes are
identical and the retained solver log finishes without errors. No failed run is
accepted as a valid result. `--materials`, `--scenarios`, `--meshes` allow explicit
subsets; a single mesh cannot pass the convergence gate.

## Completed numerical screen

All 54 cases solved successfully. Finest-mesh flange displacement changed by
at most 0.981% between 10 and 5 mm meshes (2% gate); maximum force-balance error
over the matrix was 0.119%. These are numerical checks, not strength approval.
The six structural/connection test modules passed 35 tests, including the saved
matrix checks. CAD, runner and material-profile hashes match the saved report.

For the provisional PETG profile at 5 mm: gravity flange translation 1.848 mm;
lateral reference 2.946 mm; heavier wheels 2.009 mm. The two flange mean
translations differ by 0.341 mm in the lateral case; their rotations and actual
wheel nip alignment still need checking. Gravity upper/lower plate integration
stress peaks are 6.002/5.574 MPa; lateral 8.339/7.942 MPa. These omit bolt-hole
notches and do not establish a plate, bolt or printed-part safety factor.

Rear-only 2g carrying gives 47.475 mm flange translation in the linear PETG
model: a warning against that support assumption, not a trustworthy physical
displacement prediction. Keep carrying support at separated lower-frame points
and verify joints physically before adding basket/battery. No CAD or printable
STLs were changed by this analysis.

The PETG model includes about 10.13 kg total loaded mass, including 3.355 kg
for the two plates, 0.282 kg equivalent spacer mass and 2.780 kg lumped motors/
launcher wheels. It excludes the components listed above; it is not robot mass.

## Interpretation limit

This linear elastic screen can identify flexible load paths. It cannot approve
printing, high-speed operation, fatigue, rotating imbalance, resonance, thermal
creep, bolt strength, FDM layer separation or motor-shaft/bearing capacity.
Large rear-only carry deflection is outside a reliable small-deformation
prediction: use it to reject that lifting assumption and demand a better support
layout/nonlinear analysis, **not** as an exact predicted physical deflection.
Basket/battery added later require their own measured masses, CGs and mounts.
