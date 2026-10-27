# Front crossbar width study — 2026-10-10

Diagnostic candidates only. Production SCAD and STLs are unchanged.

Reproduce with:

```sh
python3 engineering/chassis-strength-bench/run_global_frame_fea.py \
  --front-width-study \
  --work engineering/chassis-strength-bench/fea/work/front_width_study \
  --output engineering/chassis-strength-bench/results/global_frame/front_width_study.json
```

45 CalculiX B32R solutions: five layouts, three provisional isotropic materials,
three meshes (20/10/5 mm). Identical prescribed loads and rear clamps to the
original comparison. Maximum displacement convergence change is 0.382%; the
unbraced analytic PLA check differs by 0.086%.

Width is along X, height along Z. To retain the ramp-facing plane at X=319.4 mm,
the wider candidates move their centre from X=306.9 to X=294.4 mm. Thus this is
a practical width-plus-placement comparison, not a pure width-only experiment.
The bottom remains Z=22 mm; diagonal ribs remain 4 mm wide with a 32 mm reach.
No full CAD interference check has been performed for either new candidate.

At 5 mm mesh, maximum front displacement for PLA is:

- No crossbar: 0.408422 mm.
- Historical middle 50x30: 0.099028 mm.
- Current front 25x20: 0.195835 mm.
- Hypothetical front 50x20, rearward-expanded: 0.070308 mm.
- Hypothetical front 50x30, rearward-expanded: 0.055669 mm.

The 50x20 candidate reduces this metric by about 64.1% versus current front
25x20, and 29.0% versus historical middle 50x30 in this particular load case.
Increasing height to 30 mm reduces it a further 20.8%, but reaches Z=52 mm,
obstructing the current motor nut-access envelope (which starts at Z=44.3 mm).
The 20 mm height reaches Z=42 mm and retains nominal 2.3 mm vertical clearance.
Relative front X displacement does not improve: for PLA it increases from
0.011256 mm (25x20) to 0.012829 mm (50x20). Do not generalize the maximum
displacement improvement to every deformation component or load case.

This is an ideal-joint centreline-beam comparison, not a failure/load-rating
calculation. It excludes sleeve play, local holes, bolt preload, glue,
layer anisotropy, fatigue and the detailed W geometry. A 50x20 front member
is worth exploring next in CAD, including redesigned T nodes and sleeves,
wheel/ramp clearance and fastener access; it is not released for printing.
