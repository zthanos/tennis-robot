# Tennis-ball compliant-contact calibration report

Date: 2026-08-25
Gate scope: independent ball calibration only; no launcher RPM, nip, trajectory, or performance trials were run.

## Executive result

The axisymmetric compliant-contact implementation passes the independent ITF deformation, rigid-surface rebound, timestep-convergence, energy, finite-cylinder, and bilateral-symmetry gates. Normal-contact launcher trials are therefore authorized. Launcher tyre friction is not independently known, is disabled rather than guessed, and remains a mandatory limitation on any later friction- or spin-dependent conclusion.

This is a reduced-order model fitted to published acceptance bands, not a material characterization of a particular tennis ball. Its loading exponent, ball-specific inertia, tyre compliance, and behavior above the ITF drop-test impact speed remain assumptions or unresolved measurements.

## A. Exact implemented contact model

For physical diametral compression `delta > 0`, loading uses

`F_elastic = K_load * delta^n_load`.

Unloading from the contact-history maximum `delta_max` uses

`F_elastic = K_load * delta_max^n_load * (delta / delta_max)^n_unload`.

The rate term is

`F_damping = C * delta^n_load * delta_dot`,

and the applied normal force is

`Fn = clamp(F_elastic + F_damping, 0, 5000 N)`.

State consists of maximum compression, unloading state, and explicit separation. Compression at or below zero produces zero force. An unloading contact separates when the clamped force reaches zero and cannot generate tension. Compression above 35 mm raises a guard error rather than silently extrapolating.

The standalone solver uses explicit RK4. The Gazebo Harmonic system applies the same law in `PreUpdate` through DART world wrenches. Sphere-to-closed-finite-cylinder geometry selects side, cap, or circular edge by closest point. It exposes normal force, tangential relative velocity and force, friction limit, ball and wheel torques, contact point and normal, compression, and compression rate on `/tennis_ball/compliant_contacts`.

Tyre friction is absent from the model SDF. Consequently tangential force and torque are exactly zero rather than based on an invented coefficient. The regularized Coulomb interface is implemented for later use after independent friction measurement.

To avoid double response, the ball's native collision mask excludes ground and wheel categories. When wheel links become available, the analytical plugin removes only their native collision entities; cradle and unrelated collisions remain native. This is necessary because the supported URDF-to-SDF path emits wheel bitmask extension tags at invalid link scope.

## B. Calibration methodology

The automated runner reproduces the relevant ITF protocol independently of launcher geometry:

1. Three orthogonal axes receive three 25.4 mm load/unload conditioning cycles each. Because the model is axisymmetric and has no hours-scale conditioning memory, these nine curves are identical simulations, not specimen measurements.
2. A 15.57 N contact preload establishes the deformation datum.
3. The platen advances at 3.333 mm/s to 95.64 N, holds 5 s, and records forward deformation.
4. The prescribed path reaches the 25.4 mm precompression travel, unloads to 95.64 N, holds 10 s, and records return deformation before complete unloading.
5. A separate rigid-plane case drops the ball from 2.54 m, fits dynamic damping to the 1.41 m midpoint of the ITF rebound band at a 25 microsecond reference step, then validates without refitting at 1.00, 0.50, and 0.25 ms.

Launcher geometry and outcomes are not inputs to any fit.

## C. Parameters and provenance

External/physical inputs:

- ITF Type 2 nominal diameter: 66 mm; implemented radius: 33 mm.
- Simulation mass: 58 g.
- Gravity: 9.80665 m/s^2.
- Platen loads, speed, holds, deformation bands, and rebound band: [ITF 2026 Technical Booklet](https://www.itftennis.com/media/15648/2026-technical-booklet.pdf).
- Nonlinear dynamic hysteresis motivation: R. Cross, *Dynamic properties of tennis balls*, Sports Engineering 2 (1999) 23–33, DOI 10.1046/j.1460-2687.1999.00019.x.

Calibrated/derived quantities:

- Preload compression: 0.002761218979 m.
- Loading stiffness: 107309.294042 N/m^1.5.
- Loading exponent: 1.5, a provisional Hertz-like model-form assumption not identifiable from one ITF loading point.
- Unloading exponent: 1.986635371, derived from the return-deformation midpoint and prescribed maximum travel.
- Dynamic damping: 4692.375891 N·s/m^1.5, fitted only to the 1.41 m rigid-surface rebound target.

Numerical parameters:

- RK4 fit step: 0.025 ms.
- Accepted validation steps: 1.00, 0.50, 0.25 ms.
- Maximum supported compression: 35 mm.
- Normal-force safety cap: 5000 N; it is not active in accepted calibration results.

## D. Deformation results

All three accepted timesteps produce 6.500 mm forward deformation and 9.400 mm return deformation at 95.64 N. These are the midpoints of the ITF 5.6–7.4 mm and 8.0–10.8 mm bands. Maximum total compression in the prescribed platen path is 28.1612 mm and peak elastic force is 507.124 N. Residual deformation is zero because permanent set is not represented.

## E. Hysteresis results

At 0.25 ms, the platen path stores 5.712497 J, recovers 4.781716 J, dissipates 0.930781 J through loading/unloading hysteresis, and dissipates 0.001665 J through the slow rate term. The unloading branch remains below the loading branch and never generates tensile force.

## F. Rebound results

At 0.25 ms the independent RK4 result is:

- Impact velocity: 7.058171 m/s.
- Contact duration: 7.000 ms.
- Maximum compression: 15.1349 mm.
- Peak normal force: 206.743 N.
- Separation compression: 0.6498 mm.
- Rebound velocity: 5.259941 m/s.
- Ball-bottom rebound height: 1.409974 m, within the 1.35–1.47 m ITF band.

The separately executed Gazebo/DART bench produced a ball-bottom first-rebound apex of 1.410931 m. Its difference from the 0.25 ms standalone result is 0.000958 m, or 0.068%, verifying implementation parity without fitting to launcher behavior.

## G. Timestep convergence

| dt (ms) | max compression (mm) | peak force (N) | duration (ms) | rebound (m) | dissipated (J) |
|---:|---:|---:|---:|---:|---:|
| 1.00 | 15.0207 | 210.963 | 7.000 | 1.411776 | 0.639229 |
| 0.50 | 15.0287 | 208.312 | 7.000 | 1.415173 | 0.639626 |
| 0.25 | 15.1349 | 206.743 | 7.000 | 1.409974 | 0.642713 |

Declared limits and observed relative spreads are: deformation 1.0% / 0.758%; peak force 2.5% / 2.022%; duration 1.0% / effectively zero; rebound 0.5% / 0.368%; dissipated energy 1.0% / 0.544%. Every criterion passes. The peak-force criterion is looser because the 1 ms case resolves the 7 ms contact with only seven steps; it was declared in the runner before gate evaluation and does not affect the ITF pass/fail outcome.

## H. Energy accounting

For the 0.25 ms rebound, incident translational/gravitational energy is 1.444716 J, rebound energy is 0.801973 J, hysteresis loss is 0.197091 J, viscous loss is 0.445539 J, and residual internal energy at force separation is 0.0000836 J. The balance residual is 0.00002985 J (0.00207% of incident energy). Across all steps, the worst residual is 0.1722%, below the declared 0.5% limit. No accepted case creates net mechanical energy. Rotational energy is zero because initial spin and calibrated tangential traction are zero.

## I. Ball inertia

The selected isotropic thin-shell approximation is `I = 2/3 m r^2 = 4.2108e-5 kg m^2`. The solid-sphere alternative, `2/5 m r^2 = 2.52648e-5 kg m^2`, is retained only as a sensitivity value. Inertia was not fitted to launcher trajectory. Since the present normal-only calibration has no spin, this choice does not alter the platen or rebound gate; later spin work must include the sensitivity case or replace it with measured inertia.

## J. Finite-cylinder validation

Python and C++ tests exercise side, end-cap, and circular-edge closest-point regions, unit outward normals, compression, zero force without penetration, the 35 mm guard, and post-separation no-tension behavior. The 200 mm diameter and 50 mm width analytical cylinders match the accepted flywheel collision geometry. No launcher performance case was run.

## K. Bilateral symmetry

At the accepted wheel centers `y = +/-0.129 m`, a centered 66 mm ball in the 58 mm nip produces 4 mm compression per side. The two contact normals are opposite and both force magnitudes agree within 1e-12 N in the unit test. Tangential-interface tests verify equal/opposite forces and reaction torques; fixed-step calibration runs are byte-for-value deterministic at the result-object level.

## L. Remaining unknowns

- Launcher tyre static and dynamic friction.
- Launcher tyre normal compliance; the current wheel interface treats the cylinder surface as rigid and applies the calibrated ball law.
- Measured inertia of the selected ball construction.
- Ball-specific, multi-point, three-axis force/deformation curves and permanent set.
- Rate dependence above the 7.058 m/s ITF drop-test impact velocity.
- Variation with pressure ball age, temperature, humidity, and wear.

No later report may treat friction- or spin-dependent output as calibrated until independent tyre evidence closes the friction gate.

## M. Tests executed

- `python3 scripts/run_tennis_ball_calibration.py`: PASS; all required classifications true, tyre friction pending.
- `pytest -q tests/test_tennis_ball_contact_model.py tests/test_flywheel_launcher_design_gate.py`: PASS, 18 tests.
- CMake release build of `tennis_ball_contact_system`: PASS.
- `ctest --test-dir /tmp/tennis_ball_contact_system_build --output-on-failure`: PASS, 1/1 C++ contact-model test.
- `gz sdf -k gazebo/models/tennis_ball_compliant/model.sdf`: PASS.
- Isolated Gazebo/DART 2.54 m rebound runtime: PASS, 1.410931 m ball-bottom apex.

## N. Files changed

Primary implementation and evidence files are:

- `scripts/tennis_ball_contact_model.py`
- `scripts/run_tennis_ball_calibration.py`
- `ros2_ws/src/tennis_ball_contact_system/`
- `gazebo/models/tennis_ball_compliant/`
- `gazebo/worlds/tennis_ball_rebound_calibration.sdf`
- `config/tennis_ball_compliance_design.json`
- `config/tennis_ball_compliance_calibration_results.json`
- `docs/mechanism/tennis-ball-compliance-calibration/`
- `tests/test_tennis_ball_contact_model.py`
- `tests/test_flywheel_launcher_design_gate.py`
- `scripts/evaluate_flywheel_launcher_design_gate.py`

The accepted launcher geometry dimensions, pitch, local coordinates, wheel joint locations and axes, isolated architecture, and DART selection were not changed.

## O. Final classifications

- `BALL_COMPLIANCE_MODEL_IMPLEMENTED = true`
- `BALL_DEFORMATION_CALIBRATED_TO_ITF = true`
- `BALL_REBOUND_CALIBRATED_TO_ITF = true`
- `BALL_LOADING_UNLOADING_HYSTERESIS_VALIDATED = true`
- `BALL_INERTIA_MODEL_DOCUMENTED = true`
- `TIME_STEP_CONVERGENCE_VALIDATED = true`
- `ENERGY_ACCOUNTING_VALIDATED = true`
- `FINITE_CYLINDER_CONTACT_VALIDATED = true`
- `BILATERAL_CONTACT_SYMMETRY_VALIDATED = true`
- `LAUNCHER_TYRE_FRICTION_CALIBRATION_PENDING = true`
- `LAUNCHER_PHYSICS_TRIALS_AUTHORIZED = true` (normal-contact gate only; no launcher trial was performed in this task)

Machine-readable values and gate logic are authoritative in `config/tennis_ball_compliance_calibration_results.json`.
