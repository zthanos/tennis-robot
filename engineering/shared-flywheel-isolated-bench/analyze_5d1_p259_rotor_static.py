#!/usr/bin/env python3
"""Static rotating-assembly screen for P2-59; no ROS or Gazebo."""
from __future__ import annotations

import json
import math
from pathlib import Path


RHO_TPU = 1200.0
RADIUS_M = 0.100
TPU_E_MPA = (10.0, 25.0, 50.0)  # existing Phase-1 low/nominal/high screen
KT = 0.031
KV_RPM_V = 270.0
R_OHM = 0.039
CURRENT_A = 20.0
SPIN_INERTIA = 0.00508981582897
ROTOR_INERTIA = 0.00009259233848
PROVISIONAL_HUB_MASS = 0.07820188778436878
PROVISIONAL_HUB_INERTIA = 2.134168618579316e-05


rows = []
for speed in (12.0, 16.0, 20.0, 24.0):
    omega = speed / RADIUS_M
    rpm = omega * 60.0 / (2.0*math.pi)
    hoop_pa = RHO_TPU * speed * speed
    expansions = {f"E_{value:g}_MPa": 1000.0*RADIUS_M*hoop_pa/(value*1e6) for value in TPU_E_MPA}
    back_emf = rpm / KV_RPM_V
    required_at_20a = back_emf + CURRENT_A*R_OHM
    rows.append({
        "surface_speed_m_s": speed, "omega_rad_s": omega, "rpm": rpm,
        "tpu_free_ring_hoop_stress_mpa": hoop_pa/1e6,
        "tpu_free_ring_radial_expansion_mm": expansions,
        "d5065_back_emf_v": back_emf, "voltage_required_at_20a_v": required_at_20a,
        "voltage_margin_at_15v_v": 15.0-required_at_20a,
        "voltage_margin_at_21v_v": 21.0-required_at_20a,
        "speed_fraction_of_no_load_at_15v": rpm/(KV_RPM_V*15.0),
        "speed_fraction_of_no_load_at_21v": rpm/(KV_RPM_V*21.0),
        "ideal_spinup_time_at_20a_s": (SPIN_INERTIA+ROTOR_INERTIA)*omega/(KT*CURRENT_A),
        "rim_centripetal_acceleration_m_s2": omega*omega*RADIUS_M,
        "imbalance_force_n_per_gram_at_100mm": 0.001*omega*omega*RADIUS_M,
    })

report = {
    "status": "TPU_SCREEN_REQUIRES_PHYSICAL_OVERSPEED_AND_HUB_REMAINS_UNDEFINED",
    "inputs": {"tpu_density_kg_m3": RHO_TPU, "tpu_modulus_screen_mpa": list(TPU_E_MPA),
               "radius_m": RADIUS_M, "wheel_spin_inertia_kg_m2": SPIN_INERTIA,
               "d5065_rotor_inertia_kg_m2": ROTOR_INERTIA, "d5065_kv_rpm_v": KV_RPM_V,
               "d5065_kt_nm_a": KT, "phase_neutral_resistance_ohm": R_OHM,
               "operating_current_limit_a": CURRENT_A},
    "rows": rows,
    "tpu_interpretation": "Thin free-ring rho*v^2 screen only. Bonding, layer orientation, creep, temperature, imbalance and hub constraint are not represented; no repository TPU tensile/allowable value exists, so this is not a burst-safety pass.",
    "hub": {
        "status": "NOT_RELEASABLE",
        "provisional_model_mass_kg": PROVISIONAL_HUB_MASS,
        "provisional_model_polar_inertia_kg_m2": PROVISIONAL_HUB_INERTIA,
        "maximum_transmitted_motor_torque_nm": KT*CURRENT_A,
        "reason": "The aluminium clamp hub is only a primitive mass/inertia envelope. No controlled actual hub drawing, material, bore, engagement, axial retention, balance or torque capacity exists, so hub stress or expansion cannot be calculated honestly.",
        "screen_interpretation": "At 24 m/s the rim sees 5760 m/s^2 and each gram of imbalance at 100 mm produces 5.76 N radial force; at the required 1.5x overspeed this becomes 12.96 N per gram. The interface must also transmit 0.62 N m per motor.",
        "required_physical_gate": "selected metal hub drawing plus balance/runout, 1.5x commanded-rpm guarded overspeed, thermal soak and post-test crack/creep inspection"
    },
    "motor_interpretation": "All 1146-2292 rpm points are below 57% of 15 V no-load speed and have >=5.73 V electrical margin at the provisional 20 A limit. Speed is feasible; continuous torque, controller, battery sag, cooling and measured rotating inertia remain physical gates."
}

out = Path("engineering/chassis-strength-bench/results/shared_flywheel_phase2/gazebo_5d1_p259/rotor_static.json")
out.parent.mkdir(parents=True, exist_ok=True)
out.write_text(json.dumps(report, indent=2) + "\n")
print(json.dumps(report, indent=2))
