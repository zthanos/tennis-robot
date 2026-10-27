#!/usr/bin/env python3
"""Conservative closed-form structural screen for the P2-67 inclined arch."""
from __future__ import annotations

import json
import math
from pathlib import Path

OUT = Path("engineering/chassis-strength-bench/results/shared_flywheel_phase2/p267_arch_screen.json")

# Geometry in mm; conservative PETG member properties.  This is a beam screen,
# not local joint, layer, fatigue or impact certification.
base_x, base_z = 318.0, 52.0
top_x, top_z = 390.0, 245.0
length = math.hypot(top_x-base_x, top_z-base_z)
inclination = math.degrees(math.atan2(top_x-base_x, top_z-base_z))
b, h, wall = 50.0, 30.0, 3.2
iy = (b*h**3 - (b-2*wall)*(h-2*wall)**3)/12
section_modulus = iy/(h/2)
elastic_modulus = 2000.0  # MPa, deliberately conservative printed-PETG screen
screen_allowable = 20.0  # MPa, not a material certification value

# Frozen P2-40 hold torque 12.6 N m.  Minimum actuator lever arm over
# -10..+20 deg is calculated from the actual fixed/moving clevis geometry.
pivot = (387.0, 49.837363636)
fixed = (320.0, 120.0)
forces = []
for angle in range(-10, 21):
    t = math.radians(angle)
    moving = (pivot[0] + math.cos(t)*50 - math.sin(t)*90,
              pivot[1] + math.sin(t)*50 + math.cos(t)*90)
    dx, dz = fixed[0]-moving[0], fixed[1]-moving[1]
    actuator_length = math.hypot(dx, dz)
    ux, uz = dx/actuator_length, dz/actuator_length
    rx, rz = moving[0]-pivot[0], moving[1]-pivot[1]
    lever = abs(rx*uz-rz*ux)
    forces.append((angle, 12600.0/lever))
max_angle, actuator_force = max(forces, key=lambda item: item[1])

# The right leg takes the complete actuator force at Z120.  LiDAR 0.15 kg,
# camera 0.08 kg and 0.35 kg mounts are screened at 3g, split over two legs.
sensor_mass = 0.58
sensor_horizontal_each = sensor_mass*9.81*3/2
actuator_height = 120.0-base_z
top_height = top_z-base_z
base_moment_nmm = actuator_force*actuator_height + sensor_horizontal_each*top_height
stress_mpa = base_moment_nmm/section_modulus
fos = screen_allowable/stress_mpa

# Cantilever superposition gives a deliberately conservative horizontal top
# displacement; braces and crossbar sharing are ignored.
actuator_station = actuator_height/math.cos(math.radians(inclination))
tip_from_actuator = (actuator_force*actuator_station**2*(3*length-actuator_station)
                     /(6*elastic_modulus*iy))
tip_from_sensors = sensor_horizontal_each*length**3/(3*elastic_modulus*iy)
tip_deflection = tip_from_actuator + tip_from_sensors

result = {
    "phase": "P2-67",
    "method": "conservative single-leg Euler-Bernoulli closed-form screen",
    "geometry": {
        "base_x_z_mm": [base_x, base_z], "top_x_z_mm": [top_x, top_z],
        "inclination_from_vertical_deg": inclination, "member_length_mm": length,
        "tube_outer_mm": [30.0, 50.0], "wall_mm": wall,
        "crossmember_rear_edge_mm": top_x-25,
    },
    "loads": {
        "p2_40_hold_torque_nm": 12.6,
        "worst_angle_deg": max_angle,
        "actuator_force_n": actuator_force,
        "sensor_and_mount_mass_kg": sensor_mass,
        "sensor_inertial_case_g": 3,
        "sensor_horizontal_force_per_leg_n": sensor_horizontal_each,
    },
    "material_screen": {"elastic_modulus_mpa": elastic_modulus,
                         "screen_allowable_mpa": screen_allowable},
    "results": {"base_moment_nm": base_moment_nmm/1000,
                "max_bending_stress_mpa": stress_mpa,
                "factor_to_screen_allowable": fos,
                "top_deflection_mm": tip_deflection},
    "limitations": [
        "No credit for the two chassis braces or crossbar load sharing.",
        "Does not certify clevis bracket, fastener bearing, printed layers, fatigue, vibration or impact.",
        "Physical proof-load and vibration checks remain mandatory before release.",
    ],
}
OUT.parent.mkdir(parents=True, exist_ok=True)
OUT.write_text(json.dumps(result, indent=2)+"\n")
print(json.dumps(result, indent=2))
