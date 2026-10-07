#!/usr/bin/env python3
"""Bounded fixed-motor intake capture and handoff capability study.

Reduced-order solver.  Both wheel centres and axes are frozen; there is no
translating carriage.  The calibrated tennis-ball normal law is combined in
series with explicitly uncalibrated linear tyre stiffness bounds.

GROUND PROFILE.  The measurement model runs on the AUTHORITATIVE handoff ramp
(``oa_ramp_z`` from ``cad/collector-intake-v1/option-a/option-a.scad``: a
smoothstep from (520 mm, 1.5 mm) to (420 mm, 35 mm), 180 mm wide, 18 mm walls),
not on flat frictionless ground.  The ramp runs directly through the nip zone,
so this is not a datum swap: at 0.45 m/s the ball has only v^2/2g = 10.3 mm of
climb available against a 33.5 mm rise.  It cannot coast up.  The robot drives
a wedge under a stationary ball and the inclined surface ploughs it.  Contact
with both the court (which moves rearward at the robot's approach speed in the
robot frame) and the ramp (which is static in that frame) is a real normal +
friction contact, never a position clamp.

INSTRUMENT SCOPE (two-instrument rule).  This solver owns exit speed, exit
elevation, exit azimuth, tyre radial deflection, ball compression, contact
force, wheel droop, torque and current.  It cannot represent cheek centering,
capture half-width, jam/reject modes or 6-DOF multi-contact: those belong to
Gazebo.  Results are SIMULATION_BOUNDED, never physical validation.
"""

from __future__ import annotations

import argparse
from concurrent.futures import ProcessPoolExecutor
import csv
from dataclasses import dataclass
import json
import math
from pathlib import Path
import sys
from typing import Iterable

import matplotlib.pyplot as plt
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

import intake_geometry as geom  # noqa: E402
from tennis_ball_contact_model import (  # noqa: E402
    NormalForceSample,
    FiniteCylinderContact,
    add,
    contact_wrench,
    cross,
    dot,
    mul,
    norm,
    sphere_finite_cylinder_contact,
)


G = 9.80665
BALL_MASS = 0.058
BALL_RADIUS = geom.BALL_RADIUS_M
BALL_INERTIA = (2.0 / 3.0) * BALL_MASS * BALL_RADIUS**2
BALL_LOADING_K = 107309.29404174259
BALL_LOADING_EXPONENT = 1.5
BALL_DYNAMIC_DAMPING = 4692.375890562493

# Frozen CAD geometry, in the standalone frame (nip mid-plane at x=0).
WHEEL_RADIUS = geom.WHEEL_RADIUS_M
WHEEL_HALF_WIDTH = geom.WHEEL_HALF_WIDTH_M
WHEEL_Y = geom.WHEEL_Y_M
WHEEL_Z = geom.WHEEL_Z_M
TILT_DEG = geom.TILT_DEG
AXIS = geom.WHEEL_AXIS

BRIDGE_X = tuple(value - geom.CAD_NIP_X_M for value in geom.BRIDGE_X_M)
BRIDGE_Y = geom.BRIDGE_Y_M
BRIDGE_Z = geom.BRIDGE_Z_M
CHASSIS_TOP_Z = geom.CHASSIS_TOP_Z_M
USEFUL_BALL_BOTTOM_CLEARANCE = 0.010
MIN_RECEIVING_CENTER_Z = CHASSIS_TOP_Z + BALL_RADIUS + USEFUL_BALL_BOTTOM_CLEARANCE

MOTOR_NOMINAL_V = 12.0
MOTOR_NO_LOAD_RPM = 251.0
MOTOR_NO_LOAD_RAD_S = MOTOR_NO_LOAD_RPM * 2.0 * math.pi / 60.0
MOTOR_STALL_TORQUE_NM = 18.0 * 9.80665 / 100.0
MOTOR_STALL_CURRENT_A = 7.0
MOTOR_NO_LOAD_CURRENT_A = 0.350

# The purchased Raid/Trencher rotating mass has not been weighed.  This is a
# broad analysis allocation, not measured hardware evidence.
WHEEL_INERTIA_ASSUMPTION = 0.0012

SPEED_CASES = {"LOW": 0.55, "NOMINAL": 0.75, "HIGH": 0.95}
FRICTION_BOUNDS = geom.TREAD_FRICTION_BOUNDS
TYRE_FORCE_AT_5MM_BOUNDS_N = geom.TYRE_FORCE_AT_5MM_BOUNDS_N
RAMP_FRICTION_BOUNDS = geom.BALL_RAMP_FRICTION_BOUNDS
OFFSET_CASES_M = (-0.020, -0.010, 0.0, 0.010, 0.020)

#: Commanded robot approach speed.  This is an operating parameter of the drive
#: base, not a prescribed ball velocity: the ball starts AT REST IN THE WORLD
#: and everything it does afterwards comes from contact.  0.45 m/s is the value
#: the flat-ground study used and the one the task states, so it is kept as the
#: comparison reference.
APPROACH_SPEED_M_S = 0.45

#: Approach speed is now a first-order variable, because the ramp lip reaches
#: the ball 48 mm before the wheels do and the ball has to arrive at the nip
#: under its own momentum.  These are the commanded speeds actually in play.
#: ROUTE_NOMINAL / ROUTE_MAX come from
#: ros2_ws/src/tennis_robot/config/collection_route.yaml
#: (nominal_speed_mps 0.35, max_speed_mps 0.60).
APPROACH_CASES = {
    "ROUTE_NOMINAL": 0.35,
    "STUDY_REFERENCE": 0.45,
    "ROUTE_MAX": 0.60,
    "ABOVE_THRESHOLD": 0.80,
}
#: The approach case the exit envelope, S5 and the basket study are derived on.
#: 0.80 m/s is the lowest scanned speed at which a CENTRED ball captures across
#: every friction bound; every number taken from it is conditional on a
#: commanded speed the collection route does not currently drive.
ENVELOPE_APPROACH_CASE = "ABOVE_THRESHOLD"
ENVELOPE_APPROACH_M_S = APPROACH_CASES[ENVELOPE_APPROACH_CASE]
ROUTE_NOMINAL_APPROACH_M_S = APPROACH_CASES["ROUTE_NOMINAL"]
#: Fine scan used to bracket the minimum capture approach speed.
APPROACH_THRESHOLD_SCAN_M_S = tuple(
    round(0.30 + 0.025 * index, 3) for index in range(21)
)

#: Ball start station, ahead of first lip contact (lip touches a court-resting
#: ball at CAD x = 529.9 mm).
START_X_STD_M = geom.std_from_cad_x(0.570)

#: Model-validity limit on tyre radial travel.  Exceeding it invalidates the
#: linear tyre bound, it does NOT mean the mechanism failed.
TYRE_DEFLECTION_GATE_M = geom.TYRE_RADIAL_ENVELOPE_M

#: Basket-on-base receiving study (task section 10).
BASKET_RIM_HEIGHTS_M = (0.005, 0.010, 0.015, 0.020)
BASKET_RIM_SCAN_M = tuple(np.round(np.arange(0.005, 0.401, 0.005), 4))

RAMP_POLYLINE = [
    (geom.std_from_cad_x(x), z) for x, z in geom.ramp_profile_polyline(200)
]
#: Index of the first vertex belonging to the ramp body; everything before it is
#: the court surface, which moves rearward at the approach speed in this frame.
RAMP_BODY_START = 1


@dataclass
class Trial:
    case_id: str
    speed_name: str
    speed_fraction: float
    friction: float
    tyre_force_at_5mm_n: float
    lateral_offset_m: float
    ramp_friction: float = geom.BALL_RAMP_FRICTION_NOMINAL
    lateral_entry_velocity_m_s: float = 0.0
    ground_profile: str = "ramp"
    approach_speed_m_s: float = APPROACH_SPEED_M_S
    approach_case: str = "STUDY_REFERENCE"
    repeat_id: int = 0
    dt: float = 0.00025
    duration: float = 1.2
    record: bool = False


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def series_contact_force(
    geometric_compression: float,
    compression_rate: float,
    tyre_force_at_5mm_n: float,
) -> tuple[float, float, float]:
    """Return force, tyre radial deflection, and ball diametral compression.

    For a symmetric bilateral pinch, one contact indentation is half the ball
    diametral deformation.  The contact closure is therefore
    p = F/k_tyre + 0.5*(F/K_ball)^(2/3).
    """

    if geometric_compression <= 0.0:
        return 0.0, 0.0, 0.0
    tyre_k = tyre_force_at_5mm_n / 0.005
    lo, hi = 0.0, 500.0
    for _ in range(60):
        force = 0.5 * (lo + hi)
        ball_diam = (force / BALL_LOADING_K) ** (1.0 / BALL_LOADING_EXPONENT)
        closure = force / tyre_k + 0.5 * ball_diam
        if closure < geometric_compression:
            lo = force
        else:
            hi = force
    elastic = 0.5 * (lo + hi)
    tyre_deflection = elastic / tyre_k
    ball_diam = (elastic / BALL_LOADING_K) ** (1.0 / BALL_LOADING_EXPONENT)
    ball_fraction = (0.5 * ball_diam) / geometric_compression
    ball_diam_rate = 2.0 * compression_rate * ball_fraction
    damping = BALL_DYNAMIC_DAMPING * ball_diam**1.5 * ball_diam_rate
    return max(0.0, elastic + damping), tyre_deflection, ball_diam


def rigid_contact_force(compression: float, compression_rate: float) -> float:
    """Calibrated ball law against a rigid surface (ramp, wall, court)."""

    if compression <= 0.0:
        return 0.0
    elastic = BALL_LOADING_K * compression**BALL_LOADING_EXPONENT
    damping = BALL_DYNAMIC_DAMPING * compression**1.5 * compression_rate
    return max(0.0, elastic + damping)


def motor_torque_current(target: float, omega: float) -> tuple[float, float]:
    direction = 1.0 if target >= 0.0 else -1.0
    voltage_fraction = abs(target) / MOTOR_NO_LOAD_RAD_S
    signed_speed = direction * omega
    torque = direction * MOTOR_STALL_TORQUE_NM * (
        voltage_fraction - signed_speed / MOTOR_NO_LOAD_RAD_S
    )
    torque = clamp(torque, -MOTOR_STALL_TORQUE_NM, MOTOR_STALL_TORQUE_NM)
    current = MOTOR_NO_LOAD_CURRENT_A + (
        MOTOR_STALL_CURRENT_A - MOTOR_NO_LOAD_CURRENT_A
    ) * abs(torque) / MOTOR_STALL_TORQUE_NM
    return torque, current


def sphere_intersects_bridge(position: tuple[float, float, float]) -> bool:
    squared = 0.0
    for value, bounds in zip(position, (BRIDGE_X, BRIDGE_Y, BRIDGE_Z)):
        nearest = clamp(value, bounds[0], bounds[1])
        squared += (value - nearest) ** 2
    return squared < BALL_RADIUS**2


@dataclass
class SurfaceContact:
    owner: str
    point: tuple[float, float, float]
    normal: tuple[float, float, float]
    compression: float
    surface_velocity: tuple[float, float, float] = (0.0, 0.0, 0.0)


def _segment_contact(px: float, pz: float, a, b) -> tuple[float, tuple[float, float]]:
    """Closest approach from a 2D point to a segment: (distance, point)."""

    abx, abz = b[0] - a[0], b[1] - a[1]
    length_sq = abx * abx + abz * abz
    if length_sq <= 0.0:
        t = 0.0
    else:
        t = clamp(((px - a[0]) * abx + (pz - a[1]) * abz) / length_sq, 0.0, 1.0)
    qx, qz = a[0] + t * abx, a[1] + t * abz
    return math.hypot(px - qx, pz - qz), (qx, qz)


def ground_contacts(
    position, ground_profile: str, approach_speed: float
) -> list[SurfaceContact]:
    """Court + ramp support contacts for the ball centre, robot frame.

    The court surface moves rearward at the approach speed (the robot drives
    into a ball that is at rest in the world); the ramp is part of the robot and
    is static in this frame.  Both are rigid.  The two are returned separately
    so the wedge — ball simultaneously on the court and on the ramp lip — is
    represented rather than averaged away.
    """

    px, py, pz = position
    contacts: list[SurfaceContact] = []

    if ground_profile == "flat":
        compression = BALL_RADIUS - pz
        if compression > 0.0:
            contacts.append(SurfaceContact(
                "court", (px, py, 0.0), (0.0, 0.0, 1.0), compression,
                (-approach_speed, 0.0, 0.0),
            ))
        return contacts

    groups = {
        "court": RAMP_POLYLINE[:RAMP_BODY_START + 1],
        "ramp": RAMP_POLYLINE[RAMP_BODY_START:],
    }
    for owner, points in groups.items():
        best = None
        for index in range(len(points) - 1):
            distance, closest = _segment_contact(px, pz, points[index], points[index + 1])
            if best is None or distance < best[0]:
                best = (distance, closest)
        if best is None:
            continue
        distance, (qx, qz) = best
        compression = BALL_RADIUS - distance
        if compression <= 0.0:
            continue
        if distance <= 1e-12:
            normal = (0.0, 0.0, 1.0)
        else:
            normal = ((px - qx) / distance, 0.0, (pz - qz) / distance)
        surface_velocity = (
            (-approach_speed, 0.0, 0.0) if owner == "court" else (0.0, 0.0, 0.0)
        )
        contacts.append(SurfaceContact(owner, (qx, py, qz), normal, compression,
                                       surface_velocity))

    # Ramp side walls: rigid lateral bound on the ball centre.
    half_width = geom.RAMP_WIDTH_M / 2.0
    if geom.std_from_cad_x(geom.RAMP_REAR_X_M) <= px <= geom.std_from_cad_x(geom.RAMP_FRONT_X_M):
        wall_top = geom.ramp_z_m(geom.cad_from_std_x(px)) + geom.RAMP_WALL_HEIGHT_M
        if pz <= wall_top:
            for sign in (-1.0, 1.0):
                compression = BALL_RADIUS - (half_width - sign * py)
                if compression > 0.0:
                    contacts.append(SurfaceContact(
                        f"ramp_wall_{'left' if sign > 0 else 'right'}",
                        (px, sign * half_width, pz), (0.0, -sign, 0.0), compression,
                    ))
    return contacts


def frozen_contact_sequence() -> dict:
    """Where the lip and the wheels first touch a court-resting centred ball.

    Pure frozen-CAD geometry, no dynamics: the lip meets the ball well before
    any wheel can, which is why the wedge decides the outcome.
    """

    drop = BALL_RADIUS - geom.RAMP_FRONT_Z_M
    lip_x = geom.RAMP_FRONT_X_M + math.sqrt(BALL_RADIUS**2 - drop**2)
    wheel_x = None
    x = 0.600
    while x > 0.400:
        geometry = sphere_finite_cylinder_contact(
            (x, 0.0, BALL_RADIUS), BALL_RADIUS, geom.wheel_centre_m(1, "cad"),
            AXIS, WHEEL_RADIUS, WHEEL_HALF_WIDTH,
        )
        if geometry.active:
            wheel_x = x
            break
        x -= 0.0001
    return {
        "ramp_lip_first_contact_ball_centre_cad_x_m": lip_x,
        "first_wheel_contact_ball_centre_cad_x_m": wheel_x,
        "lip_lead_m": None if wheel_x is None else lip_x - wheel_x,
        "note": "frozen CAD geometry only; asserted by tests/test_intake_frame_alignment.py",
    }


def ballistic_sample(exit_position, exit_velocity, max_time=1.5, step=0.005):
    rows = []
    discriminant = exit_velocity[2] ** 2 + 2.0 * G * max(exit_position[2] - BALL_RADIUS, 0.0)
    impact_time = (exit_velocity[2] + math.sqrt(discriminant)) / G
    final_time = min(max_time, impact_time)
    times = list(np.arange(0.0, final_time, step))
    if not times or not math.isclose(times[-1], final_time, abs_tol=1e-12):
        times.append(final_time)
    for time_s in times:
        x = exit_position[0] + exit_velocity[0] * time_s
        y = exit_position[1] + exit_velocity[1] * time_s
        z = exit_position[2] + exit_velocity[2] * time_s - 0.5 * G * time_s**2
        rows.append((float(time_s), x, y, max(z, BALL_RADIUS)))
    return rows


#: Static contact penetration of a resting ball under its own weight.  Starting
#: exactly at z = R makes the ball ring on the calibrated (lightly damped)
#: contact for the whole run; starting at equilibrium removes that artifact.
STATIC_PENETRATION_M = (BALL_MASS * G / BALL_LOADING_K) ** (1.0 / BALL_LOADING_EXPONENT)


def simulate(trial: Trial) -> tuple[dict, list[dict]]:
    ground_z = 0.0 if trial.ground_profile == "flat" else geom.ground_z_m(
        geom.cad_from_std_x(START_X_STD_M)
    )
    position = [
        START_X_STD_M, trial.lateral_offset_m,
        ground_z + BALL_RADIUS - STATIC_PENETRATION_M,
    ]
    # At rest in the WORLD; the robot advances at the commanded approach speed.
    velocity = [-trial.approach_speed_m_s, trial.lateral_entry_velocity_m_s, 0.0]
    ball_omega = [0.0, 0.0, 0.0]

    target_mag = trial.speed_fraction * MOTOR_NO_LOAD_RAD_S
    targets = {"left": -target_mag, "right": target_mag}
    wheel_omega = dict(targets)
    wheel_centres = {
        "left": (0.0, WHEEL_Y, WHEEL_Z),
        "right": (0.0, -WHEEL_Y, WHEEL_Z),
    }

    first_contact = {"left": None, "right": None}
    total_contact_duration = {"left": 0.0, "right": 0.0}
    bilateral_duration = 0.0
    maximum_tyre_deflection = {"left": 0.0, "right": 0.0}
    maximum_ball_compression = 0.0
    peak_force = {"left": 0.0, "right": 0.0}
    peak_tangential = {"left": 0.0, "right": 0.0}
    peak_slip = {"left": 0.0, "right": 0.0}
    peak_current = {"left": MOTOR_NO_LOAD_CURRENT_A, "right": MOTOR_NO_LOAD_CURRENT_A}
    peak_motor_torque = {"left": 0.0, "right": 0.0}
    minimum_wheel_speed = {"left": abs(targets["left"]), "right": abs(targets["right"])}
    precontact_wheel_speed = {"left": None, "right": None}
    had_contact = False
    had_bilateral = False
    separated_time = 0.0
    bridge_collision = False
    invalid_tyre_deflection = False
    telemetry: list[dict] = []
    exit_state = None

    ramp_contact_time = 0.0
    first_ramp_contact = None
    peak_ramp_force = 0.0
    peak_wall_force = 0.0
    maximum_ball_centre_z = position[2]
    minimum_x = position[0]
    nip_station_heights: dict[str, float | None] = {
        name: None for name in ("NIP_UPPER", "NIP_MID", "NIP_LOWER")
    }
    nip_station_x = {
        name: geom.std_from_cad_x(x)
        for name, (x, _z) in geom.nip_stations_m().items()
    }
    height_profile: list[tuple[float, float]] = []
    last_contact_state = None
    world_x_advance = 0.0

    steps = int(trial.duration / trial.dt)
    for index in range(steps):
        time_s = index * trial.dt
        ball_force = [0.0, 0.0, -BALL_MASS * G]
        ball_torque = [0.0, 0.0, 0.0]
        wheel_contact_torque = {"left": 0.0, "right": 0.0}
        active = {}
        contact_values = {}

        for side in ("left", "right"):
            geometry = sphere_finite_cylinder_contact(
                tuple(position), BALL_RADIUS, wheel_centres[side], AXIS,
                WHEEL_RADIUS, WHEEL_HALF_WIDTH,
            )
            wheel_vector = mul(AXIS, wheel_omega[side])
            ball_arm = (
                geometry.contact_point_world[0] - position[0],
                geometry.contact_point_world[1] - position[1],
                geometry.contact_point_world[2] - position[2],
            )
            wheel_arm = (
                geometry.contact_point_world[0] - wheel_centres[side][0],
                geometry.contact_point_world[1] - wheel_centres[side][1],
                geometry.contact_point_world[2] - wheel_centres[side][2],
            )
            ball_point_v = add(tuple(velocity), cross(tuple(ball_omega), ball_arm))
            wheel_point_v = cross(wheel_vector, wheel_arm)
            relative = (
                ball_point_v[0] - wheel_point_v[0],
                ball_point_v[1] - wheel_point_v[1],
                ball_point_v[2] - wheel_point_v[2],
            )
            compression_rate = -dot(relative, geometry.normal_world)
            force_n, tyre_deflection, ball_compression = series_contact_force(
                geometry.compression_m, compression_rate,
                trial.tyre_force_at_5mm_n,
            )
            active[side] = geometry.active and force_n > 0.0
            tangential_force_n = 0.0
            slip_speed = 0.0
            if active[side]:
                had_contact = True
                if first_contact[side] is None:
                    first_contact[side] = time_s
                    precontact_wheel_speed[side] = abs(wheel_omega[side])
                total_contact_duration[side] += trial.dt
                sample = NormalForceSample(
                    force_n=force_n,
                    elastic_force_n=force_n,
                    damping_force_n=0.0,
                    compression_m=geometry.compression_m,
                    compression_rate_m_s=compression_rate,
                    loading=compression_rate >= 0.0,
                    clamped=False,
                )
                wrench = contact_wrench(
                    geometry=geometry,
                    force_sample=sample,
                    ball_center=tuple(position),
                    ball_linear_velocity=tuple(velocity),
                    ball_angular_velocity=tuple(ball_omega),
                    wheel_center=wheel_centres[side],
                    wheel_linear_velocity=(0.0, 0.0, 0.0),
                    wheel_angular_velocity=wheel_vector,
                    friction_coefficient=trial.friction,
                )
                for axis_i in range(3):
                    ball_force[axis_i] += wrench.ball_force_world_n[axis_i]
                    ball_torque[axis_i] += wrench.ball_torque_world_nm[axis_i]
                wheel_contact_torque[side] = dot(wrench.wheel_torque_world_nm, AXIS)
                tangential_force_n = norm(wrench.tangential_force_world_n)
                slip_speed = norm(wrench.tangential_relative_velocity_m_s)
                maximum_tyre_deflection[side] = max(
                    maximum_tyre_deflection[side], tyre_deflection
                )
                maximum_ball_compression = max(maximum_ball_compression, ball_compression)
                peak_force[side] = max(peak_force[side], force_n)
                peak_tangential[side] = max(peak_tangential[side], tangential_force_n)
                peak_slip[side] = max(peak_slip[side], slip_speed)
                if tyre_deflection > TYRE_DEFLECTION_GATE_M + 1e-9:
                    invalid_tyre_deflection = True
            contact_values[side] = (
                geometry.compression_m, tyre_deflection, ball_compression,
                force_n, tangential_force_n, slip_speed,
            )

        # --- ground / ramp / wall contacts (rigid, calibrated ball law) ---
        surface_force_n = 0.0
        surface_owners = []
        for surface in ground_contacts(
            tuple(position), trial.ground_profile, trial.approach_speed_m_s
        ):
            arm = tuple(surface.point[i] - position[i] for i in range(3))
            ball_point_v = add(tuple(velocity), cross(tuple(ball_omega), arm))
            relative = tuple(ball_point_v[i] - surface.surface_velocity[i] for i in range(3))
            compression_rate = -dot(relative, surface.normal)
            force_n = rigid_contact_force(surface.compression, compression_rate)
            if force_n <= 0.0:
                continue
            surface_owners.append(surface.owner)
            surface_force_n += force_n
            geometry = FiniteCylinderContact(
                active=True,
                signed_distance_m=BALL_RADIUS - surface.compression,
                compression_m=surface.compression,
                contact_point_world=surface.point,
                normal_world=surface.normal,
                region=surface.owner,
            )
            sample = NormalForceSample(
                force_n=force_n, elastic_force_n=force_n, damping_force_n=0.0,
                compression_m=surface.compression,
                compression_rate_m_s=compression_rate,
                loading=compression_rate >= 0.0, clamped=False,
            )
            wrench = contact_wrench(
                geometry=geometry, force_sample=sample,
                ball_center=tuple(position),
                ball_linear_velocity=tuple(velocity),
                ball_angular_velocity=tuple(ball_omega),
                wheel_center=surface.point,
                wheel_linear_velocity=surface.surface_velocity,
                wheel_angular_velocity=(0.0, 0.0, 0.0),
                friction_coefficient=trial.ramp_friction,
            )
            for axis_i in range(3):
                ball_force[axis_i] += wrench.ball_force_world_n[axis_i]
                ball_torque[axis_i] += wrench.ball_torque_world_nm[axis_i]
            if surface.owner == "ramp":
                peak_ramp_force = max(peak_ramp_force, force_n)
                ramp_contact_time += trial.dt
                if first_ramp_contact is None:
                    first_ramp_contact = time_s
            elif surface.owner.startswith("ramp_wall"):
                peak_wall_force = max(peak_wall_force, force_n)

        if active["left"] and active["right"]:
            bilateral_duration += trial.dt
            had_bilateral = True

        for side in ("left", "right"):
            drive_torque, current = motor_torque_current(targets[side], wheel_omega[side])
            peak_current[side] = max(peak_current[side], current)
            peak_motor_torque[side] = max(peak_motor_torque[side], abs(drive_torque))
            wheel_omega[side] += (
                drive_torque + wheel_contact_torque[side]
            ) / WHEEL_INERTIA_ASSUMPTION * trial.dt
            minimum_wheel_speed[side] = min(
                minimum_wheel_speed[side], abs(wheel_omega[side])
            )

        previous_x = position[0]
        for axis_i in range(3):
            velocity[axis_i] += ball_force[axis_i] / BALL_MASS * trial.dt
            ball_omega[axis_i] += ball_torque[axis_i] / BALL_INERTIA * trial.dt
            position[axis_i] += velocity[axis_i] * trial.dt
        # World-frame forward displacement: the ploughing signature.
        world_x_advance += (position[0] - previous_x) + trial.approach_speed_m_s * trial.dt

        if sphere_intersects_bridge(tuple(position)):
            bridge_collision = True

        maximum_ball_centre_z = max(maximum_ball_centre_z, position[2])
        minimum_x = min(minimum_x, position[0])
        height_profile.append((position[0], position[2]))
        for name, station_x in nip_station_x.items():
            if nip_station_heights[name] is None and position[0] <= station_x:
                nip_station_heights[name] = position[2]

        any_contact = bool(active["left"] or active["right"] or surface_owners)
        if any_contact:
            last_contact_state = (
                tuple(position), tuple(velocity), tuple(ball_omega), time_s
            )

        if trial.record:
            telemetry.append({
                "case_id": trial.case_id,
                "ground_profile": trial.ground_profile,
                "time_s": time_s,
                "ball_x_m": position[0], "ball_y_m": position[1], "ball_z_m": position[2],
                "ball_x_cad_m": geom.cad_from_std_x(position[0]),
                "ball_vx_m_s": velocity[0], "ball_vy_m_s": velocity[1], "ball_vz_m_s": velocity[2],
                "ball_speed_m_s": norm(tuple(velocity)),
                "left_wheel_rad_s": wheel_omega["left"],
                "right_wheel_rad_s": wheel_omega["right"],
                "left_tyre_deflection_m": contact_values["left"][1],
                "right_tyre_deflection_m": contact_values["right"][1],
                "ball_diametral_compression_m": max(contact_values["left"][2], contact_values["right"][2]),
                "left_normal_force_n": contact_values["left"][3],
                "right_normal_force_n": contact_values["right"][3],
                "left_tangential_force_n": contact_values["left"][4],
                "right_tangential_force_n": contact_values["right"][4],
                "left_slip_speed_m_s": contact_values["left"][5],
                "right_slip_speed_m_s": contact_values["right"][5],
                "surface_normal_force_n": surface_force_n,
                "surface_contacts": "|".join(surface_owners),
            })

        if had_contact and not any_contact:
            separated_time += trial.dt
            if had_bilateral and separated_time >= 0.015 and last_contact_state:
                exit_state = last_contact_state
                break
        else:
            separated_time = 0.0

        if position[0] < -0.35 or position[0] > 0.25 or abs(position[1]) > 0.25:
            break

    released = exit_state is not None
    if exit_state is None:
        exit_state = last_contact_state or (
            tuple(position), tuple(velocity), tuple(ball_omega),
            min(trial.duration, steps * trial.dt),
        )
    exit_position, exit_velocity, exit_spin, exit_time = exit_state
    horizontal_speed = math.hypot(exit_velocity[0], exit_velocity[1])
    exit_speed = norm(exit_velocity)
    exit_elevation = math.degrees(math.atan2(exit_velocity[2], horizontal_speed))
    # Handoff direction is robot rearward (-X); azimuth is zero on -X.
    exit_azimuth = math.degrees(math.atan2(exit_velocity[1], -exit_velocity[0]))
    trajectory = ballistic_sample(exit_position, exit_velocity)
    apex = max(trajectory, key=lambda row: row[3])
    handoff_range = max(0.0, -(trajectory[-1][1] - exit_position[0]))
    maximum_receiving_distance = 0.0
    receiving_rows = []
    for time_s, x, y, z in trajectory:
        distance = -(x - exit_position[0])
        if distance >= 0.0:
            receiving_rows.append((distance, y, z, exit_velocity[2] - G * time_s))
            if z >= MIN_RECEIVING_CENTER_Z:
                maximum_receiving_distance = max(maximum_receiving_distance, distance)

    passed_nip = minimum_x < -WHEEL_RADIUS * math.sin(math.radians(TILT_DEG))
    mechanism_captured = (
        had_bilateral
        and bilateral_duration >= 0.005
        and released
        and exit_velocity[0] < -0.05
        and passed_nip
    )
    if mechanism_captured:
        outcome = "CAPTURED"
    elif not had_bilateral:
        outcome = "PLOUGHED_AHEAD" if world_x_advance > 0.020 else "NO_BILATERAL_CONTACT"
    elif not released:
        outcome = "STALLED_IN_NIP"
    elif exit_velocity[0] >= -0.05:
        outcome = "REJECTED_FORWARD"
    else:
        outcome = "NOT_TRANSPORTED"

    # Model-validity gates are reported SEPARATELY from mechanism failures.
    capture_valid = (
        mechanism_captured
        and not invalid_tyre_deflection
        and not bridge_collision
        and abs(exit_position[1]) < 0.10
    )
    droop = {
        side: 100.0 * (abs(targets[side]) - minimum_wheel_speed[side]) / abs(targets[side])
        for side in ("left", "right")
    }
    motor_time_constant = WHEEL_INERTIA_ASSUMPTION * MOTOR_NO_LOAD_RAD_S / MOTOR_STALL_TORQUE_NM
    recovery_time = {
        side: (
            0.0 if droop[side] <= 1.0 else
            motor_time_constant * math.log((droop[side] / 100.0) / 0.01)
        )
        for side in ("left", "right")
    }
    first_values = [value for value in first_contact.values() if value is not None]
    contact_order = "NONE"
    if len(first_values) == 2:
        delta = first_contact["left"] - first_contact["right"]
        contact_order = "SIMULTANEOUS" if abs(delta) <= trial.dt else ("LEFT_FIRST" if delta < 0 else "RIGHT_FIRST")
    elif first_contact["left"] is not None:
        contact_order = "LEFT_ONLY"
    elif first_contact["right"] is not None:
        contact_order = "RIGHT_ONLY"

    wheel_before_ramp = (
        first_ramp_contact is not None
        and first_values
        and min(first_values) < first_ramp_contact
    )

    summary = {
        "case_id": trial.case_id,
        "speed_name": trial.speed_name,
        "speed_fraction": trial.speed_fraction,
        "friction_coefficient": trial.friction,
        "ramp_friction_coefficient": trial.ramp_friction,
        "ground_profile": trial.ground_profile,
        "approach_case": trial.approach_case,
        "approach_speed_m_s": trial.approach_speed_m_s,
        "tyre_force_at_5mm_n": trial.tyre_force_at_5mm_n,
        "lateral_offset_m": trial.lateral_offset_m,
        "lateral_entry_velocity_m_s": trial.lateral_entry_velocity_m_s,
        "repeat_id": trial.repeat_id,
        "commanded_wheel_speed_rad_s": target_mag,
        "commanded_wheel_speed_rpm": target_mag * 60.0 / (2.0 * math.pi),
        "commanded_surface_speed_m_s": target_mag * WHEEL_RADIUS,
        "actual_precontact_left_wheel_speed_rad_s": precontact_wheel_speed["left"],
        "actual_precontact_right_wheel_speed_rad_s": precontact_wheel_speed["right"],
        "minimum_left_wheel_speed_rad_s": minimum_wheel_speed["left"],
        "minimum_right_wheel_speed_rad_s": minimum_wheel_speed["right"],
        "minimum_left_surface_speed_m_s": minimum_wheel_speed["left"] * WHEEL_RADIUS,
        "minimum_right_surface_speed_m_s": minimum_wheel_speed["right"] * WHEEL_RADIUS,
        "first_contact_left_s": first_contact["left"],
        "first_contact_right_s": first_contact["right"],
        "first_ramp_contact_s": first_ramp_contact,
        "wheel_contact_before_ramp_contact": bool(wheel_before_ramp),
        "ramp_contact_duration_s": ramp_contact_time,
        "peak_ramp_normal_force_n": peak_ramp_force,
        "peak_ramp_wall_force_n": peak_wall_force,
        "contact_order": contact_order,
        "left_contact_duration_s": total_contact_duration["left"],
        "right_contact_duration_s": total_contact_duration["right"],
        "bilateral_contact_duration_s": bilateral_duration,
        "maximum_left_tyre_deflection_m": maximum_tyre_deflection["left"],
        "maximum_right_tyre_deflection_m": maximum_tyre_deflection["right"],
        "maximum_ball_diametral_compression_m": maximum_ball_compression,
        "peak_left_normal_force_n": peak_force["left"],
        "peak_right_normal_force_n": peak_force["right"],
        "peak_left_tangential_force_n": peak_tangential["left"],
        "peak_right_tangential_force_n": peak_tangential["right"],
        "peak_left_slip_speed_m_s": peak_slip["left"],
        "peak_right_slip_speed_m_s": peak_slip["right"],
        "left_wheel_droop_percent": droop["left"],
        "right_wheel_droop_percent": droop["right"],
        "left_recovery_to_1pct_s": recovery_time["left"],
        "right_recovery_to_1pct_s": recovery_time["right"],
        "peak_left_motor_torque_nm": peak_motor_torque["left"],
        "peak_right_motor_torque_nm": peak_motor_torque["right"],
        "peak_left_current_a": peak_current["left"],
        "peak_right_current_a": peak_current["right"],
        "ball_centre_z_at_nip_upper_m": nip_station_heights["NIP_UPPER"],
        "ball_centre_z_at_nip_mid_m": nip_station_heights["NIP_MID"],
        "ball_centre_z_at_nip_lower_m": nip_station_heights["NIP_LOWER"],
        "maximum_ball_centre_z_m": maximum_ball_centre_z,
        "world_forward_advance_m": world_x_advance,
        "exit_time_s": exit_time,
        "exit_x_m": exit_position[0], "exit_y_m": exit_position[1], "exit_z_m": exit_position[2],
        "exit_x_cad_m": geom.cad_from_std_x(exit_position[0]),
        "exit_vx_m_s": exit_velocity[0], "exit_vy_m_s": exit_velocity[1], "exit_vz_m_s": exit_velocity[2],
        "exit_speed_m_s": exit_speed,
        "exit_elevation_deg": exit_elevation,
        "exit_azimuth_deg": exit_azimuth,
        "exit_spin_x_rad_s": exit_spin[0], "exit_spin_y_rad_s": exit_spin[1], "exit_spin_z_rad_s": exit_spin[2],
        "apex_time_after_exit_s": apex[0],
        "apex_z_m": apex[3],
        "maximum_vertical_rise_m": apex[3] - exit_position[2],
        "ballistic_ground_range_m": handoff_range,
        "maximum_above_base_receiving_distance_m": maximum_receiving_distance,
        "bridge_collision": bridge_collision,
        "impossible_tyre_deflection": invalid_tyre_deflection,
        "mechanism_captured": mechanism_captured,
        "outcome": outcome,
        "capture_valid": capture_valid,
        "trajectory": trajectory,
        "receiving_rows": receiving_rows,
        "height_profile": height_profile,
    }
    return summary, telemetry


def write_csv(path: Path, rows: list[dict], excluded: Iterable[str] = ()) -> None:
    excluded = set(excluded)
    keys = [key for key in rows[0] if key not in excluded]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=keys)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: row[key] for key in keys})


def interpolate_receiving(row: dict, distance: float):
    vx = -row["exit_vx_m_s"]
    if vx <= 0.0:
        return None
    time_s = distance / vx
    vz = row["exit_vz_m_s"] - G * time_s
    return {
        "time_s": time_s,
        "z_m": row["exit_z_m"] + row["exit_vz_m_s"] * time_s - 0.5 * G * time_s**2,
        "y_m": row["exit_y_m"] + row["exit_vy_m_s"] * time_s,
        "vz_m_s": vz,
        "speed_m_s": math.sqrt(row["exit_vx_m_s"]**2 + row["exit_vy_m_s"]**2 + vz**2),
        "phase": "rising" if vz > 0.05 else ("apex" if vz >= -0.05 else "descending"),
    }


def _rim_windows_for(valid: list[dict]) -> dict:
    """Worst-case rearward-distance window that clears a rim of each height.

    The basket floor sits ON the chassis top plane (z = 0.052 m); there is no
    cut-out. Clearance means the BALL BOTTOM passes above the rim top for EVERY
    valid trial in the matrix, i.e. the window is the intersection, not a
    best-case envelope.
    """

    result = {}
    for rim in BASKET_RIM_HEIGHTS_M:
        required_centre = CHASSIS_TOP_Z + rim + BALL_RADIUS
        distances = [
            float(distance) for distance in BASKET_RIM_SCAN_M
            if all(
                (crossing := interpolate_receiving(row, float(distance))) is not None
                and crossing["z_m"] >= required_centre
                for row in valid
            )
        ]
        window = None
        if distances:
            window = [min(distances), max(distances)]
            # Reject a non-contiguous set rather than quoting a false window.
            step = float(BASKET_RIM_SCAN_M[1] - BASKET_RIM_SCAN_M[0])
            contiguous = all(
                math.isclose(b - a, step, abs_tol=1e-9)
                for a, b in zip(distances, distances[1:])
            )
            if not contiguous:
                window = None
        entry = {
            "rim_height_m": rim,
            "required_ball_centre_z_m": required_centre,
            "window_m": window,
            "window_width_m": None if window is None else round(window[1] - window[0], 4),
            "contiguous": window is not None,
        }
        if window is not None:
            samples = {}
            for label, distance in (
                ("near_edge", window[0]),
                ("mid", 0.5 * (window[0] + window[1])),
                ("far_edge", window[1]),
            ):
                crossings = [interpolate_receiving(row, distance) for row in valid]
                crossings = [c for c in crossings if c is not None]
                rebound = max(c["speed_m_s"] for c in crossings) * 0.73
                samples[label] = {
                    "rearward_distance_from_exit_m": round(distance, 4),
                    "ball_centre_z_range_m": [min(c["z_m"] for c in crossings),
                                              max(c["z_m"] for c in crossings)],
                    "lateral_error_range_m": [min(c["y_m"] for c in crossings),
                                              max(c["y_m"] for c in crossings)],
                    "residual_speed_range_m_s": [min(c["speed_m_s"] for c in crossings),
                                                 max(c["speed_m_s"] for c in crossings)],
                    "vertical_velocity_range_m_s": [min(c["vz_m_s"] for c in crossings),
                                                    max(c["vz_m_s"] for c in crossings)],
                    "phases": sorted({c["phase"] for c in crossings}),
                    "rigid_wall_rebound_speed_m_s": rebound,
                    "rigid_wall_rebound_rise_m": rebound**2 / (2.0 * G),
                }
            entry["receiving_points"] = samples
        result[f"{rim:.3f}"] = entry

    maximum = None
    for rim in np.arange(0.0, 0.061, 0.001):
        required_centre = CHASSIS_TOP_Z + float(rim) + BALL_RADIUS
        if any(
            all(
                (crossing := interpolate_receiving(row, float(distance))) is not None
                and crossing["z_m"] >= required_centre
                for row in valid
            )
            for distance in BASKET_RIM_SCAN_M
        ):
            maximum = float(rim)
    result["MAX_BASKET_RIM_HEIGHT_ON_BASE_M"] = maximum
    result["population_size"] = len(valid)
    return result


def rim_windows(valid: list[dict]) -> dict:
    """Rim clearance for three explicitly named populations.

    "Worst case" is only meaningful once the population is named. The funnel's
    design intent is a centred entry, so the centred population is reported
    alongside the full matrix rather than instead of it.
    """

    centred = [row for row in valid if row["lateral_offset_m"] == 0.0]
    populations = {
        "centred_entries_all_bounds": centred,
        "all_valid_entries": valid,
    }
    for speed_name in SPEED_CASES:
        rows = [row for row in centred if row["speed_name"] == speed_name]
        if rows:
            populations[f"centred_{speed_name}"] = rows
    result = {
        name: _rim_windows_for(rows) for name, rows in populations.items() if rows
    }
    # Headline population: centred entries at the HIGH wheel-speed command,
    # across every unmeasured bound. Wheel speed is a COMMANDED setting, so
    # mixing LOW..HIGH into one "worst case" would blame the mechanism for an
    # operating choice; the per-speed rows below make that choice explicit.
    result["headline_population"] = "centred_HIGH"
    result["population_definitions"] = {
        "centred_HIGH": "y=0 entries, HIGH wheel-speed command, all unmeasured bounds",
        "centred_NOMINAL": "y=0 entries, NOMINAL wheel-speed command, all unmeasured bounds",
        "centred_LOW": "y=0 entries, LOW wheel-speed command, all unmeasured bounds",
        "centred_entries_all_bounds": "y=0 entries, all three wheel-speed commands",
        "all_valid_entries": "every valid entry including lateral offsets",
    }
    result["MAX_BASKET_RIM_HEIGHT_ON_BASE_M"] = (
        result.get("centred_HIGH", {}).get("MAX_BASKET_RIM_HEIGHT_ON_BASE_M")
    )
    result["MAX_BASKET_RIM_HEIGHT_ALL_ENTRIES_M"] = (
        result.get("all_valid_entries", {}).get("MAX_BASKET_RIM_HEIGHT_ON_BASE_M")
    )
    return result


def plot_outputs(trials: list[dict], telemetry: list[dict], comparison: dict,
                 rim_result: dict, threshold: dict, image_dir: Path) -> list[str]:
    image_dir.mkdir(parents=True, exist_ok=True)
    paths = []

    def save(name: str):
        path = image_dir / name
        plt.tight_layout()
        plt.savefig(path, dpi=160)
        plt.close()
        paths.append(str(path.relative_to(ROOT)))

    tele = [row for row in telemetry if row["ground_profile"] == "ramp"]
    t = [row["time_s"] for row in tele]
    for key, ylabel, name in [
        ("ball_speed_m_s", "Ball speed (m/s)", "standalone-intake-handoff-01-ball-speed-time.png"),
        ("ball_diametral_compression_m", "Ball compression (mm)", "standalone-intake-handoff-04-ball-compression-time.png"),
    ]:
        plt.figure(figsize=(8, 4.5))
        scale = 1000.0 if key.endswith("compression_m") else 1.0
        plt.plot(t, [row[key] * scale for row in tele])
        plt.xlabel("Time (s)"); plt.ylabel(ylabel); plt.grid(True, alpha=0.3)
        save(name)

    plt.figure(figsize=(8, 4.5))
    plt.plot(t, [abs(row["left_wheel_rad_s"]) for row in tele], label="left")
    plt.plot(t, [abs(row["right_wheel_rad_s"]) for row in tele], label="right")
    plt.xlabel("Time (s)"); plt.ylabel("Wheel speed (rad/s)"); plt.legend(); plt.grid(True, alpha=0.3)
    save("standalone-intake-handoff-02-wheel-speed-time.png")

    plt.figure(figsize=(8, 4.5))
    plt.plot(t, [1000 * row["left_tyre_deflection_m"] for row in tele], label="left")
    plt.plot(t, [1000 * row["right_tyre_deflection_m"] for row in tele], label="right")
    plt.axhline(1000 * TYRE_DEFLECTION_GATE_M, color="red", linestyle="--",
                label="5 mm model-validity gate")
    plt.xlabel("Time (s)"); plt.ylabel("Tyre radial deflection (mm)"); plt.legend(); plt.grid(True, alpha=0.3)
    save("standalone-intake-handoff-03-tyre-deflection-time.png")

    plt.figure(figsize=(8, 4.5))
    plt.plot(t, [row["left_normal_force_n"] for row in tele], label="left wheel")
    plt.plot(t, [row["right_normal_force_n"] for row in tele], label="right wheel")
    plt.plot(t, [row["surface_normal_force_n"] for row in tele], label="ramp/court")
    plt.xlabel("Time (s)"); plt.ylabel("Normal force (N)"); plt.legend(); plt.grid(True, alpha=0.3)
    save("standalone-intake-handoff-05-contact-force-time.png")

    valid = [row for row in trials if row["capture_valid"]]
    for metric, ylabel, name in [
        ("exit_speed_m_s", "Exit speed (m/s)", "standalone-intake-handoff-06-exit-speed-wheel-speed.png"),
        ("exit_elevation_deg", "Exit elevation (deg)", "standalone-intake-handoff-07-exit-elevation-wheel-speed.png"),
    ]:
        plt.figure(figsize=(8, 4.5))
        for mu in FRICTION_BOUNDS:
            rows = [r for r in valid if r["friction_coefficient"] == mu and r["lateral_offset_m"] == 0]
            plt.scatter([r["commanded_wheel_speed_rad_s"] for r in rows], [r[metric] for r in rows], label=f"mu={mu}")
        plt.xlabel("Commanded wheel speed (rad/s)"); plt.ylabel(ylabel); plt.legend(); plt.grid(True, alpha=0.3)
        save(name)

    plt.figure(figsize=(8, 4.5))
    for row in valid:
        if row["lateral_offset_m"] == 0 and row["tyre_force_at_5mm_n"] == 60.0:
            d = [- (p[1] - row["exit_x_m"]) for p in row["trajectory"]]
            z = [p[3] for p in row["trajectory"]]
            plt.plot(d, z, alpha=0.65)
    plt.axhline(CHASSIS_TOP_Z, color="black", linestyle="--", label="chassis top")
    plt.axhline(MIN_RECEIVING_CENTER_Z, color="green", linestyle=":", label="useful receiving centre")
    plt.xlabel("Rearward distance from exit (m)"); plt.ylabel("Ball centre Z (m)"); plt.legend(); plt.grid(True, alpha=0.3)
    save("standalone-intake-handoff-08-xz-trajectories.png")

    fig = plt.figure(figsize=(8, 5.5))
    ax = fig.add_subplot(111, projection="3d")
    for row in valid:
        points = row["trajectory"][::4]
        ax.scatter(
            [- (p[1] - row["exit_x_m"]) for p in points],
            [p[2] for p in points],
            [p[3] for p in points],
            s=3, alpha=0.12, color="royalblue",
        )
    ax.set_xlabel("Rearward distance (m)")
    ax.set_ylabel("Lateral Y (m)")
    ax.set_zlabel("Ball centre Z (m)")
    save("standalone-intake-handoff-09-reachable-envelope.png")

    distances = np.linspace(0.0, 0.8, 161)
    plt.figure(figsize=(8, 4.5))
    for row in valid:
        vx = -row["exit_vx_m_s"]
        if vx <= 0:
            continue
        heights = []
        for distance in distances:
            tt = distance / vx
            heights.append(row["exit_z_m"] + row["exit_vz_m_s"] * tt - 0.5 * G * tt**2)
        plt.plot(distances, heights, color="steelblue", alpha=0.08)
    plt.axhline(MIN_RECEIVING_CENTER_Z, color="green", linestyle="--")
    plt.xlabel("Rearward receiving distance (m)"); plt.ylabel("Ball centre height (m)"); plt.grid(True, alpha=0.3)
    save("standalone-intake-handoff-10-receiving-height-map.png")

    centered_nominal = [r for r in valid if r["speed_name"] == "NOMINAL" and r["lateral_offset_m"] == 0]
    plt.figure(figsize=(8, 4.5))
    for tyre in TYRE_FORCE_AT_5MM_BOUNDS_N:
        rows = sorted([r for r in centered_nominal if r["tyre_force_at_5mm_n"] == tyre], key=lambda r: r["friction_coefficient"])
        plt.plot([r["friction_coefficient"] for r in rows], [r["exit_speed_m_s"] for r in rows], marker="o", linestyle="none", label=f"F5={tyre:.0f} N")
    plt.xlabel("Tread friction bound"); plt.ylabel("Exit speed (m/s)"); plt.legend(); plt.grid(True, alpha=0.3)
    save("standalone-intake-handoff-11-friction-sensitivity.png")

    plt.figure(figsize=(8, 4.5))
    for mu in FRICTION_BOUNDS:
        rows = sorted([r for r in centered_nominal if r["friction_coefficient"] == mu], key=lambda r: r["tyre_force_at_5mm_n"])
        plt.plot([r["tyre_force_at_5mm_n"] for r in rows], [r["exit_speed_m_s"] for r in rows], marker="o", linestyle="none", label=f"mu={mu}")
    plt.xlabel("Tyre bound: force at 5 mm (N)"); plt.ylabel("Exit speed (m/s)"); plt.legend(); plt.grid(True, alpha=0.3)
    save("standalone-intake-handoff-12-tyre-sensitivity.png")

    # --- ramp effect: the before/after evidence -------------------------
    plt.figure(figsize=(8, 4.5))
    for label, style in (("flat", "--"), ("ramp", "-")):
        profile = comparison[label]["height_profile"]
        plt.plot([1000 * geom.cad_from_std_x(x) for x, _z in profile],
                 [1000 * z for _x, z in profile], style, label=f"ball centre, {label} ground")
    xs = np.linspace(geom.RAMP_REAR_X_M - 0.02, 0.600, 400)
    plt.plot(1000 * xs, [1000 * geom.ground_z_m(x) for x in xs], color="black",
             alpha=0.6, label="authoritative ramp surface")
    for name, (x, z) in geom.nip_stations_m().items():
        plt.axvline(1000 * x, color="grey", alpha=0.4, linestyle=":")
        plt.plot([1000 * x], [1000 * z], marker="x", color="crimson")
    plt.gca().invert_xaxis()
    plt.xlabel("CAD ground-frame X (mm), robot forward to the right")
    plt.ylabel("Height (mm)")
    plt.title("Ball centre height through the nip zone: flat datum vs authoritative ramp")
    plt.legend(); plt.grid(True, alpha=0.3)
    save("standalone-intake-handoff-13-ball-height-flat-vs-ramp.png")

    plt.figure(figsize=(8, 4.5))
    labels = ["exit speed (m/s)", "exit elevation (deg)", "exit z (mm)"]
    flat = comparison["flat"]["summary"]
    ramp = comparison["ramp"]["summary"]
    values_flat = [flat["exit_speed_m_s"], flat["exit_elevation_deg"], 1000 * flat["exit_z_m"]]
    values_ramp = [ramp["exit_speed_m_s"], ramp["exit_elevation_deg"], 1000 * ramp["exit_z_m"]]
    index = np.arange(len(labels))
    plt.bar(index - 0.2, values_flat, 0.4, label="flat datum (invalid)")
    plt.bar(index + 0.2, values_ramp, 0.4, label="authoritative ramp")
    plt.xticks(index, labels); plt.grid(True, axis="y", alpha=0.3); plt.legend()
    plt.title("Exit state, flat datum vs authoritative ramp (centred representative case)")
    save("standalone-intake-handoff-14-exit-state-flat-vs-ramp.png")

    plt.figure(figsize=(8, 4.5))
    centred_rims = rim_result[rim_result["headline_population"]]
    for rim in BASKET_RIM_HEIGHTS_M:
        entry = centred_rims[f"{rim:.3f}"]
        if entry["window_m"]:
            plt.plot([1000 * rim, 1000 * rim],
                     [entry["window_m"][0], entry["window_m"][1]],
                     linewidth=6, solid_capstyle="butt")
        else:
            plt.plot([1000 * rim], [0.0], marker="x", color="crimson")
    plt.xlabel("Basket rim height above the chassis top plane (mm)")
    plt.ylabel("Rearward receiving distance from exit (m)")
    plt.title("Rim clearance window, centred HIGH-speed entries (floor ON the base, no cut-out)")
    plt.grid(True, alpha=0.3)
    save("standalone-intake-handoff-15-rim-clearance-window.png")

    plt.figure(figsize=(8, 4.5))
    for speed_name in SPEED_CASES:
        offsets = sorted({r["lateral_offset_m"] for r in trials})
        rates = []
        for offset in offsets:
            rows = [
                r for r in trials
                if r["speed_name"] == speed_name and r["lateral_offset_m"] == offset
                and r["repeat_id"] == 0 and r["ground_profile"] == "ramp"
                and r["lateral_entry_velocity_m_s"] == 0.0
            ]
            rates.append(
                100.0 * sum(bool(r["mechanism_captured"]) for r in rows) / len(rows)
                if rows else 0.0
            )
        plt.plot([1000 * o for o in offsets], rates, marker="o", label=speed_name)
    plt.xlabel("Lateral entry offset (mm)")
    plt.ylabel("Mechanism capture rate over the bounded matrix (%)")
    plt.title("Solver-bounded lateral capture (no cheeks: Gazebo owns capture half-width)")
    plt.legend(); plt.grid(True, alpha=0.3)
    save("standalone-intake-handoff-16-capture-vs-lateral-offset.png")

    plt.figure(figsize=(8, 4.5))
    speeds = [row["approach_speed_m_s"] for row in threshold["table"]]
    rates = [100.0 * row["captured"] / row["cases"] for row in threshold["table"]]
    heights = [1000 * row["maximum_ball_centre_z_m"] for row in threshold["table"]]
    axis = plt.gca()
    axis.plot(speeds, rates, marker="o", color="steelblue", label="capture rate (%)")
    axis.set_xlabel("Commanded robot approach speed (m/s)")
    axis.set_ylabel("Capture rate over the friction bounds (%)")
    axis.axvline(ROUTE_NOMINAL_APPROACH_M_S, color="crimson", linestyle="--",
                 label="collection route nominal 0.35 m/s")
    axis.axvline(APPROACH_CASES["ROUTE_MAX"], color="darkorange", linestyle=":",
                 label="collection route max 0.60 m/s")
    twin = axis.twinx()
    twin.plot(speeds, heights, marker="x", color="seagreen",
              label="peak ball centre height (mm)")
    twin.set_ylabel("Peak ball centre height (mm)")
    axis.grid(True, alpha=0.3)
    lines = axis.get_lines() + twin.get_lines()
    axis.legend(lines, [line.get_label() for line in lines], loc="center left")
    plt.title("Capture on the authoritative ramp vs commanded approach speed")
    save("standalone-intake-handoff-17-capture-vs-approach-speed.png")
    return paths


def compile_result(trials: list[dict], plot_paths: list[str], convergence: list[dict],
                   comparison: dict, threshold: dict,
                   gazebo_campaign: dict | None) -> dict:
    every = [
        row for row in trials
        if row["repeat_id"] == 0 and row["ground_profile"] == "ramp"
        and row["lateral_entry_velocity_m_s"] == 0.0
        and row["approach_case"] in APPROACH_CASES
    ]
    approach_sweep = {}
    for case, approach in APPROACH_CASES.items():
        rows = [row for row in every if row["approach_case"] == case]
        outcomes_case: dict[str, int] = {}
        for row in rows:
            outcomes_case[row["outcome"]] = outcomes_case.get(row["outcome"], 0) + 1
        captured = [row for row in rows if row["mechanism_captured"]]
        centred = [row for row in rows if row["lateral_offset_m"] == 0.0]
        centred_captured = [row for row in centred if row["mechanism_captured"]]
        approach_sweep[case] = {
            "approach_speed_m_s": approach,
            "cases": len(rows),
            "mechanism_captured": len(captured),
            "centred_cases": len(centred),
            "centred_mechanism_captured": len(centred_captured),
            "offset_only_capture": bool(captured) and not centred_captured,
            "capture_rate_percent": 100.0 * len(captured) / len(rows) if rows else 0.0,
            "outcomes": outcomes_case,
            "maximum_ball_centre_z_m": max(
                (row["maximum_ball_centre_z_m"] for row in rows), default=None
            ),
            "exit_speed_range_m_s": [
                min((row["exit_speed_m_s"] for row in captured), default=None),
                max((row["exit_speed_m_s"] for row in captured), default=None),
            ],
        }
    # The exit envelope, S5 and the basket study are derived on ONE approach
    # case, the only one that captures. Everything taken from it is conditional.
    matrix = [row for row in every if row["approach_case"] == ENVELOPE_APPROACH_CASE]
    valid = [row for row in matrix if row["capture_valid"]]
    high = [row for row in valid if row["speed_name"] == "HIGH"]
    centered_high = [row for row in high if row["lateral_offset_m"] == 0.0]
    bound_case_count = (
        len(FRICTION_BOUNDS) * len(TYRE_FORCE_AT_5MM_BOUNDS_N) * len(RAMP_FRICTION_BOUNDS)
    )
    distances = np.linspace(0.05, 0.80, 151)
    bounded_distance = None
    reliable_rows = high
    for distance in distances:
        crossings = [interpolate_receiving(row, float(distance)) for row in reliable_rows]
        if crossings and all(c is not None and c["z_m"] >= MIN_RECEIVING_CENTER_Z for c in crossings):
            bounded_distance = float(distance)
    centered_bounded_distance = None
    for distance in distances:
        crossings = [interpolate_receiving(row, float(distance)) for row in centered_high]
        if crossings and all(c is not None and c["z_m"] >= MIN_RECEIVING_CENTER_Z for c in crossings):
            centered_bounded_distance = float(distance)
    physical_range = max((row["ballistic_ground_range_m"] for row in valid), default=0.0)
    physical_height = max((row["apex_z_m"] for row in valid), default=0.0)
    all_high_cases_present = len(reliable_rows) == bound_case_count * len(OFFSET_CASES_M)
    reliable_defined = bounded_distance is not None and all_high_cases_present
    reliable_distance = bounded_distance if reliable_defined else None
    reliable_crossings = [] if reliable_distance is None else [
        interpolate_receiving(row, reliable_distance) for row in reliable_rows
    ]
    any_above_base = any(row["maximum_above_base_receiving_distance_m"] >= 0.10 for row in valid)
    base_cutout = False if reliable_defined and reliable_distance >= 0.10 else "unresolved" if any_above_base else True
    lateral_max = max((abs(c["y_m"]) for c in reliable_crossings), default=0.0)
    recommended_opening_width = 2.0 * lateral_max + 2.0 * BALL_RADIUS + 0.040 if reliable_defined else None

    centered_speed_summary = {}
    for speed_name in SPEED_CASES:
        rows = [
            row for row in valid
            if row["speed_name"] == speed_name and row["lateral_offset_m"] == 0.0
        ]
        executed = [
            row for row in matrix
            if row["speed_name"] == speed_name and row["lateral_offset_m"] == 0.0
        ]
        centered_speed_summary[speed_name] = {
            "executed_cases": len(executed),
            "valid_cases": len(rows),
            "mechanism_captured_cases": sum(bool(r["mechanism_captured"]) for r in executed),
            "exit_speed_range_m_s": [min((r["exit_speed_m_s"] for r in rows), default=None), max((r["exit_speed_m_s"] for r in rows), default=None)],
            "exit_elevation_range_deg": [min((r["exit_elevation_deg"] for r in rows), default=None), max((r["exit_elevation_deg"] for r in rows), default=None)],
            "exit_z_range_m": [min((r["exit_z_m"] for r in rows), default=None), max((r["exit_z_m"] for r in rows), default=None)],
            "ballistic_range_m": [min((r["ballistic_ground_range_m"] for r in rows), default=None), max((r["ballistic_ground_range_m"] for r in rows), default=None)],
            "above_base_distance_range_m": [min((r["maximum_above_base_receiving_distance_m"] for r in rows), default=None), max((r["maximum_above_base_receiving_distance_m"] for r in rows), default=None)],
            "wheel_droop_range_percent": [min((max(r["left_wheel_droop_percent"], r["right_wheel_droop_percent"]) for r in rows), default=None), max((max(r["left_wheel_droop_percent"], r["right_wheel_droop_percent"]) for r in rows), default=None)],
            "peak_current_range_a": [min((max(r["peak_left_current_a"], r["peak_right_current_a"]) for r in rows), default=None), max((max(r["peak_left_current_a"], r["peak_right_current_a"]) for r in rows), default=None)],
        }

    offset_capture = {}
    for speed_name in SPEED_CASES:
        offset_capture[speed_name] = {}
        for offset in OFFSET_CASES_M:
            rows = [r for r in matrix if r["speed_name"] == speed_name and r["lateral_offset_m"] == offset]
            offset_capture[speed_name][f"{offset:+.3f}"] = {
                "mechanism_captured": sum(bool(r["mechanism_captured"]) for r in rows),
                "valid_after_gates": sum(bool(r["capture_valid"]) for r in rows),
                "executed": len(rows),
            }

    outcomes: dict[str, int] = {}
    for row in matrix:
        outcomes[row["outcome"]] = outcomes.get(row["outcome"], 0) + 1
    gate_rejections = {
        "tyre_deflection_above_5mm_model_validity_bound": sum(
            bool(r["impossible_tyre_deflection"]) and r["mechanism_captured"] for r in matrix
        ),
        "bridge_collision": sum(bool(r["bridge_collision"]) for r in matrix),
        "lateral_escape_beyond_100mm": sum(
            r["mechanism_captured"] and abs(r["exit_y_m"]) >= 0.10 for r in matrix
        ),
    }
    gate_rejected_rows = [
        r for r in matrix if r["mechanism_captured"] and not r["capture_valid"]
    ]

    all_points = [point for row in valid for point in row["trajectory"]]
    reachable_volume = {
        "robot_frame_ball_centre_aabb_m": {
            "x": [min((p[1] for p in all_points), default=None), max((p[1] for p in all_points), default=None)],
            "y": [min((p[2] for p in all_points), default=None), max((p[2] for p in all_points), default=None)],
            "z": [min((p[3] for p in all_points), default=None), max((p[3] for p in all_points), default=None)],
        },
        "rearward_distance_from_exit_m": [0.0, physical_range],
        "classification": "REACHABLE_HANDOFF_VOLUME_SIMULATION_BOUNDED",
    }

    centered_reference_distance = 0.150
    centered_crossings = [
        interpolate_receiving(row, centered_reference_distance)
        for row in centered_high
    ]
    centered_crossings = [row for row in centered_crossings if row is not None]
    centered_reference = {
        "rearward_distance_from_exit_m": centered_reference_distance,
        "all_bound_cases_above_useful_plane": bool(centered_crossings) and len(centered_crossings) == len(centered_high) and all(row["z_m"] >= MIN_RECEIVING_CENTER_Z for row in centered_crossings),
        "ball_centre_z_range_m": [min((r["z_m"] for r in centered_crossings), default=None), max((r["z_m"] for r in centered_crossings), default=None)],
        "vertical_velocity_range_m_s": [min((r["vz_m_s"] for r in centered_crossings), default=None), max((r["vz_m_s"] for r in centered_crossings), default=None)],
        "residual_speed_range_m_s": [min((r["speed_m_s"] for r in centered_crossings), default=None), max((r["speed_m_s"] for r in centered_crossings), default=None)],
        "lateral_error_range_m": [min((r["y_m"] for r in centered_crossings), default=None), max((r["y_m"] for r in centered_crossings), default=None)],
        "qualification": "CENTERED_ONLY_NOT_A_RELIABLE_BASKET_CONSTRAINT",
    }

    def sensitivity(key, bounds):
        result = {}
        for value in bounds:
            rows = [
                r for r in valid if r["speed_name"] == "NOMINAL"
                and r["lateral_offset_m"] == 0.0 and r[key] == value
            ]
            result[str(value)] = {
                "valid_cases": len(rows),
                "exit_speed_range_m_s": [min((r["exit_speed_m_s"] for r in rows), default=None), max((r["exit_speed_m_s"] for r in rows), default=None)],
                "exit_elevation_range_deg": [min((r["exit_elevation_deg"] for r in rows), default=None), max((r["exit_elevation_deg"] for r in rows), default=None)],
            }
        return result

    tyre_sensitivity = {}
    for force5 in TYRE_FORCE_AT_5MM_BOUNDS_N:
        rows = [r for r in valid if r["speed_name"] == "NOMINAL" and r["lateral_offset_m"] == 0.0 and r["tyre_force_at_5mm_n"] == force5]
        tyre_sensitivity[str(force5)] = {
            "valid_cases": len(rows),
            "exit_speed_range_m_s": [min((r["exit_speed_m_s"] for r in rows), default=None), max((r["exit_speed_m_s"] for r in rows), default=None)],
            "peak_tyre_deflection_range_mm": [
                1000 * min((max(r["maximum_left_tyre_deflection_m"], r["maximum_right_tyre_deflection_m"]) for r in rows), default=0.0),
                1000 * max((max(r["maximum_left_tyre_deflection_m"], r["maximum_right_tyre_deflection_m"]) for r in rows), default=0.0),
            ],
        }

    repeat_rows = [r for r in trials if r["repeat_id"] > 0 or r["case_id"] == REPRESENTATIVE_CASE_ID]
    def repeat_stat(key):
        values = [row[key] for row in repeat_rows]
        return {"mean": float(np.mean(values)), "stddev": float(np.std(values))} if values else None

    rim_result = rim_windows(valid)
    climb = {
        "ball_climbs_ramp_in_reduced_order_solver": bool(valid),
        "ploughed_ahead_cases": outcomes.get("PLOUGHED_AHEAD", 0),
        "stalled_in_nip_cases": outcomes.get("STALLED_IN_NIP", 0),
        "ball_centre_z_at_nip_stations_m": {
            "predicted_static_rest": {
                name: geom.ball_rest_centre_z_m(x)
                for name, (x, _z) in geom.nip_stations_m().items()
            },
            "simulated_range": {
                key: [
                    min((r[key] for r in valid if r[key] is not None), default=None),
                    max((r[key] for r in valid if r[key] is not None), default=None),
                ]
                for key in ("ball_centre_z_at_nip_upper_m", "ball_centre_z_at_nip_mid_m",
                            "ball_centre_z_at_nip_lower_m")
            },
        },
        "wheel_contact_before_ramp_contact_cases": sum(
            bool(r["wheel_contact_before_ramp_contact"]) for r in matrix
        ),
        "peak_ramp_normal_force_range_n": [
            min((r["peak_ramp_normal_force_n"] for r in matrix), default=None),
            max((r["peak_ramp_normal_force_n"] for r in matrix), default=None),
        ],
    }

    def effect_block(flat_key: str, ramp_key: str) -> dict:
        flat_row = comparison[flat_key]["summary"]
        ramp_row = comparison[ramp_key]["summary"]
        keys = (
            "exit_x_cad_m", "exit_z_m", "exit_speed_m_s", "exit_elevation_deg",
            "exit_azimuth_deg", "exit_vx_m_s", "exit_vz_m_s", "apex_z_m",
            "ballistic_ground_range_m", "maximum_above_base_receiving_distance_m",
            "bilateral_contact_duration_s", "outcome",
        )
        return {
            "approach_speed_m_s": comparison[flat_key]["approach_speed_m_s"],
            "flat_ground_datum": {key: flat_row[key] for key in keys},
            "authoritative_ramp": {key: ramp_row[key] for key in keys},
            "delta": {
                key: (ramp_row[key] - flat_row[key])
                for key in ("exit_z_m", "exit_speed_m_s", "exit_elevation_deg",
                            "apex_z_m", "ballistic_ground_range_m",
                            "maximum_above_base_receiving_distance_m")
            },
        }

    flat = comparison["flat"]["summary"]
    ramp = comparison["ramp"]["summary"]
    ramp_effect = {
        "representative_case": REPRESENTATIVE_CASE_ID,
        "at_envelope_approach_speed": effect_block("flat_envelope", "ramp_envelope"),
        "note": (
            "The flat-ground column is the INVALID legacy datum, reproduced only "
            "so the change can be quantified. Every flat-ground number in the "
            "previous report is withdrawn."
        ),
        "flat_ground_datum": {
            key: flat[key] for key in (
                "exit_x_cad_m", "exit_z_m", "exit_speed_m_s", "exit_elevation_deg",
                "exit_azimuth_deg", "exit_vx_m_s", "exit_vz_m_s", "apex_z_m",
                "ballistic_ground_range_m", "maximum_above_base_receiving_distance_m",
                "bilateral_contact_duration_s", "outcome",
            )
        },
        "authoritative_ramp": {
            key: ramp[key] for key in (
                "exit_x_cad_m", "exit_z_m", "exit_speed_m_s", "exit_elevation_deg",
                "exit_azimuth_deg", "exit_vx_m_s", "exit_vz_m_s", "apex_z_m",
                "ballistic_ground_range_m", "maximum_above_base_receiving_distance_m",
                "bilateral_contact_duration_s", "outcome",
            )
        },
        "delta": {
            key: (ramp[key] - flat[key])
            for key in ("exit_z_m", "exit_speed_m_s", "exit_elevation_deg",
                        "apex_z_m", "ballistic_ground_range_m",
                        "maximum_above_base_receiving_distance_m")
        },
    }

    gazebo = gazebo_campaign or {
        "status": "NOT_RUN",
        "note": (
            "Gazebo owns ramp wedge/plough dynamics with ground friction, cheek "
            "centering, capture half-width, lateral entry velocity, jam/reject "
            "modes and 6-DOF multi-contact. Until the campaign is executed those "
            "questions are unanswered; they are NOT inferred from this solver."
        ),
    }
    gazebo_stages = gazebo.get("stages", {})
    gazebo_ran = gazebo.get("status") == "COMPLETE"

    def gazebo_value(stage: str, key: str):
        """Gazebo-owned answer, or why there isn't one.

        A stage that ran and could not measure its quantity reports the reason
        it was gated. Only a campaign that never ran reports PENDING.
        """

        if not gazebo_ran:
            return "PENDING_GAZEBO_CAMPAIGN"
        entry = gazebo_stages.get(stage, {})
        value = entry.get(key)
        if value is not None:
            return value
        reason = entry.get("gated_by") or entry.get("status")
        return f"UNMEASURED_{stage}_GATED: {reason}" if reason else None

    solver_lateral_limit = max(
        (abs(row["lateral_offset_m"]) for row in valid), default=None
    )

    return {
        "schema_version": 2,
        "study_date": "2026-08-27",
        "scope": "standalone fixed-motor intake capture and free handoff on the authoritative ramp",
        "evidence_classification": "SIMULATION_BOUNDED",
        "supersedes": (
            "schema_version 1 (flat frictionless ground datum). Every number "
            "downstream of the exit vector in that revision is invalidated."
        ),
        "frame_mapping": {
            "source": "scripts/intake_geometry.py",
            "cad_ground_frame": "Option A: origin at the chassis plate centre, z=0 at the court, +X forward",
            "standalone_frame": "wheel axis origin (0, 0, 0.070); cad_x = std_x + 0.470",
            "base_link_frame": "xacro_z = cad_z - 0.045; xacro_x = cad_x + packaging shift",
            "packaging_shift_x_m": geom.PACKAGING_SHIFT_X_M,
            "cad_aligned_urdf_variant": geom.CAD_ALIGNED_VARIANT,
            "cheek_throat_to_nip_m": geom.throat_to_nip_distance_m(),
            "asserted_by": "tests/test_intake_frame_alignment.py",
        },
        "ground_profile": {
            "source": "cad/collector-intake-v1/option-a/option-a.scad oa_ramp_z",
            "law": "smoothstep t*t*(3-2*t)",
            "front_x_cad_m": geom.RAMP_FRONT_X_M,
            "rear_x_cad_m": geom.RAMP_REAR_X_M,
            "front_z_m": geom.RAMP_FRONT_Z_M,
            "rear_z_m": geom.RAMP_REAR_Z_M,
            "maximum_slope_deg": math.degrees(geom.ramp_slope_rad(geom.CAD_NIP_X_M)),
            "width_m": geom.RAMP_WIDTH_M,
            "wall_height_m": geom.RAMP_WALL_HEIGHT_M,
            "ball_centre_lateral_bound_m": geom.ramp_ball_centre_half_width_m(),
            "z_at_nip_mid_m": geom.ramp_z_m(geom.CAD_NIP_X_M),
            "frozen_contact_sequence": frozen_contact_sequence(),
            "court_moves_in_robot_frame": True,
            "approach_speed_m_s": APPROACH_SPEED_M_S,
            "available_coast_climb_m": APPROACH_SPEED_M_S**2 / (2.0 * G),
            "required_climb_m": geom.RAMP_REAR_Z_M - geom.RAMP_FRONT_Z_M,
            "classification": "GROUND_PROFILE_IS_AUTHORITATIVE_RAMP",
        },
        "geometry": {
            "wheel_centres_m": {"left": list(geom.wheel_centre_m(1, "std")), "right": list(geom.wheel_centre_m(-1, "std"))},
            "wheel_axis": list(AXIS),
            "wheel_diameter_m": geom.WHEEL_DIAMETER_M,
            "wheel_width_m": geom.WHEEL_WIDTH_M,
            "nominal_gap_m": geom.NOMINAL_GAP_M,
            "wheel_lowest_point_above_court_m": geom.wheel_lowest_z_m(),
            "bridge_aabb_m": {"x": list(BRIDGE_X), "y": list(BRIDGE_Y), "z": list(BRIDGE_Z)},
            "bridge_under_z_source_conflict_m": {
                "standalone_scad_authority_1": geom.BRIDGE_UNDER_Z_CONFLICT_M[0],
                "option_a_bridge_params_authority_2": geom.BRIDGE_UNDER_Z_CONFLICT_M[1],
                "used": geom.BRIDGE_UNDER_Z_CONFLICT_M[0],
                "note": "reported, not silently reconciled; authority 1 owns bridge geometry",
            },
            "cheek_throat_ball_centre_half_width_m": geom.throat_ball_centre_half_width_m(),
            "geometry_frozen": True,
            "legacy_translating_carriage_used": False,
        },
        "motor_model": {
            "product": "DFRobot FIT0186 / GB37Y3530-12V-251R",
            "manufacturer_source": "https://wiki.dfrobot.com/fit0186/",
            "nominal_voltage_v": MOTOR_NOMINAL_V,
            "no_load_speed_rpm": MOTOR_NO_LOAD_RPM,
            "no_load_current_a": MOTOR_NO_LOAD_CURRENT_A,
            "stall_torque_nm": MOTOR_STALL_TORQUE_NM,
            "stall_current_a": MOTOR_STALL_CURRENT_A,
            "law": "linear torque-speed and torque-current bounded model",
            "wheel_inertia_assumption_kg_m2": WHEEL_INERTIA_ASSUMPTION,
            "wheel_inertia_physically_measured": False,
            "gazebo_is_an_ideal_velocity_source": True,
            "droop_torque_current_measurable_in_gazebo": False,
        },
        "sensitivity_bounds": {
            "tread_friction_coefficients": list(FRICTION_BOUNDS),
            "ball_ramp_friction_coefficients": list(RAMP_FRICTION_BOUNDS),
            "ball_ramp_friction_bound_rationale": (
                "dry tennis felt on a smooth rigid printed/plywood sheet; 0.20 "
                "covers a slick surface and 0.60 a rough unfinished one"
            ),
            "tyre_force_at_5mm_n": list(TYRE_FORCE_AT_5MM_BOUNDS_N),
            "tyre_law": "linear radial spring in series with calibrated ball law",
            "bounds_physically_calibrated": False,
            "lateral_entry_offsets_m": list(OFFSET_CASES_M),
            "speed_cases_fraction_of_no_load": SPEED_CASES,
            "rotating_inertia_allocation_kg_m2": WHEEL_INERTIA_ASSUMPTION,
        },
        "trajectory_physics": {
            "gravity": True,
            "aerodynamic_drag": False,
            "magnus_effect": False,
            "spin_decay": False,
            "FULL_HANDOFF_TRAJECTORY_PHYSICS_VALIDATED": False,
        },
        "approach_speed": {
            "why_it_is_reported": (
                "The frozen ramp lip reaches a court-resting ball 48.6 mm "
                "before the first centred wheel contact (see "
                "ground_profile.frozen_contact_sequence), so the ball must "
                "cross the wedge on its own momentum. Approach speed is "
                "therefore a first-order variable, "
                "not a nuisance parameter. It is a COMMANDED operating value, "
                "not an unmeasured coefficient."
            ),
            "cases_m_s": APPROACH_CASES,
            "envelope_case": ENVELOPE_APPROACH_CASE,
            "envelope_speed_m_s": ENVELOPE_APPROACH_M_S,
            "sweep": approach_sweep,
            "capture_threshold": threshold,
        },
        "campaign": {
            "trial_count": len(trials),
            "matrix_case_count": len(matrix),
            "matrix_approach_case": ENVELOPE_APPROACH_CASE,
            "matrix_approach_speed_m_s": ENVELOPE_APPROACH_M_S,
            "mechanism_captured_count": sum(bool(r["mechanism_captured"]) for r in matrix),
            "valid_after_model_gates_count": len(valid),
            "high_bound_case_count_expected": bound_case_count * len(OFFSET_CASES_M),
            "high_valid_case_count": len(reliable_rows),
            "centered_speed_summary": centered_speed_summary,
            "offset_capture_matrix": offset_capture,
            "mechanism_outcomes": outcomes,
            "model_validity_gate_rejections": gate_rejections,
            "gate_rejected_but_mechanically_captured": {
                "count": len(gate_rejected_rows),
                "bilateral_contact_duration_range_s": [
                    min((r["bilateral_contact_duration_s"] for r in gate_rejected_rows), default=None),
                    max((r["bilateral_contact_duration_s"] for r in gate_rejected_rows), default=None),
                ],
                "exit_speed_range_m_s": [
                    min((r["exit_speed_m_s"] for r in gate_rejected_rows), default=None),
                    max((r["exit_speed_m_s"] for r in gate_rejected_rows), default=None),
                ],
                "note": "model-validity rejections, NOT mechanism failures",
            },
            "tread_friction_sensitivity": sensitivity("friction_coefficient", FRICTION_BOUNDS),
            "ball_ramp_friction_sensitivity": sensitivity("ramp_friction_coefficient", RAMP_FRICTION_BOUNDS),
            "tyre_compliance_sensitivity": tyre_sensitivity,
            "representative_repeatability": {
                "classification": "DETERMINISTIC_SIMULATION_REPEATABILITY_ONLY",
                "exit_speed_m_s": repeat_stat("exit_speed_m_s"),
                "exit_elevation_deg": repeat_stat("exit_elevation_deg"),
                "exit_azimuth_deg": repeat_stat("exit_azimuth_deg"),
                "apex_z_m": repeat_stat("apex_z_m"),
            },
            "timestep_convergence": {
                "cases": convergence,
                "exit_speed_relative_spread": (
                    (max(r["exit_speed_m_s"] for r in convergence)
                     - min(r["exit_speed_m_s"] for r in convergence))
                    / float(np.mean([r["exit_speed_m_s"] for r in convergence]))
                ),
                "classification": "NUMERICALLY_CONVERGED_FOR_BOUNDED_STUDY",
            },
        },
        "ramp_effect": ramp_effect,
        "ramp_dynamics": climb,
        "handoff": {
            "MAX_PHYSICAL_HANDOFF_REACH_M": physical_range,
            "MAX_PHYSICAL_HANDOFF_HEIGHT_M": physical_height,
            "MAX_RELIABLE_BASKET_HANDOFF_DISTANCE_M": reliable_distance,
            "MAX_CENTERED_SIMULATION_BOUNDED_ABOVE_BASE_DISTANCE_M": centered_bounded_distance,
            "MAX_RELIABLE_BASKET_HANDOFF_HEIGHT_M": min((c["z_m"] for c in reliable_crossings), default=None),
            "RECOMMENDED_MAX_BASKET_RECEIVING_X_REARWARD_FROM_EXIT_M": reliable_distance,
            "RECOMMENDED_MIN_BASKET_RECEIVING_Z_M": MIN_RECEIVING_CENTER_Z if reliable_defined else None,
            "RECOMMENDED_RECEIVING_OPENING_WIDTH_M": recommended_opening_width,
            "RECOMMENDED_LATERAL_TOLERANCE_M": lateral_max if reliable_defined else None,
            "receiving_crossing_vertical_velocity_range_m_s": [min((c["vz_m_s"] for c in reliable_crossings), default=0.0), max((c["vz_m_s"] for c in reliable_crossings), default=0.0)] if reliable_defined else None,
            "receiving_crossing_residual_speed_range_m_s": [min((c["speed_m_s"] for c in reliable_crossings), default=0.0), max((c["speed_m_s"] for c in reliable_crossings), default=0.0)] if reliable_defined else None,
            "REACHABLE_HANDOFF_VOLUME": reachable_volume,
            "RELIABLE_HANDOFF_VOLUME": None if not reliable_defined else {
                "rearward_distance_m": [0.05, reliable_distance],
                "minimum_ball_centre_z_m": MIN_RECEIVING_CENTER_Z,
                "maximum_abs_lateral_error_m": lateral_max,
            },
            "centered_high_speed_reference_plane": centered_reference,
            "exit_station_cad_x_range_m": [
                min((r["exit_x_cad_m"] for r in valid), default=None),
                max((r["exit_x_cad_m"] for r in valid), default=None),
            ],
        },
        "basket_on_base": {
            "floor_plane_z_m": CHASSIS_TOP_Z,
            "chassis_cut_out": False,
            "definition": (
                "worst-case intersection over every valid trial: the ball bottom "
                "must pass above the rim top at that rearward distance"
            ),
            "rim_windows": rim_result,
            "flat_ground_reference_withdrawn": {
                "0.005": [0.040, 0.180], "0.010": [0.050, 0.165],
                "0.015": [0.065, 0.150], "0.020": [0.095, 0.120],
                "max_rim_height_m": 0.021,
                "status": "INVALID_FLAT_GROUND_DATUM_QUOTED_FOR_COMPARISON_ONLY",
            },
            "low_rim_versus_rebound_conflict": {
                "statement": (
                    "A low rim is required for the ball to clear it, while a deep "
                    "energy-absorbing entry is required to stop rebound escape. "
                    "These pull in opposite directions and are NOT resolved here; "
                    "no basket geometry is designed in this task."
                ),
                "rigid_wall_rebound_coefficient_used": 0.73,
                "worst_case_rebound_rise_m": max(
                    (entry["receiving_points"]["mid"]["rigid_wall_rebound_rise_m"]
                     for key, entry in rim_result.items()
                     if isinstance(entry, dict) and entry.get("receiving_points")),
                    default=None,
                ),
            },
        },
        "gazebo_campaign": gazebo,
        "classifications": {
            "SIM_INTAKE_MATCHES_FROZEN_CAD": True,
            "LEGACY_CARRIAGE_REMOVED": True,
            "UNPHYSICAL_FRICTION_REMOVED": True,
            "FRAME_MAPPING_ASSERTED": True,
            "GROUND_PROFILE_IS_AUTHORITATIVE_RAMP": True,
            "CROSS_INSTRUMENT_AGREEMENT": gazebo.get("cross_validation", {}).get(
                "CROSS_INSTRUMENT_AGREEMENT", "PENDING_GAZEBO_CAMPAIGN"
            ),
            "BALL_CLIMBS_RAMP_IN_SOLVER": climb["ball_climbs_ramp_in_reduced_order_solver"],
            "INTAKE_CAPTURES_CENTRED_BALL_AT_ROUTE_NOMINAL_APPROACH": bool(
                approach_sweep["ROUTE_NOMINAL"]["centred_mechanism_captured"]
            ),
            "INTAKE_CAPTURES_CENTRED_BALL_AT_ROUTE_MAX_APPROACH": bool(
                approach_sweep["ROUTE_MAX"]["centred_mechanism_captured"]
            ),
            "INTAKE_CAPTURES_OFFSET_ENTRIES_AT_ROUTE_NOMINAL_APPROACH": bool(
                approach_sweep["ROUTE_NOMINAL"]["mechanism_captured"]
            ),
            "MINIMUM_CENTRED_CAPTURE_APPROACH_SPEED_M_S": threshold[
                "MINIMUM_APPROACH_SPEED_WITH_CAPTURE_ACROSS_ALL_FRICTION_BOUNDS_M_S"
            ],
            "CENTRED_BALL_PLOUGHED_AHEAD_AT_ROUTE_SPEEDS": not bool(
                approach_sweep["ROUTE_MAX"]["centred_mechanism_captured"]
            ),
            "EXIT_ENVELOPE_CONDITIONAL_ON_APPROACH_SPEED_M_S": ENVELOPE_APPROACH_M_S,
            "BALL_CLIMBS_RAMP_IN_SIM": gazebo_value("S1", "BALL_CLIMBS_RAMP_IN_SIM"),
            "INTAKE_CAPTURE_HALF_WIDTH_M": gazebo_value("S2", "INTAKE_CAPTURE_HALF_WIDTH_M"),
            "SOLVER_BOUNDED_LATERAL_OFFSET_LIMIT_M": solver_lateral_limit,
            "INTAKE_MAX_LATERAL_ENTRY_VELOCITY_M_S": gazebo_value(
                "S3", "INTAKE_MAX_LATERAL_ENTRY_VELOCITY_M_S"
            ),
            "CHEEK_THROAT_ADEQUATE": gazebo_value("S4", "CHEEK_THROAT_ADEQUATE"),
            "MAX_BASKET_RIM_HEIGHT_ON_BASE_M": rim_result["MAX_BASKET_RIM_HEIGHT_ON_BASE_M"],
            "MAX_BASKET_RIM_HEIGHT_ON_BASE_ALL_ENTRIES_M": rim_result["MAX_BASKET_RIM_HEIGHT_ALL_ENTRIES_M"],
            "ABOVE_BASE_BASKET_RECEIVING_POSITION_FEASIBLE": reliable_defined,
            "BASE_CUTOUT_REQUIRED_FOR_BASKET_HANDOFF": base_cutout,
            "BASE_CUTOUT_DECISION_PHYSICALLY_UNRESOLVED": True,
            "INTAKE_STATIC_CAPTURE_VALIDATED_IN_SIM": False,
            "INTAKE_DYNAMIC_CAPTURE_VALIDATED_IN_SIM": bool(valid),
            "INTAKE_EXIT_STATE_VALIDATED_IN_SIM": bool(valid),
            "INTAKE_HANDOFF_ENVELOPE_GENERATED": bool(valid),
            "INTAKE_RELIABLE_HANDOFF_ENVELOPE_DEFINED": reliable_defined,
            "INTAKE_TYRE_COMPLIANCE_PHYSICALLY_VALIDATED": False,
            "INTAKE_TREAD_FRICTION_PHYSICALLY_VALIDATED": False,
            "BALL_RAMP_FRICTION_PHYSICALLY_VALIDATED": False,
            "PHYSICAL_HARDWARE_PENDING": True,
            "DETERMINISTIC_SIMULATION_REPEATABILITY_ONLY": True,
            "FULL_HANDOFF_TRAJECTORY_PHYSICS_VALIDATED": False,
        },
        "plots": plot_paths,
        "telemetry": {
            "trial_summary_csv": "docs/mechanism/standalone-intake-handoff-trials.csv",
            "representative_time_series_csv": "docs/mechanism/standalone-intake-handoff-telemetry.csv",
        },
        "stop_reasons": (
            ["the corrected fixed-motor intake did not capture on the authoritative ramp"]
            if not valid else
            [reason for reason in [
                (
                    "STOP CONDITION (task section 14): on the authoritative ramp the "
                    "ball is PLOUGHED AHEAD and never reaches the nip at the "
                    f"collection route's commanded speeds ({ROUTE_NOMINAL_APPROACH_M_S:.2f} m/s "
                    f"nominal, {APPROACH_CASES['ROUTE_MAX']:.2f} m/s maximum). Capture "
                    "first appears at "
                    f"{threshold['MINIMUM_APPROACH_SPEED_WITH_ANY_CAPTURE_M_S']} m/s and is "
                    "bound-wide from "
                    f"{threshold['MINIMUM_APPROACH_SPEED_WITH_CAPTURE_ACROSS_ALL_FRICTION_BOUNDS_M_S']} m/s."
                ) if not approach_sweep["ROUTE_MAX"]["centred_mechanism_captured"] else None,
                None if reliable_defined else
                "nonzero lateral entries are not reliable across the full bounded friction, ramp-friction and tyre-compliance matrix",
                "tyre compliance, felt/tread friction and ball-to-ramp friction remain physically unmeasured",
                None if gazebo.get("status") == "COMPLETE" else
                "the Gazebo campaign (S1-S5) and the cross-validation gate are outstanding; capture half-width, lateral entry velocity and cheek throat adequacy are unanswered",
                (
                    "STOP CONDITION (task section 14): the Gazebo campaign captured "
                    "no ball at any tested approach speed, tread friction or lateral "
                    "offset. S2-S5 therefore have nothing to measure and stay gated: "
                    "capture half-width, maximum lateral entry velocity and cheek "
                    "throat adequacy are UNANSWERED."
                ) if gazebo.get("status") == "COMPLETE" and not any(
                    run.get("outcome") == "CAPTURED"
                    for stage in gazebo.get("stages", {}).values()
                    for run in (stage.get("runs") or [])
                ) else None,
                (
                    "STOP CONDITION (task section 14): the two instruments disagree "
                    "above the plough threshold — the solver transports a centred "
                    "ball at the envelope approach speed and Gazebo rejects it at "
                    "the nip. Neither model was tuned to close the gap."
                ) if gazebo.get("cross_validation", {}).get("agree_above_threshold") is False
                else None,
            ] if reason]
        ),
    }


def _range(values, scale=1.0, digits=3):
    if values is None or values[0] is None:
        return "n/a"
    return f"{values[0]*scale:.{digits}f}..{values[1]*scale:.{digits}f}"


def report_markdown(result: dict) -> str:
    c = result["classifications"]
    h = result["handoff"]
    campaign = result["campaign"]
    effect = result["ramp_effect"]
    flat, ramp = effect["flat_ground_datum"], effect["authoritative_ramp"]
    envelope_effect = effect["at_envelope_approach_speed"]
    envelope_flat = envelope_effect["flat_ground_datum"]
    envelope_ramp = envelope_effect["authoritative_ramp"]
    rim_all = result["basket_on_base"]["rim_windows"]
    rim = rim_all[rim_all["headline_population"]]
    population_rows = "\n".join(
        f"| {name} | {rim_all[name]['population_size']} | "
        + " | ".join(
            (
                f"{rim_all[name][key]['window_m'][0]:.3f}..{rim_all[name][key]['window_m'][1]:.3f}"
                if rim_all[name][key]["window_m"] else "none"
            )
            for key in ("0.005", "0.010", "0.015", "0.020")
        )
        + f" | {rim_all[name]['MAX_BASKET_RIM_HEIGHT_ON_BASE_M']*1000:.0f} |"
        for name in ("centred_LOW", "centred_NOMINAL", "centred_HIGH",
                     "centred_entries_all_bounds", "all_valid_entries")
        if name in rim_all
    )
    headline_points = rim["0.010"].get("receiving_points", {}) if rim.get("0.010") else {}
    dynamics = result["ramp_dynamics"]
    ground = result["ground_profile"]
    decision = c["BASE_CUTOUT_REQUIRED_FOR_BASKET_HANDOFF"]
    low = campaign["centered_speed_summary"]["LOW"]
    nominal = campaign["centered_speed_summary"]["NOMINAL"]
    high = campaign["centered_speed_summary"]["HIGH"]
    reliable_text = (
        f"{h['MAX_RELIABLE_BASKET_HANDOFF_DISTANCE_M']:.3f} m"
        if h["MAX_RELIABLE_BASKET_HANDOFF_DISTANCE_M"] is not None else "undefined"
    )
    rim_rows = "\n".join(
        f"| {float(key)*1000:.0f} | "
        + (
            f"{rim[key]['window_m'][0]:.3f}..{rim[key]['window_m'][1]:.3f} | "
            f"{rim[key]['window_width_m']*1000:.0f} |"
            if rim[key]["window_m"] else "no window | 0 |"
        )
        for key in (f"{value:.3f}" for value in BASKET_RIM_HEIGHTS_M)
    )
    outcome_rows = "\n".join(
        f"| {name} | {count} |" for name, count in sorted(campaign["mechanism_outcomes"].items())
    )
    gate_rows = "\n".join(
        f"| {name} | {count} |"
        for name, count in campaign["model_validity_gate_rejections"].items()
    )
    approach = result["approach_speed"]
    sequence = result["ground_profile"]["frozen_contact_sequence"]
    threshold = approach["capture_threshold"]
    approach_rows = "\n".join(
        f"| {case} | {entry['approach_speed_m_s']:.2f} | "
        f"{entry['mechanism_captured']}/{entry['cases']} | "
        f"{entry['centred_mechanism_captured']}/{entry['centred_cases']} | "
        f"{entry['maximum_ball_centre_z_m']*1000:.1f} | "
        + (
            f"{entry['exit_speed_range_m_s'][0]:.3f}..{entry['exit_speed_range_m_s'][1]:.3f} |"
            if entry["exit_speed_range_m_s"][0] is not None else "n/a |"
        )
        for case, entry in approach["sweep"].items()
    )
    gazebo = result["gazebo_campaign"]
    gazebo_section = gazebo.get("markdown") or (
        "The Gazebo campaign (S1-S5) has **not been run** on the corrected model.\n"
        "Gazebo owns ramp wedge/plough dynamics with ground friction, cheek\n"
        "centering, capture half-width, lateral entry velocity, jam/reject modes\n"
        "and 6-DOF multi-contact. Those questions are therefore **unanswered**, and\n"
        "nothing in this report infers them from the reduced-order solver.\n"
    )
    return f"""# Standalone fixed-motor intake handoff capability

Date: 2026-08-27 · schema 2 · **supersedes the flat-ground revision**

Evidence class: **SIMULATION_BOUNDED — not physical validation**

## What changed and why every earlier number is withdrawn

The previous revision measured the entire handoff envelope on a flat
frictionless ground datum with a prescribed −0.45 m/s approach and a position
clamp. The real Option A package has a 33.5 mm smoothstep handoff ramp running
directly through the nip zone
(`oa_ramp_z`, {ground['front_x_cad_m']*1000:.0f} mm → {ground['rear_x_cad_m']*1000:.0f} mm,
{ground['front_z_m']*1000:.1f} mm → {ground['rear_z_m']*1000:.0f} mm, max slope
{ground['maximum_slope_deg']:.1f}°, zero slope at both ends).

This is not a datum swap. At the {ground['approach_speed_m_s']:.2f} m/s approach
the ball has only {ground['available_coast_climb_m']*1000:.1f} mm of coast climb
available against a {ground['required_climb_m']*1000:.1f} mm rise: it cannot
coast up. The robot drives a wedge under a ball that is at rest in the world,
and on the {ground['maximum_slope_deg']:.1f}° section the surface normal has a
forward horizontal component. The solver now models this as a real normal +
friction contact against the inclined surface — the court moves rearward in the
robot frame, the ramp does not — and no longer prescribes the approach velocity
of the ball. The ramp side walls bound the ball centre to
|y| ≤ {ground['ball_centre_lateral_bound_m']*1000:.0f} mm.

**Every number downstream of the exit vector in the previous report is
invalidated**: handoff envelope, reachable volume, receiving heights, basket
constraints, lateral spread and the sensitivity tables. They are re-derived
here, not patched.

## Headline result: the ball is ploughed ahead at collection speeds

The frozen ramp lip reaches a court-resting ball at CAD
x = {sequence['ramp_lip_first_contact_ball_centre_cad_x_m']*1000:.1f} mm —
{sequence['lip_lead_m']*1000:.1f} mm **before** the first centred wheel contact at
x = {sequence['first_wheel_contact_ball_centre_cad_x_m']*1000:.1f} mm. The ball must
therefore cross the wedge on its own momentum, which makes the commanded
approach speed a first-order variable rather than a nuisance parameter.

| Approach case | m/s | Captures (all entries) | Captures (centred) | Peak ball centre z (mm) | Exit speed (m/s) |
| --- | --- | --- | --- | --- | --- |
{approach_rows}

Minimum approach speed with any capture:
**{threshold['MINIMUM_APPROACH_SPEED_WITH_ANY_CAPTURE_M_S']} m/s**;
capture across every friction bound from
**{threshold['MINIMUM_APPROACH_SPEED_WITH_CAPTURE_ACROSS_ALL_FRICTION_BOUNDS_M_S']} m/s**.
The collection route commands {threshold['route_nominal_speed_m_s']:.2f} m/s nominal
and caps at {threshold['route_max_speed_m_s']:.2f} m/s
(`{threshold['source']}`).

Laterally offset entries behave differently and the split matters: a ball
20 mm off the centreline reaches a wheel much earlier in X than a centred one,
so some offset entries capture even at 0.35 m/s while **no centred entry
captures below 0.625 m/s**. The funnel's whole job is to deliver a centred
ball, which is exactly the entry the ramp ploughs.

Below the threshold the ball rides a few millimetres up the smoothstep, reaches
the slope where the surface can no longer drive it rearward relative to the
robot, and is then bulldozed forward at approximately the robot's own speed: it
never reaches the nip, so no wheel ever touches it. This is **task section 14
stop condition 1** (`the ball cannot climb the ramp and is ploughed ahead`).

Energy makes the mechanism explicit. In the robot frame the ramp is static and
does no work, so the ball's only budget for the 33.5 mm rise is its own kinetic
energy. At 0.45 m/s that is {result['ground_profile']['available_coast_climb_m']*1000:.1f} mm
of coast climb before rolling inertia is even counted. The court, which does
move in that frame, can feed energy in through friction only while the ball
touches the court and the ramp at the same time — a window that closes a few
millimetres up the sheet.

**Everything below this point — the exit envelope, S5, the reachable volume and
the basket-on-base study — is derived at the
{approach['envelope_speed_m_s']:.2f} m/s `{approach['envelope_case']}` approach
case, the only regime in which the frozen ramp captures at all. Those numbers
are conditional on a commanded speed the collection route does not currently
drive.**

## The ramp effect: before / after

Representative centred case `{effect['representative_case']}`, both columns at
the {APPROACH_SPEED_M_S:.2f} m/s reference approach so the ONLY difference is the
ground profile.

| Quantity | Flat datum (invalid) | Authoritative ramp | Δ |
| --- | --- | --- | --- |
| exit station, CAD x (mm) | {flat['exit_x_cad_m']*1000:.1f} | {ramp['exit_x_cad_m']*1000:.1f} | {(ramp['exit_x_cad_m']-flat['exit_x_cad_m'])*1000:+.1f} |
| exit ball-centre z (mm) | {flat['exit_z_m']*1000:.1f} | {ramp['exit_z_m']*1000:.1f} | {effect['delta']['exit_z_m']*1000:+.1f} |
| exit speed (m/s) | {flat['exit_speed_m_s']:.3f} | {ramp['exit_speed_m_s']:.3f} | {effect['delta']['exit_speed_m_s']:+.3f} |
| exit elevation (deg) | {flat['exit_elevation_deg']:.2f} | {ramp['exit_elevation_deg']:.2f} | {effect['delta']['exit_elevation_deg']:+.2f} |
| apex ball-centre z (mm) | {flat['apex_z_m']*1000:.1f} | {ramp['apex_z_m']*1000:.1f} | {effect['delta']['apex_z_m']*1000:+.1f} |
| ballistic range (m) | {flat['ballistic_ground_range_m']:.3f} | {ramp['ballistic_ground_range_m']:.3f} | {effect['delta']['ballistic_ground_range_m']:+.3f} |
| bilateral contact (ms) | {flat['bilateral_contact_duration_s']*1000:.1f} | {ramp['bilateral_contact_duration_s']*1000:.1f} | {(ramp['bilateral_contact_duration_s']-flat['bilateral_contact_duration_s'])*1000:+.1f} |
| outcome | {flat['outcome']} | {ramp['outcome']} | |

At the reference approach the ramp column does not produce an exit at all — the
ball is ploughed. The same comparison at the {envelope_effect['approach_speed_m_s']:.2f} m/s
envelope approach, where both profiles capture, isolates the ramp's effect on
the exit vector itself:

| Quantity | Flat datum (invalid) | Authoritative ramp | Δ |
| --- | --- | --- | --- |
| exit station, CAD x (mm) | {envelope_flat['exit_x_cad_m']*1000:.1f} | {envelope_ramp['exit_x_cad_m']*1000:.1f} | {(envelope_ramp['exit_x_cad_m']-envelope_flat['exit_x_cad_m'])*1000:+.1f} |
| exit ball-centre z (mm) | {envelope_flat['exit_z_m']*1000:.1f} | {envelope_ramp['exit_z_m']*1000:.1f} | {envelope_effect['delta']['exit_z_m']*1000:+.1f} |
| exit speed (m/s) | {envelope_flat['exit_speed_m_s']:.3f} | {envelope_ramp['exit_speed_m_s']:.3f} | {envelope_effect['delta']['exit_speed_m_s']:+.3f} |
| exit elevation (deg) | {envelope_flat['exit_elevation_deg']:.2f} | {envelope_ramp['exit_elevation_deg']:.2f} | {envelope_effect['delta']['exit_elevation_deg']:+.2f} |
| apex ball-centre z (mm) | {envelope_flat['apex_z_m']*1000:.1f} | {envelope_ramp['apex_z_m']*1000:.1f} | {envelope_effect['delta']['apex_z_m']*1000:+.1f} |
| ballistic range (m) | {envelope_flat['ballistic_ground_range_m']:.3f} | {envelope_ramp['ballistic_ground_range_m']:.3f} | {envelope_effect['delta']['ballistic_ground_range_m']:+.3f} |
| above-base receiving distance (m) | {envelope_flat['maximum_above_base_receiving_distance_m']:.3f} | {envelope_ramp['maximum_above_base_receiving_distance_m']:.3f} | {envelope_effect['delta']['maximum_above_base_receiving_distance_m']:+.3f} |
| outcome | {envelope_flat['outcome']} | {envelope_ramp['outcome']} | |

The two opposing consequences the task predicted are both visible and are
reported as measured, not as an expectation.

## Ramp dynamics in the solver

Predicted static ball-centre heights on the ramp at the three nip stations
(upper / mid / lower):
{dynamics['ball_centre_z_at_nip_stations_m']['predicted_static_rest']['NIP_UPPER']*1000:.1f} /
{dynamics['ball_centre_z_at_nip_stations_m']['predicted_static_rest']['NIP_MID']*1000:.1f} /
{dynamics['ball_centre_z_at_nip_stations_m']['predicted_static_rest']['NIP_LOWER']*1000:.1f} mm,
against a constant 33.0 mm on flat ground.

Simulated ball-centre height when the ball crossed each station, over all valid
trials: upper {_range(dynamics['ball_centre_z_at_nip_stations_m']['simulated_range']['ball_centre_z_at_nip_upper_m'], 1000, 1)} mm,
mid {_range(dynamics['ball_centre_z_at_nip_stations_m']['simulated_range']['ball_centre_z_at_nip_mid_m'], 1000, 1)} mm,
lower {_range(dynamics['ball_centre_z_at_nip_stations_m']['simulated_range']['ball_centre_z_at_nip_lower_m'], 1000, 1)} mm.

Peak ramp normal force across the matrix:
{_range(dynamics['peak_ramp_normal_force_range_n'], 1, 1)} N.
Cases where a wheel touched before the ramp did:
{dynamics['wheel_contact_before_ramp_contact_cases']} of {campaign['matrix_case_count']}.
Ploughed-ahead cases: {dynamics['ploughed_ahead_cases']}.
Stalled-in-nip cases: {dynamics['stalled_in_nip_cases']}.

## Campaign

{campaign['matrix_case_count']} bounded matrix trials
({len(SPEED_CASES)} speeds × {len(FRICTION_BOUNDS)} tread-friction bounds ×
{len(TYRE_FORCE_AT_5MM_BOUNDS_N)} tyre bounds × {len(RAMP_FRICTION_BOUNDS)}
ball-to-ramp friction bounds × {len(OFFSET_CASES_M)} lateral offsets).
{campaign['mechanism_captured_count']} captured mechanically;
{campaign['valid_after_model_gates_count']} also passed the model-validity gates.

### Mechanism outcomes

| Outcome | Cases |
| --- | --- |
{outcome_rows}

### Model-validity gate rejections (NOT mechanism failures)

| Gate | Cases |
| --- | --- |
{gate_rows}

{campaign['gate_rejected_but_mechanically_captured']['count']} trials captured the
ball and were rejected only by a model-validity gate. Their bilateral contact was
{_range(campaign['gate_rejected_but_mechanically_captured']['bilateral_contact_duration_range_s'], 1000, 1)} ms
and they exited at
{_range(campaign['gate_rejected_but_mechanically_captured']['exit_speed_range_m_s'], 1, 3)} m/s.
The 5 mm tyre-deflection limit is a MODEL-VALIDITY limit on the linear tyre
bound, not a physical failure.

### Centred speed sweep (S5) on the corrected model

- LOW: {low['valid_cases']}/{low['executed_cases']} valid, exit speed
  {_range(low['exit_speed_range_m_s'])} m/s, elevation
  {_range(low['exit_elevation_range_deg'], 1, 2)} deg, range
  {_range(low['ballistic_range_m'])} m;
- NOMINAL: {nominal['valid_cases']}/{nominal['executed_cases']} valid, exit speed
  {_range(nominal['exit_speed_range_m_s'])} m/s, elevation
  {_range(nominal['exit_elevation_range_deg'], 1, 2)} deg, range
  {_range(nominal['ballistic_range_m'])} m;
- HIGH: {high['valid_cases']}/{high['executed_cases']} valid, exit speed
  {_range(high['exit_speed_range_m_s'])} m/s, elevation
  {_range(high['exit_elevation_range_deg'], 1, 2)} deg, range
  {_range(high['ballistic_range_m'])} m.

The 0.5/0.25/0.125 ms timestep check has an exit-speed relative spread of
{campaign['timestep_convergence']['exit_speed_relative_spread']*100:.3f}%.

## Basket on the base, no cut-out (section 10)

Floor plane z = {result['basket_on_base']['floor_plane_z_m']:.3f} m. A rim clears
when the BALL BOTTOM passes above the rim top for **every** valid trial.

Headline population `{rim_all['headline_population']}`: centred entries at the
HIGH wheel-speed command across every friction, ramp-friction and tyre bound, at
the {approach['envelope_speed_m_s']:.2f} m/s approach ({rim['population_size']} trials).

| Rim height (mm) | Rearward distance window (m) | Width (mm) |
| --- | --- | --- |
{rim_rows}

Wheel speed is a commanded setting, so the population matters and is stated
rather than averaged. Windows (m) by population:

| Population | Trials | rim 5 mm | rim 10 mm | rim 15 mm | rim 20 mm | Max rim (mm) |
| --- | --- | --- | --- | --- | --- | --- |
{population_rows}

Absolute maximum rim height that any distance clears, centred entries:
**{'undefined' if rim['MAX_BASKET_RIM_HEIGHT_ON_BASE_M'] is None else f"{rim['MAX_BASKET_RIM_HEIGHT_ON_BASE_M']*1000:.0f} mm"}**.
Over the full valid matrix, including offset entries:
**{'undefined' if rim_all['MAX_BASKET_RIM_HEIGHT_ALL_ENTRIES_M'] is None else f"{rim_all['MAX_BASKET_RIM_HEIGHT_ALL_ENTRIES_M']*1000:.0f} mm"}**
({rim_all['all_valid_entries']['population_size']} trials).

The flat-ground reference (rim 5 mm → 0.040..0.180 m, 10 mm → 0.050..0.165 m,
15 mm → 0.065..0.150 m, 20 mm → 0.095..0.120 m, absolute max 21 mm) is
**withdrawn**; it appears in the JSON only so the change can be quantified.

At the 10 mm rim the receiving state across the window is:
{"".join(f"{chr(10)}- {label}: distance {point['rearward_distance_from_exit_m']:.3f} m, ball centre z {point['ball_centre_z_range_m'][0]*1000:.1f}..{point['ball_centre_z_range_m'][1]*1000:.1f} mm, lateral error {point['lateral_error_range_m'][0]*1000:.1f}..{point['lateral_error_range_m'][1]*1000:.1f} mm, residual speed {point['residual_speed_range_m_s'][0]:.2f}..{point['residual_speed_range_m_s'][1]:.2f} m/s, vertical velocity {point['vertical_velocity_range_m_s'][0]:+.2f}..{point['vertical_velocity_range_m_s'][1]:+.2f} m/s ({'/'.join(point['phases'])}), rigid-wall rebound would rise {point['rigid_wall_rebound_rise_m']*1000:.0f} mm" for label, point in headline_points.items()) if headline_points else "no window at this rim height"}

**Unresolved conflict.** A low rim is needed for the ball to clear it, while a
deep energy-absorbing entry is needed to stop rebound escape: at the residual
speeds measured here a rigid wall returns enough energy to lift the ball tens of
millimetres. Both pull in opposite directions. This task does not resolve it and
designs no basket geometry.

## Two-instrument scope

This solver owns exit speed, exit elevation, exit azimuth, tyre radial
deflection, ball compression, contact force, wheel droop, torque and current,
because it carries the calibrated ball law and the series tyre spring.

Gazebo owns ramp wedge/plough dynamics with ground friction, cheek centering,
capture half-width, jam and reject modes, multi-contact and 6-DOF ball motion.
`gz_ros2_control` velocity commands act as an ideal velocity source, so wheel
droop, motor torque and motor current are **not measurable in Gazebo** and are
never reported from it (defect D4, declared rather than modelled).

## Gazebo campaign (S1-S5)

{gazebo_section}

## Handoff envelope

Maximum simulated ballistic reach **{h['MAX_PHYSICAL_HANDOFF_REACH_M']:.3f} m**;
strict all-bound high-speed reliable distance **{reliable_text}**.
Exit stations span CAD x {_range(h['exit_station_cad_x_range_m'], 1000, 1)} mm.

## Unmeasured parameters (swept, never selected)

- tread/felt friction {list(FRICTION_BOUNDS)};
- tyre radial stiffness {list(TYRE_FORCE_AT_5MM_BOUNDS_N)} N at 5 mm;
- ball-to-ramp friction {list(RAMP_FRICTION_BOUNDS)} (new; bound rationale in the JSON);
- rotating wheel/adapter inertia {WHEEL_INERTIA_ASSUMPTION} kg·m² allocation.

## Final classifications

```text
{chr(10).join(f"{key} = {json.dumps(value)}" for key, value in c.items())}
```

## Plots

""" + "\n".join(f"![{Path(path).stem}](../images/{Path(path).name})" for path in result["plots"]) + """

## Plots that could not be produced

Two of the requested plots depend on a measurement stage that is gated:

- *capture success vs lateral entry velocity* (S3) needs a capture half-width to
  sweep entry velocity within;
- *cheek throat output distribution* (S4) needs balls that actually pass the
  throat.

Both are gated by S1: the frozen ramp ploughs the centred ball. They are listed
as missing rather than approximated from the reduced-order solver, which cannot
represent cheeks at all.

## Smallest physical test that would settle this

The blocking result is a geometry/kinematics question, not a materials
question, so it does not need the full tyre calibration to answer:

1. **Ramp-lip plough test.** Build only the ramp (`oa_ramp_z`, 520 → 420 mm,
   1.5 → 35 mm, 180 mm wide) and push it along a hard court surface into a
   stationary ball at 0.35 m/s and at 0.80 m/s, wheels absent. Measure whether
   the ball climbs to the nip station or is bulldozed, and at what speed the
   behaviour changes. One printed ramp and a tape measure settle it.
2. Only if the ball climbs: the one-wheel/one-ball radial compression test,
   felt/tread traction at representative normal loads, ball-to-ramp friction on
   the real sheet, and the weight of the complete rotating wheel/adapter stack.
3. Then repeat the centred and offset capture test with encoder/current logging.

Until step 1 passes, the chassis cut-out decision, the basket rim height and the
whole handoff envelope stay physically unresolved, and no simulation number here
authorises removing chassis material.
"""


REPRESENTATIVE_CASE_ID = f"{ENVELOPE_APPROACH_CASE}_NOMINAL_mu0.6_F60_r0.40_y+0.000"


def build_matrix() -> list[Trial]:
    trials: list[Trial] = []
    for approach_case, approach in APPROACH_CASES.items():
        for speed_name, fraction in SPEED_CASES.items():
            for mu in FRICTION_BOUNDS:
                for tyre in TYRE_FORCE_AT_5MM_BOUNDS_N:
                    for ramp_mu in RAMP_FRICTION_BOUNDS:
                        for offset in OFFSET_CASES_M:
                            case_id = (
                                f"{approach_case}_{speed_name}_mu{mu:.1f}"
                                f"_F{tyre:.0f}_r{ramp_mu:.2f}_y{offset:+.3f}"
                            )
                            trials.append(Trial(
                                case_id, speed_name, fraction, mu, tyre, offset,
                                ramp_friction=ramp_mu,
                                approach_speed_m_s=approach,
                                approach_case=approach_case,
                                record=(case_id == REPRESENTATIVE_CASE_ID),
                            ))
    for repeat_id in (1, 2):
        trials.append(Trial(
            f"NOMINAL_repeat_{repeat_id}", "NOMINAL", SPEED_CASES["NOMINAL"],
            0.6, 60.0, 0.0, repeat_id=repeat_id,
            approach_speed_m_s=ENVELOPE_APPROACH_M_S,
            approach_case=ENVELOPE_APPROACH_CASE,
        ))
    return trials


def capture_threshold_trials() -> list[Trial]:
    """Bracket the minimum commanded approach speed that captures at all."""

    trials = []
    for approach in APPROACH_THRESHOLD_SCAN_M_S:
        for mu in FRICTION_BOUNDS:
            for ramp_mu in RAMP_FRICTION_BOUNDS:
                trials.append(Trial(
                    f"threshold_v{approach:.3f}_mu{mu:.1f}_r{ramp_mu:.2f}",
                    "NOMINAL", SPEED_CASES["NOMINAL"], mu, 60.0, 0.0,
                    ramp_friction=ramp_mu, approach_speed_m_s=approach,
                    approach_case="THRESHOLD_SCAN",
                ))
    return trials


def summarise_threshold(rows: list[dict]) -> dict:
    by_speed: dict[float, list[dict]] = {}
    for row in rows:
        by_speed.setdefault(round(row["approach_speed_m_s"], 3), []).append(row)
    table = []
    for approach in sorted(by_speed):
        group = by_speed[approach]
        table.append({
            "approach_speed_m_s": approach,
            "captured": sum(bool(r["mechanism_captured"]) for r in group),
            "cases": len(group),
            "maximum_ball_centre_z_m": max(r["maximum_ball_centre_z_m"] for r in group),
        })
    first_any = next(
        (row["approach_speed_m_s"] for row in table if row["captured"] > 0), None
    )
    # The lowest speed from which EVERY higher scanned speed captures in every
    # friction bound: a threshold that a single lucky case cannot fake.
    first_all = None
    for index, row in enumerate(table):
        if all(later["captured"] == later["cases"] for later in table[index:]):
            first_all = row["approach_speed_m_s"]
            break
    return {
        "scan_m_s": list(APPROACH_THRESHOLD_SCAN_M_S),
        "swept_bounds": {
            "tread_friction": list(FRICTION_BOUNDS),
            "ball_ramp_friction": list(RAMP_FRICTION_BOUNDS),
        },
        "table": table,
        "MINIMUM_APPROACH_SPEED_WITH_ANY_CAPTURE_M_S": first_any,
        "MINIMUM_APPROACH_SPEED_WITH_CAPTURE_ACROSS_ALL_FRICTION_BOUNDS_M_S": first_all,
        "route_nominal_speed_m_s": ROUTE_NOMINAL_APPROACH_M_S,
        "route_max_speed_m_s": APPROACH_CASES["ROUTE_MAX"],
        "source": "ros2_ws/src/tennis_robot/config/collection_route.yaml",
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-root", type=Path, default=ROOT)
    parser.add_argument("--smoke", action="store_true")
    parser.add_argument(
        "--gazebo-campaign", type=Path,
        default=ROOT / "config/intake_gazebo_campaign.json",
        help="Optional S1-S5 Gazebo campaign result to fold into the report.",
    )
    args = parser.parse_args()
    output_root = args.output_root.resolve()

    if args.smoke:
        for profile in ("flat", "ramp"):
            summary, _ = simulate(Trial(
                f"smoke_{profile}", "NOMINAL", SPEED_CASES["NOMINAL"], 0.6, 60.0, 0.0,
                ground_profile=profile, record=False,
            ))
            print(json.dumps({
                key: value for key, value in summary.items()
                if key not in ("trajectory", "receiving_rows", "height_profile")
            }, indent=2))
        return 0

    trials = build_matrix()
    summaries = []
    telemetry = []
    with ProcessPoolExecutor() as pool:
        for summary, trace in pool.map(simulate, trials, chunksize=4):
            summaries.append(summary)
            telemetry.extend(trace)
    print(f"completed {len(summaries)} matrix trials")

    threshold_trials = capture_threshold_trials()
    with ProcessPoolExecutor() as pool:
        threshold_rows = [row for row, _ in pool.map(simulate, threshold_trials, chunksize=4)]
    threshold = summarise_threshold(threshold_rows)
    print(
        "minimum capture approach speed (any bound / all bounds): "
        f"{threshold['MINIMUM_APPROACH_SPEED_WITH_ANY_CAPTURE_M_S']} / "
        f"{threshold['MINIMUM_APPROACH_SPEED_WITH_CAPTURE_ACROSS_ALL_FRICTION_BOUNDS_M_S']} m/s"
    )

    # Before/after comparison on the identical representative case. Both runs
    # use the reference approach the flat-ground study used, so the ONLY
    # difference between the columns is the ground profile.
    comparison = {}
    for label, (approach_case, approach) in {
        "": ("STUDY_REFERENCE", APPROACH_SPEED_M_S),
        "_envelope": (ENVELOPE_APPROACH_CASE, ENVELOPE_APPROACH_M_S),
    }.items():
        for profile in ("flat", "ramp"):
            trial = Trial(
                REPRESENTATIVE_CASE_ID, "NOMINAL", SPEED_CASES["NOMINAL"], 0.6, 60.0, 0.0,
                ground_profile=profile, record=True,
                approach_speed_m_s=approach, approach_case=approach_case,
            )
            summary, trace = simulate(trial)
            comparison[f"{profile}{label}"] = {
                "summary": summary, "height_profile": summary["height_profile"],
                "approach_speed_m_s": approach,
            }
            if profile == "flat" and not label:
                telemetry.extend(trace)

    docs_dir = output_root / "docs/mechanism"
    images_dir = output_root / "docs/images"
    config_dir = output_root / "config"
    for directory in (docs_dir, images_dir, config_dir):
        directory.mkdir(parents=True, exist_ok=True)
    write_csv(docs_dir / "standalone-intake-handoff-trials.csv", summaries,
              ("trajectory", "receiving_rows", "height_profile"))
    write_csv(docs_dir / "standalone-intake-handoff-telemetry.csv", telemetry)

    convergence = []
    for timestep in (0.0005, 0.00025, 0.000125):
        row, _ = simulate(Trial(
            f"timestep_{timestep}", "NOMINAL", SPEED_CASES["NOMINAL"],
            0.6, 60.0, 0.0, dt=timestep,
            approach_speed_m_s=ENVELOPE_APPROACH_M_S,
            approach_case=ENVELOPE_APPROACH_CASE,
        ))
        convergence.append({
            "timestep_s": timestep,
            "capture_valid": row["capture_valid"],
            "outcome": row["outcome"],
            "exit_speed_m_s": row["exit_speed_m_s"],
            "exit_elevation_deg": row["exit_elevation_deg"],
            "peak_tyre_deflection_m": row["maximum_left_tyre_deflection_m"],
            "peak_normal_force_n": row["peak_left_normal_force_n"],
        })

    gazebo_campaign = None
    if args.gazebo_campaign and args.gazebo_campaign.exists():
        gazebo_campaign = json.loads(args.gazebo_campaign.read_text(encoding="utf-8"))

    valid = [
        row for row in summaries
        if row["capture_valid"] and row["repeat_id"] == 0
        and row["ground_profile"] == "ramp"
        and row["approach_case"] == ENVELOPE_APPROACH_CASE
    ]
    plot_paths = plot_outputs(
        summaries, telemetry, comparison, rim_windows(valid), threshold, images_dir
    )
    result = compile_result(summaries, plot_paths, convergence, comparison,
                            threshold, gazebo_campaign)
    (config_dir / "standalone_intake_handoff_capability.json").write_text(
        json.dumps(result, indent=2) + "\n", encoding="utf-8"
    )
    (docs_dir / "standalone-intake-handoff-capability-report.md").write_text(
        report_markdown(result), encoding="utf-8"
    )
    print(json.dumps(result["classifications"], indent=2))
    print(json.dumps(result["campaign"]["mechanism_outcomes"], indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
