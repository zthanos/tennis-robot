#!/usr/bin/env python3
"""Single authoritative intake geometry and frame mapping.

Every consumer of intake geometry — the reduced-order handoff solver, the
URDF/xacro generator, the ramp mesh generator and the CI frame assertion —
reads this module.  No component may carry its own private offset.

Source authority order (a lower number wins over a higher one):

1. ``cad/standalone-intake-fixed-motor/standalone-intake-fixed-motor-compliant-tyre.scad``
   wheel / motor / bridge geometry.
2. ``cad/collector-intake-v1/option-a/option-a.scad`` (+ ``params.scad``,
   ``bridge-params.scad``) cheeks, ramp, chassis opening, layout.
3. ``ros2_ws/src/tennis_robot/urdf/**`` corrected to match 1 and 2.
4. ``scripts/run_standalone_intake_handoff_study.py`` corrected to match 1 and 2.

Frames
------
CAD ground frame (``cad``)
    Option A ground frame: origin at the chassis plate centre, ``z=0`` at the
    court surface, ``+X`` forward.  ``option-a.scad`` and ``params.scad``
    (``chassis_front_x = 460``, plate 920 mm long and centred) are consistent
    with the chassis plate centre being the origin.

Standalone frame (``std``)
    ``standalone-intake-fixed-motor-compliant-tyre.scad``: wheel axis origin at
    ``(0, 0, 0.070)``, nip mid-plane at ``(0, +/-0.028, 0.070)``.
    ``cad_x = std_x + 0.470``; ``y`` and ``z`` are shared.

Robot ``base_link`` frame (``xacro``)
    ``base_link`` is the chassis plate centre, ``BASE_LINK_HEIGHT_M`` above the
    court.  ``xacro_z = cad_z - 0.045``.  ``xacro_x = cad_x + shift`` where the
    shift is the packaging-variant functional shift below — it is a whole-robot
    packaging decision, never a per-component fudge.

Nothing in this module is a design decision.  Changing a value here is only
ever a transcription correction against the two SCAD sources above.
"""

from __future__ import annotations

import math

__all__ = [
    "BALL_DIAMETER_M", "BALL_RADIUS_M",
    "TILT_DEG", "TILT_RAD", "WHEEL_AXIS",
    "WHEEL_DIAMETER_M", "WHEEL_RADIUS_M", "WHEEL_WIDTH_M", "WHEEL_HALF_WIDTH_M",
    "NOMINAL_GAP_M", "WHEEL_Y_M", "WHEEL_Z_M", "CAD_NIP_X_M", "STD_NIP_X_M",
    "BRIDGE_X_M", "BRIDGE_Y_M", "BRIDGE_Z_M",
    "CHEEK_P0", "CHEEK_P1", "CHEEK_P2", "CHEEK_P3", "CHEEK_THICKNESS_M",
    "CHEEK_MOUTH_X_M", "CHEEK_THROAT_X_M", "CHEEK_THROAT_HALF_Y_M",
    "CHEEK_BOTTOM_Z_M", "CHEEK_TOP_Z_M",
    "RAMP_FRONT_X_M", "RAMP_REAR_X_M", "RAMP_FRONT_Z_M", "RAMP_REAR_Z_M",
    "RAMP_WIDTH_M", "RAMP_WALL_HEIGHT_M", "RAMP_WALL_THICKNESS_M",
    "CHASSIS_TOP_Z_M", "CHASSIS_FRONT_X_M", "BASE_LINK_HEIGHT_M",
    "PACKAGING_SHIFT_X_M", "TREAD_FRICTION_BOUNDS", "BALL_RAMP_FRICTION_BOUNDS",
    "TYRE_FORCE_AT_5MM_BOUNDS_N", "TYRE_RADIAL_ENVELOPE_M",
    "packaging_shift_x_m", "base_link_x", "base_link_z", "cad_x_from_base_link",
    "std_from_cad_x", "cad_from_std_x",
    "ramp_z_m", "ramp_slope_rad", "ramp_profile_polyline",
    "ground_z_m", "cheek_point_m", "cheek_segments", "throat_to_nip_distance_m",
    "wheel_centre_m", "wheel_lowest_z_m", "throat_ball_centre_half_width_m",
    "ramp_ball_centre_half_width_m", "nip_line_point_m", "nip_stations_m",
    "ball_rest_centre_z_m",
]

# --- ball (calibrated contact model / CAD ball_d) -------------------------
BALL_DIAMETER_M = 0.066
BALL_RADIUS_M = BALL_DIAMETER_M / 2.0

# --- wheels / motors / bridge (authority 1) ------------------------------
TILT_DEG = 35.0
TILT_RAD = math.radians(TILT_DEG)
#: Both wheel axes are parallel and lie in the longitudinal X-Z plane.
WHEEL_AXIS = (math.sin(TILT_RAD), 0.0, math.cos(TILT_RAD))
WHEEL_DIAMETER_M = 0.124
WHEEL_RADIUS_M = WHEEL_DIAMETER_M / 2.0
WHEEL_WIDTH_M = 0.073
WHEEL_HALF_WIDTH_M = WHEEL_WIDTH_M / 2.0
NOMINAL_GAP_M = 0.056
#: ``oa_wheel_y = oa_gap/2 + oa_wheel_d/2`` — derived, never typed in twice.
WHEEL_Y_M = NOMINAL_GAP_M / 2.0 + WHEEL_RADIUS_M
WHEEL_Z_M = 0.070
#: ``oa_wheel_x`` — the nip mid-plane in the CAD ground frame.
CAD_NIP_X_M = 0.470
STD_NIP_X_M = 0.0

# Raised bridge from authority 1 (bridge_under_z = 190, bridge_t = 18).
# NOTE: ``cad/collector-intake-v1/option-a/bridge-params.scad`` states
# ``oa_bridge_under_z = 150``.  The two frozen sources disagree by 40 mm.
# Authority 1 owns bridge geometry, so 0.190 is used; the conflict is reported,
# not silently reconciled.
BRIDGE_X_M = (CAD_NIP_X_M - 0.110, CAD_NIP_X_M + 0.110)
BRIDGE_Y_M = (-0.245, 0.245)
BRIDGE_Z_M = (0.190, 0.208)
BRIDGE_UNDER_Z_CONFLICT_M = (0.190, 0.150)

# --- cheeks (authority 2) ------------------------------------------------
#: Cubic Bezier centreline control points, front/mouth -> rear/throat (metres).
CHEEK_P0 = (0.805, 0.205)
CHEEK_P1 = (0.749, 0.205)
CHEEK_P2 = (0.640, 0.083)
CHEEK_P3 = (0.585, 0.083)
CHEEK_THICKNESS_M = 0.006
CHEEK_MOUTH_X_M = CHEEK_P0[0]
CHEEK_THROAT_X_M = CHEEK_P3[0]
CHEEK_THROAT_HALF_Y_M = CHEEK_P3[1]
CHEEK_BOTTOM_Z_M = 0.018
CHEEK_TOP_Z_M = 0.150  # oa_cheek_top_z = oa_bridge_under_z

# --- handoff ramp (authority 2, ``oa_ramp_z``) ---------------------------
RAMP_FRONT_X_M = 0.520
RAMP_REAR_X_M = 0.420
RAMP_FRONT_Z_M = 0.0015
RAMP_REAR_Z_M = 0.035
RAMP_WIDTH_M = 0.180
RAMP_WALL_HEIGHT_M = 0.018
RAMP_WALL_THICKNESS_M = 0.004

# --- chassis (authority 2 / params.scad) ---------------------------------
CHASSIS_TOP_Z_M = 0.052
CHASSIS_FRONT_X_M = 0.460

# --- physically unmeasured bounds (SWEPT, never selected) ----------------
#: Felt/tread friction between the tennis ball and the Trencher tread.
TREAD_FRICTION_BOUNDS = (0.3, 0.6, 0.9)
TREAD_FRICTION_NOMINAL = 0.6
#: Ball-to-ramp friction: tennis felt on a printed/plywood handoff sheet.
#: Bound rationale: dry felt on a smooth rigid polymer sits well below the
#: rubber-tread bound; 0.20 covers a slick printed surface and 0.60 a rough
#: unfinished one.  Physically unmeasured, exactly like the other two.
BALL_RAMP_FRICTION_BOUNDS = (0.20, 0.40, 0.60)
BALL_RAMP_FRICTION_NOMINAL = 0.40
#: Linear tyre radial spring bounds, force at 5 mm radial deflection.
TYRE_FORCE_AT_5MM_BOUNDS_N = (20.0, 60.0, 120.0)
#: Model-validity limit on tyre radial travel (0..5 mm per wheel closure).
TYRE_RADIAL_ENVELOPE_M = (BALL_DIAMETER_M - NOMINAL_GAP_M) / 2.0

# --- robot model frame ---------------------------------------------------
BASE_LINK_HEIGHT_M = 0.045
#: Whole-functional-chain packaging shift per URDF packaging variant.  The
#: ``compact`` study shifts the chain: it moves intake, bridge, basket and
#: launcher together by -100 mm.  It is the ONLY variant left — the flat-shift
#: baseline / option-a-collect / option-a-launch entries were archived on
#: 2026-08-27 so that there is exactly one robot to build and test.  The mapping
#: is kept as a dict, rather than collapsed to a bare constant, so that
#: ``packaging_shift_x_m`` still fails loudly on an unknown name instead of
#: silently returning the compact shift for a stale variant string.
PACKAGING_SHIFT_X_M = {
    "compact": -0.100,
}
#: The only variant that carries the frozen Option A cheeks + ramp + bridge.
CAD_ALIGNED_VARIANT = "compact"


def packaging_shift_x_m(variant: str = CAD_ALIGNED_VARIANT) -> float:
    try:
        return PACKAGING_SHIFT_X_M[variant]
    except KeyError as error:  # fail loud: an unknown variant has no mapping
        raise KeyError(
            f"unknown packaging variant {variant!r}; "
            f"known: {sorted(PACKAGING_SHIFT_X_M)}"
        ) from error


def base_link_x(cad_x_m: float, variant: str = CAD_ALIGNED_VARIANT) -> float:
    return cad_x_m + packaging_shift_x_m(variant)


def cad_x_from_base_link(base_x_m: float, variant: str = CAD_ALIGNED_VARIANT) -> float:
    return base_x_m - packaging_shift_x_m(variant)


def base_link_z(cad_z_m: float) -> float:
    return cad_z_m - BASE_LINK_HEIGHT_M


def std_from_cad_x(cad_x_m: float) -> float:
    return cad_x_m - CAD_NIP_X_M


def cad_from_std_x(std_x_m: float) -> float:
    return std_x_m + CAD_NIP_X_M


# --- derived geometry ----------------------------------------------------
def ramp_z_m(cad_x_m: float) -> float:
    """Authoritative ``oa_ramp_z(x)`` smoothstep, in metres, CAD frame."""

    span = RAMP_FRONT_X_M - RAMP_REAR_X_M
    t = (RAMP_FRONT_X_M - cad_x_m) / span
    t = max(0.0, min(1.0, t))
    return RAMP_FRONT_Z_M + (RAMP_REAR_Z_M - RAMP_FRONT_Z_M) * (t * t * (3.0 - 2.0 * t))


def ramp_slope_rad(cad_x_m: float) -> float:
    """Upward surface slope magnitude at ``cad_x_m`` (rearward is uphill)."""

    span = RAMP_FRONT_X_M - RAMP_REAR_X_M
    t = (RAMP_FRONT_X_M - cad_x_m) / span
    if t <= 0.0 or t >= 1.0:
        return 0.0
    dz_dt = (RAMP_REAR_Z_M - RAMP_FRONT_Z_M) * 6.0 * t * (1.0 - t)
    return math.atan2(dz_dt, span)


def ground_z_m(cad_x_m: float) -> float:
    """Support height under the ball: court ahead of the lip, ramp behind it."""

    if cad_x_m >= RAMP_FRONT_X_M:
        return 0.0
    return ramp_z_m(cad_x_m)


def ball_rest_centre_z_m(cad_x_m: float) -> float:
    """Ball centre height resting on the local surface, ``z + R/cos(beta)``."""

    return ground_z_m(cad_x_m) + BALL_RADIUS_M / math.cos(ramp_slope_rad(cad_x_m))


def ramp_profile_polyline(steps: int = 100) -> list[tuple[float, float]]:
    """Ground support profile in the CAD X-Z plane, front (+x) to rear (-x).

    Starts on the court ahead of the ramp, includes the 1.5 mm vertical lip
    face, then the smoothstep sheet up to the rear edge.  A polyline, so the
    lip corner is an explicit convex vertex the ball can be ploughed by.
    """

    points = [(RAMP_FRONT_X_M + 0.400, 0.0), (RAMP_FRONT_X_M, 0.0)]
    for index in range(steps + 1):
        x = RAMP_FRONT_X_M - (RAMP_FRONT_X_M - RAMP_REAR_X_M) * index / steps
        points.append((x, ramp_z_m(x)))
    return points


def ramp_ball_centre_half_width_m() -> float:
    """Lateral bound on the ball centre imposed by the ramp side walls."""

    return RAMP_WIDTH_M / 2.0 - BALL_RADIUS_M


def throat_ball_centre_half_width_m() -> float:
    """Lateral bound on the ball centre at the cheek throat."""

    return CHEEK_THROAT_HALF_Y_M - CHEEK_THICKNESS_M / 2.0 - BALL_RADIUS_M


def cheek_point_m(t: float) -> tuple[float, float]:
    """Cheek centreline point (CAD frame, ``+y`` side), ``t`` mouth -> throat."""

    u = 1.0 - t
    return tuple(
        CHEEK_P0[i] * u ** 3
        + CHEEK_P1[i] * 3.0 * u * u * t
        + CHEEK_P2[i] * 3.0 * u * t * t
        + CHEEK_P3[i] * t ** 3
        for i in range(2)
    )


def cheek_segments(steps: int = 8) -> list[dict]:
    """Convex box stations approximating the cheek centreline (CAD frame)."""

    segments = []
    for index in range(steps):
        start = cheek_point_m(index / steps)
        end = cheek_point_m((index + 1) / steps)
        segments.append({
            "index": index,
            "x_m": (start[0] + end[0]) / 2.0,
            "y_m": (start[1] + end[1]) / 2.0,
            "length_m": math.hypot(end[0] - start[0], end[1] - start[1]),
            "yaw_rad": math.atan2(end[1] - start[1], end[0] - start[0]),
        })
    return segments


def throat_to_nip_distance_m() -> float:
    """CAD distance from the cheek throat to the nip mid-plane (0.115 m)."""

    return CHEEK_THROAT_X_M - CAD_NIP_X_M


def wheel_centre_m(side: int, frame: str = "cad") -> tuple[float, float, float]:
    """Wheel centre; ``side`` is +1 for the ``+y`` (left) wheel."""

    x = {"cad": CAD_NIP_X_M, "std": STD_NIP_X_M}[frame]
    return (x, side * WHEEL_Y_M, WHEEL_Z_M)


def wheel_lowest_z_m() -> float:
    """Lowest point of the tilted finite wheel cylinder above the court."""

    return WHEEL_Z_M - (
        WHEEL_HALF_WIDTH_M * math.cos(TILT_RAD) + WHEEL_RADIUS_M * math.sin(TILT_RAD)
    )


def nip_line_point_m(axial_offset_m: float, frame: str = "cad") -> tuple[float, float]:
    """Point on the tilted nip line at ``axial_offset_m`` along the wheel axis."""

    x = {"cad": CAD_NIP_X_M, "std": STD_NIP_X_M}[frame]
    return (
        x + axial_offset_m * WHEEL_AXIS[0],
        WHEEL_Z_M + axial_offset_m * WHEEL_AXIS[2],
    )


def nip_stations_m(frame: str = "cad") -> dict[str, tuple[float, float]]:
    """The three reporting stations on the tilted nip line (x, z)."""

    return {
        "NIP_UPPER": nip_line_point_m(+WHEEL_HALF_WIDTH_M, frame),
        "NIP_MID": nip_line_point_m(0.0, frame),
        "NIP_LOWER": nip_line_point_m(-WHEEL_HALF_WIDTH_M, frame),
    }
