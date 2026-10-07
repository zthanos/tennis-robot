#!/usr/bin/env python3
"""Calibrated reduced-order tennis-ball compliant contact model.

The scalar normal law represents total ball diametral deformation. It is
calibrated only from the ITF platen and rigid-surface drop tests, never from a
launcher outcome. Vector contact geometry is kept separate from this law so it
can be unit-tested without Gazebo or ROS.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Iterable


Vec3 = tuple[float, float, float]


def add(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def sub(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def mul(a: Vec3, scalar: float) -> Vec3:
    return (a[0] * scalar, a[1] * scalar, a[2] * scalar)


def dot(a: Vec3, b: Vec3) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def cross(a: Vec3, b: Vec3) -> Vec3:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def norm(a: Vec3) -> float:
    return math.sqrt(dot(a, a))


def unit(a: Vec3, fallback: Vec3 = (1.0, 0.0, 0.0)) -> Vec3:
    length = norm(a)
    return fallback if length <= 1e-15 else mul(a, 1.0 / length)


@dataclass(frozen=True)
class NormalParameters:
    """Physical, calibrated and numerical parameters for the normal law."""

    mass_kg: float = 0.058
    radius_m: float = 0.033
    gravity_m_s2: float = 9.80665

    # PHYSICAL_PARAMETER: independent ITF test loads / sequence.
    preload_force_n: float = 15.57
    total_test_force_n: float = 95.64
    forward_deformation_m: float = 0.0065
    return_deformation_m: float = 0.0094
    precompression_travel_m: float = 0.0254

    # MODEL_FORM_PARAMETER: Hertz-like nonlinear loading assumption. The ITF
    # single loading point cannot identify this exponent.
    loading_exponent: float = 1.5

    # CALIBRATED_PARAMETER: fitted to the 2.54 m -> 1.41 m rigid-surface drop.
    dynamic_damping_n_s_m_pow: float = 4692.375890562493

    # NUMERICAL_PARAMETER: guard outside the calibration domain.
    max_supported_compression_m: float = 0.035
    force_cap_n: float = 5000.0

    @property
    def preload_compression_m(self) -> float:
        ratio = (self.total_test_force_n / self.preload_force_n) ** (
            1.0 / self.loading_exponent
        )
        return self.forward_deformation_m / (ratio - 1.0)

    @property
    def loading_stiffness_n_m_pow(self) -> float:
        return self.preload_force_n / (
            self.preload_compression_m ** self.loading_exponent
        )

    @property
    def forward_total_compression_m(self) -> float:
        return self.preload_compression_m + self.forward_deformation_m

    @property
    def return_total_compression_m(self) -> float:
        return self.preload_compression_m + self.return_deformation_m

    @property
    def itf_max_compression_m(self) -> float:
        return self.preload_compression_m + self.precompression_travel_m

    @property
    def unloading_exponent(self) -> float:
        maximum_force = self.loading_elastic_force(self.itf_max_compression_m)
        return math.log(self.total_test_force_n / maximum_force) / math.log(
            self.return_total_compression_m / self.itf_max_compression_m
        )

    def loading_elastic_force(self, compression_m: float) -> float:
        return self.loading_stiffness_n_m_pow * max(compression_m, 0.0) ** self.loading_exponent


@dataclass
class ContactState:
    maximum_compression_m: float = 0.0
    unloading: bool = False
    separated: bool = False


@dataclass(frozen=True)
class NormalForceSample:
    force_n: float
    elastic_force_n: float
    damping_force_n: float
    compression_m: float
    compression_rate_m_s: float
    loading: bool
    clamped: bool


class CompliantNormalModel:
    def __init__(self, parameters: NormalParameters | None = None):
        self.parameters = parameters or NormalParameters()

    def evaluate(
        self,
        compression_m: float,
        compression_rate_m_s: float,
        maximum_compression_m: float,
    ) -> NormalForceSample:
        p = self.parameters
        if compression_m <= 0.0:
            return NormalForceSample(0.0, 0.0, 0.0, max(compression_m, 0.0), compression_rate_m_s, compression_rate_m_s >= 0.0, False)
        if compression_m > p.max_supported_compression_m:
            raise ValueError(
                f"compression {compression_m:.6f} m exceeds calibrated guard "
                f"{p.max_supported_compression_m:.6f} m"
            )

        maximum = max(maximum_compression_m, compression_m)
        loading = compression_rate_m_s >= 0.0
        if loading:
            elastic = p.loading_elastic_force(compression_m)
        else:
            peak = p.loading_elastic_force(maximum)
            elastic = peak * (compression_m / maximum) ** p.unloading_exponent

        raw_damping = (
            p.dynamic_damping_n_s_m_pow
            * compression_m ** p.loading_exponent
            * compression_rate_m_s
        )
        unclamped = elastic + raw_damping
        force = min(max(unclamped, 0.0), p.force_cap_n)
        return NormalForceSample(
            force_n=force,
            elastic_force_n=elastic,
            damping_force_n=force - elastic,
            compression_m=compression_m,
            compression_rate_m_s=compression_rate_m_s,
            loading=loading,
            clamped=not math.isclose(force, unclamped, rel_tol=0.0, abs_tol=1e-12),
        )

    def step_state(
        self, state: ContactState, compression_m: float, compression_rate_m_s: float
    ) -> NormalForceSample:
        if compression_m <= 0.0:
            state.maximum_compression_m = 0.0
            state.unloading = False
            state.separated = compression_rate_m_s < 0.0
            return self.evaluate(compression_m, compression_rate_m_s, 0.0)
        if state.separated and compression_rate_m_s < 0.0:
            return NormalForceSample(0.0, 0.0, 0.0, compression_m, compression_rate_m_s, False, False)
        state.maximum_compression_m = max(state.maximum_compression_m, compression_m)
        if compression_rate_m_s < 0.0:
            state.unloading = True
        sample = self.evaluate(
            compression_m,
            (
                -max(abs(compression_rate_m_s), 1e-30)
                if state.unloading
                else compression_rate_m_s
            ),
            state.maximum_compression_m,
        )
        if state.unloading and sample.force_n <= 0.0:
            state.separated = True
        return sample


@dataclass(frozen=True)
class FiniteCylinderContact:
    active: bool
    signed_distance_m: float
    compression_m: float
    contact_point_world: Vec3
    normal_world: Vec3
    region: str


def sphere_finite_cylinder_contact(
    sphere_center: Vec3,
    sphere_radius_m: float,
    cylinder_center: Vec3,
    cylinder_axis: Vec3,
    cylinder_radius_m: float,
    cylinder_half_width_m: float,
) -> FiniteCylinderContact:
    """Closest contact from a sphere centre to a closed finite cylinder."""

    axis = unit(cylinder_axis, (0.0, 0.0, 1.0))
    relative = sub(sphere_center, cylinder_center)
    axial = dot(relative, axis)
    radial_vector = sub(relative, mul(axis, axial))
    radial_distance = norm(radial_vector)
    radial_normal = unit(radial_vector, (1.0, 0.0, 0.0))
    axial_excess = abs(axial) - cylinder_half_width_m
    radial_excess = radial_distance - cylinder_radius_m

    if radial_excess > 0.0 and axial_excess > 0.0:
        region = "edge"
        closest = add(
            cylinder_center,
            add(
                mul(axis, math.copysign(cylinder_half_width_m, axial)),
                mul(radial_normal, cylinder_radius_m),
            ),
        )
        outward = sub(sphere_center, closest)
        signed_distance = norm(outward)
        normal_world = unit(outward, radial_normal)
    elif axial_excess > radial_excess:
        region = "cap"
        capped_radial = min(radial_distance, cylinder_radius_m)
        closest = add(
            cylinder_center,
            add(
                mul(axis, math.copysign(cylinder_half_width_m, axial)),
                mul(radial_normal, capped_radial),
            ),
        )
        normal_world = mul(axis, math.copysign(1.0, axial if axial != 0.0 else 1.0))
        signed_distance = axial_excess
    else:
        region = "side"
        clamped_axial = min(max(axial, -cylinder_half_width_m), cylinder_half_width_m)
        closest = add(
            cylinder_center,
            add(mul(axis, clamped_axial), mul(radial_normal, cylinder_radius_m)),
        )
        normal_world = radial_normal
        signed_distance = radial_excess

    compression = sphere_radius_m - signed_distance
    return FiniteCylinderContact(
        active=compression > 0.0,
        signed_distance_m=signed_distance,
        compression_m=max(compression, 0.0),
        contact_point_world=closest,
        normal_world=normal_world,
        region=region,
    )


@dataclass(frozen=True)
class ContactWrench:
    normal_force_n: float
    tangential_relative_velocity_m_s: Vec3
    tangential_force_world_n: Vec3
    friction_limit_n: float | None
    ball_force_world_n: Vec3
    wheel_force_world_n: Vec3
    ball_torque_world_nm: Vec3
    wheel_torque_world_nm: Vec3
    contact_point_world: Vec3
    contact_normal_world: Vec3
    compression_m: float
    compression_rate_m_s: float


def contact_wrench(
    *,
    geometry: FiniteCylinderContact,
    force_sample: NormalForceSample,
    ball_center: Vec3,
    ball_linear_velocity: Vec3,
    ball_angular_velocity: Vec3,
    wheel_center: Vec3,
    wheel_linear_velocity: Vec3,
    wheel_angular_velocity: Vec3,
    friction_coefficient: float | None,
    regularization_speed_m_s: float = 0.05,
) -> ContactWrench:
    normal = geometry.normal_world
    ball_arm = sub(geometry.contact_point_world, ball_center)
    wheel_arm = sub(geometry.contact_point_world, wheel_center)
    ball_point_velocity = add(ball_linear_velocity, cross(ball_angular_velocity, ball_arm))
    wheel_point_velocity = add(wheel_linear_velocity, cross(wheel_angular_velocity, wheel_arm))
    relative_velocity = sub(ball_point_velocity, wheel_point_velocity)
    normal_speed = dot(relative_velocity, normal)
    tangential_velocity = sub(relative_velocity, mul(normal, normal_speed))

    normal_force = force_sample.force_n
    normal_vector_force = mul(normal, normal_force)
    if friction_coefficient is None or norm(tangential_velocity) <= 1e-15:
        tangential_force = (0.0, 0.0, 0.0)
        friction_limit = None if friction_coefficient is None else friction_coefficient * normal_force
    else:
        friction_limit = friction_coefficient * normal_force
        magnitude = friction_limit * math.tanh(norm(tangential_velocity) / regularization_speed_m_s)
        tangential_force = mul(unit(tangential_velocity), -magnitude)

    ball_force = add(normal_vector_force, tangential_force)
    wheel_force = mul(ball_force, -1.0)
    return ContactWrench(
        normal_force_n=normal_force,
        tangential_relative_velocity_m_s=tangential_velocity,
        tangential_force_world_n=tangential_force,
        friction_limit_n=friction_limit,
        ball_force_world_n=ball_force,
        wheel_force_world_n=wheel_force,
        ball_torque_world_nm=cross(ball_arm, tangential_force),
        wheel_torque_world_nm=cross(wheel_arm, mul(tangential_force, -1.0)),
        contact_point_world=geometry.contact_point_world,
        contact_normal_world=normal,
        compression_m=force_sample.compression_m,
        compression_rate_m_s=force_sample.compression_rate_m_s,
    )


def relative_spread(values: Iterable[float]) -> float:
    values = tuple(values)
    mean = sum(values) / len(values)
    return 0.0 if abs(mean) <= 1e-15 else (max(values) - min(values)) / abs(mean)
