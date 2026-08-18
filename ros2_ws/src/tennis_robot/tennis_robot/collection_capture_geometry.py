"""Pure, immutable capture-plane geometry model (``base_footprint`` frame).

This module is part of the **shadow / offline-only** adaptive collection-route
work.  It has NO ROS dependencies, does NOT change the live
``plan_collection_route`` behaviour, and is NOT imported by any runtime node.

It makes the physical intake geometry explicit so the offline adaptive-approach
analysis can reason about *where the ball actually is captured* relative to the
``base_footprint`` origin, instead of assuming that capture happens exactly when
``base_footprint`` crosses the ball centre (the assumption baked into the live
planner today — see ``collection-route-optimization-plan-el.md`` §C.3).

Every plane is a forward-facing plane perpendicular to the robot +x axis,
described by:

* ``longitudinal_offset_m`` — forward (+x) distance from ``base_footprint``,
* ``half_width_m`` — lateral (±y) half extent of the sensing/contact line,
* ``provenance`` — whether the value is measured, configured, or uncalibrated,
* ``source`` — a human-readable citation of where the value comes from.

**No silent fallback / no hidden default.**  The convenience factory
:func:`repo_base_footprint_capture_geometry` requires the caller to pass the
*uncalibrated* pre-contact straight distance and its provenance explicitly, so a
value that has not been established by intake trials is always visibly tagged
``UNCALIBRATED`` rather than defaulted to a plausible-looking number.

Provenance of the packaged repo values (all in ``base_footprint`` frame; note
``base_link`` shares the x/y of ``base_footprint`` — it is only offset in z):

Every packaged value below is CONFIGURED: it is declared in the URDF/xacro/env
configuration, not backed by a physical measurement artifact.  None is tagged
MEASURED — that provenance is reserved for values an actual measurement or
intake trial establishes, which do not yet exist for these planes.

* intake mouth / contact plane — ``x≈0.876 m``, half-width ``0.205 m``
  (``urdf/components/funnel.urdf.xacro``: front cheek tips ``~(0.876, ±0.205)``,
  mouth ``410 mm``).  CONFIGURED: derived from the cheek visual/collision box
  geometry, not a single named parameter.
* entry / throat beam plane — ``x=0.720 m``, half-width ``0.105 m``
  (``urdf/tennis_robot.urdf.xacro``: ``ir_x=0.720``, ``ir_y=0.105``).
  CONFIGURED: declared in xacro, not a measured sensor position.
* roller nip plane — ``x=0.540 m``, half-width ``0.028 m``
  (``INTAKE_NIP_X_M=0.540`` in ``run_ubuntu.sh`` / ``docker-compose.yml`` /
  ``scripts/generate_robot_urdf.py``; corridor half = ``intake_wheel_gap
  0.056 / 2``).  CONFIGURED.  NOTE: the ``0.590`` figure that appears in some
  URDF *comments* is documentation drift — the injected runtime value is
  ``0.540`` in all three source-of-truth locations.
* confirmed / basket beam plane — ``x=0.350 m``, half-width ``0.105 m``
  (``basket_ir_x = basket_floor_front_x 0.42 - basket_management_run 0.14/2``;
  ``basket_ir_y=0.105``).  CONFIGURED: derived from configured basket
  dimensions, not a measured beam position.
* retention reference plane — ``x=0.420 m`` (``basket_floor_front_x`` jump lip).
  CONFIGURED.

The *capture reference plane* (which of the above the run-in should align to)
and the *required pre-contact straight distance* before first funnel contact
are, per the optimisation plan, to be chosen from intake trials — they are
UNCALIBRATED until then and are modelled as explicit required inputs.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from typing import Any, Mapping, TypeVar

_T = TypeVar("_T")


class CaptureGeometryError(ValueError):
    """A capture-geometry artifact was constructed or parsed invalid."""


class PlaneProvenance(str, Enum):
    """Where a geometric value comes from — never inferred silently.

    ``MEASURED`` is reserved for values backed by an actual physical
    measurement or intake-trial artifact.  A value that is merely declared in
    the URDF/xacro/env configuration is ``CONFIGURED`` — configuration is a
    design intent, not a measurement.  ``UNCALIBRATED`` values are not yet
    established and require intake trials.
    """

    MEASURED = "measured"      # backed by an actual physical measurement/trial artifact
    CONFIGURED = "configured"  # read/derived from URDF/xacro/env configuration (not a measurement)
    UNCALIBRATED = "uncalibrated"  # not yet established (needs intake trials)

    def __str__(self) -> str:  # pragma: no cover - trivial
        return self.value


def _finite(value: Any, name: str, *, minimum: float | None = None) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise CaptureGeometryError(f"{name} must be a finite number")
    if minimum is not None and value < minimum:
        raise CaptureGeometryError(f"{name} must be >= {minimum}")
    return float(value)


def _non_empty(value: Any, name: str) -> str:
    if not isinstance(value, str) or not value:
        raise CaptureGeometryError(f"{name} must be a non-empty string")
    return value


def _fields(data: Mapping[str, Any], expected: set[str], name: str) -> None:
    if not isinstance(data, Mapping):
        raise CaptureGeometryError(f"{name} must be a mapping")
    if set(data) != expected:
        raise CaptureGeometryError(f"{name} fields must be exactly {sorted(expected)}")


def _provenance(value: Any, name: str) -> PlaneProvenance:
    try:
        return PlaneProvenance(value)
    except (TypeError, ValueError) as exc:
        raise CaptureGeometryError(f"invalid {name}: {value!r}") from exc


CAPTURE_FRAME = "base_footprint"


@dataclass(frozen=True)
class CapturePlane:
    """A forward-facing capture/sensing plane in the ``base_footprint`` frame."""

    plane_id: str
    longitudinal_offset_m: float
    half_width_m: float
    provenance: PlaneProvenance
    source: str

    def __post_init__(self) -> None:
        _non_empty(self.plane_id, "plane_id")
        _finite(self.longitudinal_offset_m, "longitudinal_offset_m", minimum=0.0)
        _finite(self.half_width_m, "half_width_m", minimum=0.0)
        if not isinstance(self.provenance, PlaneProvenance):
            raise CaptureGeometryError("provenance must be a PlaneProvenance")
        _non_empty(self.source, "source")

    def to_dict(self) -> dict[str, Any]:
        return {
            "plane_id": self.plane_id,
            "longitudinal_offset_m": self.longitudinal_offset_m,
            "half_width_m": self.half_width_m,
            "provenance": self.provenance.value,
            "source": self.source,
        }

    @classmethod
    def from_dict(cls: type[_T], data: Mapping[str, Any]) -> _T:
        _fields(data, {"plane_id", "longitudinal_offset_m", "half_width_m", "provenance", "source"}, "CapturePlane")
        return cls(
            _non_empty(data["plane_id"], "plane_id"),
            _finite(data["longitudinal_offset_m"], "longitudinal_offset_m", minimum=0.0),
            _finite(data["half_width_m"], "half_width_m", minimum=0.0),
            _provenance(data["provenance"], "provenance"),
            _non_empty(data["source"], "source"),
        )


@dataclass(frozen=True)
class CaptureGeometry:
    """Immutable capture-plane model with explicit calibration provenance.

    ``capture_reference_plane_id`` selects which plane the run-in aligns to;
    ``required_pre_contact_straight_m`` is the straight distance that must
    precede first funnel contact.  Both come from intake trials and are, until
    then, tagged ``UNCALIBRATED`` (never defaulted).
    """

    frame: str
    planes: tuple[CapturePlane, ...]
    capture_reference_plane_id: str
    required_pre_contact_straight_m: float
    required_pre_contact_provenance: PlaneProvenance

    def __post_init__(self) -> None:
        if self.frame != CAPTURE_FRAME:
            raise CaptureGeometryError(f"frame must be {CAPTURE_FRAME!r}, got {self.frame!r}")
        if not isinstance(self.planes, tuple) or not self.planes:
            raise CaptureGeometryError("planes must be a non-empty tuple")
        if any(not isinstance(plane, CapturePlane) for plane in self.planes):
            raise CaptureGeometryError("planes must be CapturePlane instances")
        ids = [plane.plane_id for plane in self.planes]
        if len(set(ids)) != len(ids):
            raise CaptureGeometryError("plane ids must be unique")
        if self.capture_reference_plane_id not in set(ids):
            raise CaptureGeometryError("capture_reference_plane_id must name an existing plane")
        _finite(self.required_pre_contact_straight_m, "required_pre_contact_straight_m", minimum=0.0)
        if not isinstance(self.required_pre_contact_provenance, PlaneProvenance):
            raise CaptureGeometryError("required_pre_contact_provenance must be a PlaneProvenance")

    def plane(self, plane_id: str) -> CapturePlane:
        for plane in self.planes:
            if plane.plane_id == plane_id:
                return plane
        raise CaptureGeometryError(f"no capture plane {plane_id!r}")

    @property
    def capture_reference_plane(self) -> CapturePlane:
        return self.plane(self.capture_reference_plane_id)

    @property
    def minimum_alignment_corridor_m(self) -> float:
        """Minimum base-frame straight run-in that keeps the curve out of the
        capture corridor: the reference-plane offset plus the required
        pre-contact straight distance (optimisation plan §C.3)."""
        return self.capture_reference_plane.longitudinal_offset_m + self.required_pre_contact_straight_m

    @property
    def is_calibrated(self) -> bool:
        return not self.uncalibrated_fields()

    def uncalibrated_fields(self) -> tuple[str, ...]:
        """Every field still tagged ``UNCALIBRATED`` — surfaced, never hidden."""
        names = [
            f"plane:{plane.plane_id}"
            for plane in self.planes
            if plane.provenance is PlaneProvenance.UNCALIBRATED
        ]
        if self.required_pre_contact_provenance is PlaneProvenance.UNCALIBRATED:
            names.append("required_pre_contact_straight_m")
        return tuple(names)

    def to_dict(self) -> dict[str, Any]:
        return {
            "frame": self.frame,
            "planes": [plane.to_dict() for plane in self.planes],
            "capture_reference_plane_id": self.capture_reference_plane_id,
            "required_pre_contact_straight_m": self.required_pre_contact_straight_m,
            "required_pre_contact_provenance": self.required_pre_contact_provenance.value,
        }

    @classmethod
    def from_dict(cls: type[_T], data: Mapping[str, Any]) -> _T:
        _fields(
            data,
            {
                "frame",
                "planes",
                "capture_reference_plane_id",
                "required_pre_contact_straight_m",
                "required_pre_contact_provenance",
            },
            "CaptureGeometry",
        )
        if not isinstance(data["planes"], list):
            raise CaptureGeometryError("planes must be a JSON array")
        return cls(
            _non_empty(data["frame"], "frame"),
            tuple(CapturePlane.from_dict(plane) for plane in data["planes"]),
            _non_empty(data["capture_reference_plane_id"], "capture_reference_plane_id"),
            _finite(data["required_pre_contact_straight_m"], "required_pre_contact_straight_m", minimum=0.0),
            _provenance(data["required_pre_contact_provenance"], "required_pre_contact_provenance"),
        )


# ── Packaged repo values (all cited above) ──────────────────────────────────
INTAKE_MOUTH_PLANE_ID = "intake_mouth_contact"
ENTRY_BEAM_PLANE_ID = "entry_throat_beam"
ROLLER_NIP_PLANE_ID = "roller_nip"
CONFIRMED_BEAM_PLANE_ID = "confirmed_basket_beam"
RETENTION_REFERENCE_PLANE_ID = "retention_reference"


def repo_capture_planes() -> tuple[CapturePlane, ...]:
    """The five capture planes with the exact source-of-truth repo values.

    These are NOT invented: every number is cited from the URDF / run scripts
    (see the module docstring).  All five are CONFIGURED — declared in the
    URDF/xacro/env configuration.  None is MEASURED, because no physical
    measurement/trial artifact backs these positions yet; none is UNCALIBRATED,
    because the configured value is known.
    """
    return (
        CapturePlane(
            INTAKE_MOUTH_PLANE_ID,
            0.876,
            0.205,
            PlaneProvenance.CONFIGURED,
            "funnel.urdf.xacro cheek front tips ~(0.876, +/-0.205), mouth 410mm (configured)",
        ),
        CapturePlane(
            ENTRY_BEAM_PLANE_ID,
            0.720,
            0.105,
            PlaneProvenance.CONFIGURED,
            "tennis_robot.urdf.xacro ir_x=0.720, ir_y=0.105 (configured, not a measured sensor position)",
        ),
        CapturePlane(
            ROLLER_NIP_PLANE_ID,
            0.540,
            0.028,
            PlaneProvenance.CONFIGURED,
            "INTAKE_NIP_X_M=0.540 (run_ubuntu.sh/docker-compose/generate_robot_urdf); half=intake_wheel_gap 0.056/2",
        ),
        CapturePlane(
            CONFIRMED_BEAM_PLANE_ID,
            0.350,
            0.105,
            PlaneProvenance.CONFIGURED,
            "basket_ir_x = basket_floor_front_x 0.42 - basket_management_run 0.14/2; basket_ir_y=0.105 (configured/derived)",
        ),
        CapturePlane(
            RETENTION_REFERENCE_PLANE_ID,
            0.420,
            0.140,
            PlaneProvenance.CONFIGURED,
            "basket_floor_front_x=0.42 jump lip; half=basket_half_width 0.14",
        ),
    )


def repo_base_footprint_capture_geometry(
    *,
    capture_reference_plane_id: str = INTAKE_MOUTH_PLANE_ID,
    required_pre_contact_straight_m: float,
    required_pre_contact_provenance: PlaneProvenance,
) -> CaptureGeometry:
    """Build the packaged ``base_footprint`` capture geometry.

    ``required_pre_contact_straight_m`` and ``required_pre_contact_provenance``
    are REQUIRED (no default): the caller must state the value and whether it is
    calibrated.  Passing ``UNCALIBRATED`` keeps the geometry honestly tagged as
    not-yet-trial-established rather than silently defaulted.
    """
    return CaptureGeometry(
        CAPTURE_FRAME,
        repo_capture_planes(),
        capture_reference_plane_id,
        required_pre_contact_straight_m,
        required_pre_contact_provenance,
    )
