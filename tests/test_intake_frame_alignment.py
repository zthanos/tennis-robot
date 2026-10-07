"""CI frame assertion for the frozen intake (task defect D5).

The cheeks, the wheels and the handoff ramp reach the simulation through three
different parents and three different code paths. This test computes all three
in ONE common frame (the Option A CAD ground frame) and compares them against
the frozen CAD, so a frame error can never again hide behind "different parent
links". It must fail loudly rather than be tuned away: every expected value
here is a transcription of an authoritative SCAD source, never a design choice.

Authority order: standalone-intake-fixed-motor-compliant-tyre.scad >
option-a.scad > urdf/** > the reduced-order study.
"""

from __future__ import annotations

import math
import os
import re
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

import intake_geometry as geom  # noqa: E402
from generate_robot_urdf import intake_frame_offsets  # noqa: E402

COMPACT_XACRO = ROOT / "ros2_ws/src/tennis_robot/urdf/components/compact_mechanics.urdf.xacro"
ROBOT_XACRO = ROOT / "ros2_ws/src/tennis_robot/urdf/tennis_robot.urdf.xacro"
RAMP_MESH = ROOT / "ros2_ws/src/tennis_robot/meshes/option_a_handoff_ramp.stl"
GENERATOR = ROOT / "scripts/generate_robot_urdf.py"
CONTROLLERS = ROOT / "ros2_ws/src/tennis_robot/config/controllers.yaml"

VARIANT = geom.CAD_ALIGNED_VARIANT
SHIFT = geom.packaging_shift_x_m(VARIANT)

#: The frozen CAD distance from the cheek throat to the nip mid-plane.
CAD_THROAT_TO_NIP_M = 0.115
#: Position tolerance. 0.5 mm is below the printed/plywood build tolerance and
#: far below any frame error worth arguing about.
TOLERANCE_M = 0.0005


def _xacro_arg_default(name: str) -> str:
    match = re.search(
        rf'<xacro:arg name="{name}" default="([^"]+)"/>', ROBOT_XACRO.read_text("utf-8")
    )
    assert match, f"xacro arg {name} not found"
    return match.group(1)


def _cheek_segment_literals() -> list[dict]:
    """The +y cheek stations as written in the xacro, in base_link X-Y."""

    pattern = re.compile(
        r'<xacro:compact_cheek_segment index="(\d+)" x="([-\d.]+)" y="([-\d.]+)"'
        r' length="([-\d.]+)" yaw="([-\d.]+)" side="1"'
    )
    rows = [
        {
            "index": int(m.group(1)), "x_m": float(m.group(2)),
            "y_m": float(m.group(3)), "length_m": float(m.group(4)),
            "yaw_rad": float(m.group(5)),
        }
        for m in pattern.finditer(COMPACT_XACRO.read_text("utf-8"))
    ]
    assert len(rows) == 8, f"expected 8 cheek segments, found {len(rows)}"
    return sorted(rows, key=lambda row: row["index"])


def _ramp_mesh_vertices() -> list[tuple[float, float, float]]:
    vertices = []
    for line in RAMP_MESH.read_text("utf-8").splitlines():
        fields = line.split()
        if fields and fields[0] == "vertex":
            vertices.append(tuple(float(value) / 1000.0 for value in fields[1:4]))
    assert vertices, "ramp mesh has no vertices"
    return vertices


# --------------------------------------------------------------------------
# The assertion the task asks for: one common frame, real distance, 115 mm.
# --------------------------------------------------------------------------
def test_cheek_throat_to_nip_distance_matches_the_cad():
    offsets = intake_frame_offsets(VARIANT)
    nip_x_cad = offsets["nip_x_base_link_m"] - SHIFT

    segments = _cheek_segment_literals()
    throat_segment = segments[-1]
    # The rear station is the second endpoint of the last Bezier segment.
    half_chord = throat_segment["length_m"] / 2.0
    throat_x_cad = (
        throat_segment["x_m"] + half_chord * math.cos(throat_segment["yaw_rad"]) - SHIFT
    )
    throat_y = throat_segment["y_m"] + half_chord * math.sin(throat_segment["yaw_rad"])

    assert throat_x_cad == pytest.approx(geom.CHEEK_THROAT_X_M, abs=TOLERANCE_M)
    assert throat_y == pytest.approx(geom.CHEEK_THROAT_HALF_Y_M, abs=TOLERANCE_M)
    assert nip_x_cad == pytest.approx(geom.CAD_NIP_X_M, abs=TOLERANCE_M)
    assert throat_x_cad - nip_x_cad == pytest.approx(CAD_THROAT_TO_NIP_M, abs=TOLERANCE_M)
    assert geom.throat_to_nip_distance_m() == pytest.approx(CAD_THROAT_TO_NIP_M, abs=1e-9)


def test_cheek_stations_are_the_cad_bezier_under_one_shift():
    expected = geom.cheek_segments(8)
    for written, cad in zip(_cheek_segment_literals(), expected):
        assert written["x_m"] - SHIFT == pytest.approx(cad["x_m"], abs=TOLERANCE_M)
        assert written["y_m"] == pytest.approx(cad["y_m"], abs=TOLERANCE_M)
        assert written["length_m"] == pytest.approx(cad["length_m"], abs=TOLERANCE_M)
        assert written["yaw_rad"] == pytest.approx(cad["yaw_rad"], abs=1e-5)


def test_wheel_geometry_is_the_frozen_cad():
    radius = float(_xacro_arg_default("intake_wheel_radius"))
    width = float(_xacro_arg_default("intake_wheel_width"))
    gap = float(_xacro_arg_default("intake_wheel_gap"))
    tilt = float(_xacro_arg_default("intake_wheel_tilt_deg"))

    assert radius == pytest.approx(geom.WHEEL_RADIUS_M, abs=1e-9)
    assert width == pytest.approx(geom.WHEEL_WIDTH_M, abs=1e-9)
    assert gap == pytest.approx(geom.NOMINAL_GAP_M, abs=1e-9)
    assert tilt == pytest.approx(geom.TILT_DEG, abs=1e-9)
    # Derived, never typed twice.
    assert gap / 2.0 + radius == pytest.approx(geom.WHEEL_Y_M, abs=1e-9)
    assert geom.WHEEL_Y_M == pytest.approx(0.090, abs=1e-9)
    # Lowest point of the tilted finite cylinder: 4.5 mm, not the 2.8 mm the
    # legacy 60 x 80 mm wheel produced.
    assert geom.wheel_lowest_z_m() == pytest.approx(0.00454, abs=5e-5)
    assert geom.wheel_lowest_z_m() > geom.RAMP_FRONT_Z_M


def test_ramp_mesh_is_the_authoritative_option_a_smoothstep():
    vertices = _ramp_mesh_vertices()
    xs = [v[0] for v in vertices]
    ys = [v[1] for v in vertices]
    zs = [v[2] for v in vertices]

    assert min(xs) == pytest.approx(geom.RAMP_REAR_X_M + SHIFT, abs=TOLERANCE_M)
    assert max(xs) == pytest.approx(geom.RAMP_FRONT_X_M + SHIFT, abs=TOLERANCE_M)
    assert max(ys) == pytest.approx(
        geom.RAMP_WIDTH_M / 2.0 + geom.RAMP_WALL_THICKNESS_M, abs=TOLERANCE_M
    )
    assert max(zs) == pytest.approx(
        geom.RAMP_REAR_Z_M + geom.RAMP_WALL_HEIGHT_M, abs=TOLERANCE_M
    )

    # The sheet under the ball corridor must follow oa_ramp_z exactly, and the
    # side walls must stand oa_ramp_wall_h above the local sheet height.
    stations: dict[float, list[float]] = {}
    for x, y, z in vertices:
        if abs(y) <= geom.RAMP_WIDTH_M / 2.0 + 1e-9:
            stations.setdefault(round(x, 6), []).append(z)
    assert len(stations) >= 20
    for x, heights in stations.items():
        sheet_z = geom.ramp_z_m(x - SHIFT)
        assert min(abs(z - sheet_z) for z in heights) < TOLERANCE_M, x
        assert max(heights) == pytest.approx(
            sheet_z + geom.RAMP_WALL_HEIGHT_M, abs=TOLERANCE_M
        ), x

    # The frozen profile, not the compact study's own 460 -> 420 ramp.
    assert geom.RAMP_FRONT_X_M - geom.RAMP_REAR_X_M == pytest.approx(0.100)
    assert math.degrees(geom.ramp_slope_rad(geom.CAD_NIP_X_M)) == pytest.approx(26.68, abs=0.02)


def test_ramp_runs_through_the_nip_zone_and_bounds_the_ball_laterally():
    """The frozen ground profile is not a datum swap: it rises under the nip."""

    assert geom.RAMP_REAR_X_M < geom.CAD_NIP_X_M < geom.RAMP_FRONT_X_M
    assert geom.ramp_z_m(geom.CAD_NIP_X_M) == pytest.approx(0.01825, abs=1e-6)
    stations = geom.nip_stations_m()
    centres = {
        name: geom.ball_rest_centre_z_m(x) for name, (x, _z) in stations.items()
    }
    assert centres["NIP_UPPER"] == pytest.approx(0.0441, abs=5e-4)
    assert centres["NIP_MID"] == pytest.approx(0.0552, abs=5e-4)
    assert centres["NIP_LOWER"] == pytest.approx(0.0639, abs=5e-4)
    lower_x, lower_z = stations["NIP_LOWER"]
    assert geom.BALL_RADIUS_M - lower_z == pytest.approx(-0.0071, abs=5e-4)
    assert centres["NIP_LOWER"] - lower_z == pytest.approx(0.0238, abs=5e-4)
    assert geom.ramp_ball_centre_half_width_m() == pytest.approx(0.057, abs=1e-9)
    assert geom.throat_ball_centre_half_width_m() == pytest.approx(0.047, abs=1e-9)


def test_ramp_lip_reaches_a_resting_ball_before_the_wheels_do():
    """The crux of the plough finding, as pure frozen-CAD geometry."""

    from tennis_ball_contact_model import sphere_finite_cylinder_contact

    radius = geom.BALL_RADIUS_M
    drop = radius - geom.RAMP_FRONT_Z_M
    lip_contact_x = geom.RAMP_FRONT_X_M + math.sqrt(radius**2 - drop**2)
    assert lip_contact_x == pytest.approx(0.5298, abs=5e-4)

    wheel_contact_x = None
    x = 0.600
    while x > 0.400:
        geometry = sphere_finite_cylinder_contact(
            (x, 0.0, radius), radius, geom.wheel_centre_m(1, "cad"), geom.WHEEL_AXIS,
            geom.WHEEL_RADIUS_M, geom.WHEEL_HALF_WIDTH_M,
        )
        if geometry.active:
            wheel_contact_x = x
            break
        x -= 0.0001
    assert wheel_contact_x is not None
    assert wheel_contact_x == pytest.approx(0.4812, abs=5e-4)
    assert lip_contact_x - wheel_contact_x == pytest.approx(0.0486, abs=1e-3)


def test_no_component_carries_a_private_offset():
    """Every intake x-offset must resolve through intake_frame_offsets()."""

    for variant, shift in geom.PACKAGING_SHIFT_X_M.items():
        offsets = intake_frame_offsets(variant)
        assert offsets["functional_shift_x_m"] == pytest.approx(shift)
        if variant == VARIANT:
            assert offsets["nip_x_base_link_m"] == pytest.approx(
                geom.CAD_NIP_X_M + shift, abs=1e-9
            )
            assert offsets["cad_aligned"] is True
        else:
            # Non-CAD variants keep the historical funnel and nip; they are not
            # the frozen intake and must not be reported as such.
            assert offsets["cad_aligned"] is False


# --------------------------------------------------------------------------
# End-to-end: the same assertion against the actually generated model.
# --------------------------------------------------------------------------
@pytest.mark.skipif(
    shutil.which("xacro") is None or not os.environ.get("AMENT_PREFIX_PATH"),
    reason="ROS 2 environment not sourced (need xacro + AMENT_PREFIX_PATH)",
)
def test_generated_model_places_wheels_cheeks_and_ramp_in_one_frame(tmp_path):
    output = tmp_path / "compact.urdf"
    subprocess.run(
        [sys.executable, str(GENERATOR), "--packaging-variant", VARIANT,
         "--output", str(output), "--controllers-config", str(CONTROLLERS)],
        check=True, cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True,
    )
    root = ET.parse(output).getroot()
    joints = {joint.get("name"): joint for joint in root.findall("joint")}

    assert "intake_wheel_left_carriage_joint" not in joints
    mount = joints["intake_wheel_left_mount_joint"]
    assert mount.get("type") == "fixed"
    x, y, z = (float(value) for value in mount.find("origin").get("xyz").split())
    assert x - SHIFT == pytest.approx(geom.CAD_NIP_X_M, abs=TOLERANCE_M)
    assert y == pytest.approx(geom.WHEEL_Y_M, abs=TOLERANCE_M)
    assert z + geom.BASE_LINK_HEIGHT_M == pytest.approx(geom.WHEEL_Z_M, abs=TOLERANCE_M)

    # The cheek throat box, read back out of the generated model.
    cheeks = next(
        link for link in root.findall("link")
        if link.get("name") == "compact_intake_cheeks_link"
    )
    throat = next(
        collision for collision in cheeks.findall("collision")
        if collision.get("name") == "compact_cheek_1_7_col"
    )
    origin = [float(v) for v in throat.find("origin").get("xyz").split()]
    yaw = float(throat.find("origin").get("rpy").split()[2])
    length = float(throat.find("geometry").find("box").get("size").split()[0])
    throat_x_cad = origin[0] + (length / 2.0) * math.cos(yaw) - SHIFT
    assert throat_x_cad - (x - SHIFT) == pytest.approx(CAD_THROAT_TO_NIP_M, abs=TOLERANCE_M)

    ramp = next(
        link for link in root.findall("link")
        if link.get("name") == "compact_handoff_ramp_link"
    )
    mesh = ramp.find("collision").find("geometry").find("mesh").get("filename")
    assert mesh.endswith("option_a_handoff_ramp.stl")
