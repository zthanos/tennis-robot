"""CAD -> Python transcription guard for scripts/intake_geometry.py.

``intake_geometry`` declares an authority order and then hand-transcribes the
CAD numbers as Python constants. Nothing used to check those transcriptions, so
an edit to (say) ``oa_ramp_front_x`` in the CAD would leave the Python silently
stale and every downstream number — handoff envelope, ramp profile, even the
frame assertion's own expected values — would drift with no test failing. That
is the failure class behind debug-log appendix #63.1.

This module parses the authoritative ``.scad`` sources and compares them, value
by value, against the Python constants. Expected numbers are NEVER re-declared
here as literals: that would only move the transcription instead of guarding it.
Derived CAD values (``oa_wheel_y = oa_gap/2 + oa_wheel_d/2``) are evaluated from
their parsed inputs.

The CAD is authoritative. A failure here means the Python is stale — or that the
study behind it needs re-deriving. It never means "edit the CAD to match".
"""

from __future__ import annotations

import ast
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

import intake_geometry as geom  # noqa: E402

GEOMETRY_MODULE = "scripts/intake_geometry.py"

STANDALONE_SCAD = Path(
    "cad/standalone-intake-fixed-motor/standalone-intake-fixed-motor-compliant-tyre.scad"
)
OPTION_A_SCAD = Path("cad/collector-intake-v1/option-a/option-a.scad")
OPTION_A_PARAMS = Path("cad/collector-intake-v1/option-a/params.scad")
OPTION_A_BRIDGE = Path("cad/collector-intake-v1/option-a/bridge-params.scad")
COMPACT_STUDY_SCAD = Path("cad/flywheel-launcher-v0/compact-packaging-study.scad")
ROBOT_XACRO = Path("ros2_ws/src/tennis_robot/urdf/tennis_robot.urdf.xacro")

MM = 1000.0
ASSIGNMENT = re.compile(r"^[ \t]*(\$?[A-Za-z_][A-Za-z_0-9]*)[ \t]*=[ \t]*(.+?);", re.M)


class Unsupported(Exception):
    """The right-hand side is not a plain numeric expression."""


def _evaluate(expression: str, environment: dict):
    """Evaluate a numeric SCAD right-hand side against already-parsed names."""

    try:
        tree = ast.parse(expression.strip(), mode="eval")
    except SyntaxError as error:  # SCAD-only syntax (strings, ranges, calls)
        raise Unsupported(expression) from error

    def walk(node):
        if isinstance(node, ast.Constant):
            if isinstance(node.value, bool) or not isinstance(node.value, (int, float)):
                raise Unsupported(expression)
            return float(node.value)
        if isinstance(node, (ast.List, ast.Tuple)):
            return [walk(item) for item in node.elts]
        if isinstance(node, ast.Name):
            if node.id not in environment:
                raise Unsupported(expression)
            return environment[node.id]
        if isinstance(node, ast.UnaryOp) and isinstance(node.op, (ast.USub, ast.UAdd)):
            value = walk(node.operand)
            return -value if isinstance(node.op, ast.USub) else value
        if isinstance(node, ast.BinOp):
            left, right = walk(node.left), walk(node.right)
            if isinstance(node.op, ast.Add):
                return left + right
            if isinstance(node.op, ast.Sub):
                return left - right
            if isinstance(node.op, ast.Mult):
                return left * right
            if isinstance(node.op, ast.Div):
                return left / right
        raise Unsupported(expression)

    return walk(tree.body)


def scad_environment(*paths: Path) -> dict:
    """Parse SCAD assignments from the given files, in include order."""

    environment: dict = {}
    for path in paths:
        text = (ROOT / path).read_text(encoding="utf-8")
        # Strip line comments so a commented-out assignment cannot win.
        text = re.sub(r"//[^\n]*", "", text)
        for name, expression in ASSIGNMENT.findall(text):
            try:
                environment[name] = _evaluate(expression, environment)
            except Unsupported:
                continue
    return environment


@pytest.fixture(scope="module")
def standalone() -> dict:
    return scad_environment(STANDALONE_SCAD)


@pytest.fixture(scope="module")
def option_a() -> dict:
    # option-a.scad includes params.scad and bridge-params.scad, so the same
    # environment must carry them for derived values like oa_cheek_top_z.
    return scad_environment(OPTION_A_PARAMS, OPTION_A_BRIDGE, OPTION_A_SCAD)


def check_mm(python_name: str, python_value_m: float,
             environment: dict, source: Path, scad_name: str,
             tolerance_m: float = 1e-9) -> None:
    """Assert a Python metre constant equals a SCAD millimetre declaration."""

    assert scad_name in environment, (
        f"{scad_name} is no longer declared in {source}; "
        f"{GEOMETRY_MODULE}:{python_name} has lost its authoritative source"
    )
    expected_mm = environment[scad_name]
    expected_m = expected_mm / MM
    assert abs(python_value_m - expected_m) <= tolerance_m, (
        f"TRANSCRIPTION MISMATCH: {GEOMETRY_MODULE}:{python_name} = "
        f"{python_value_m} m ({python_value_m * MM:g} mm), but {source}:{scad_name} = "
        f"{expected_mm:g} mm ({expected_m} m). The CAD is authoritative — update "
        f"{GEOMETRY_MODULE} and re-derive anything downstream; do NOT edit the CAD "
        f"to match the Python."
    )


def check_value(python_name: str, python_value: float, expected: float,
                source: Path, derivation: str, tolerance: float = 1e-9) -> None:
    """Assert a Python constant equals a value derived from parsed CAD inputs."""

    assert abs(python_value - expected) <= tolerance, (
        f"TRANSCRIPTION MISMATCH: {GEOMETRY_MODULE}:{python_name} = {python_value}, "
        f"but {source} gives {expected} via {derivation}. The CAD is authoritative."
    )


# --------------------------------------------------------------------------
# Authority 1 — wheel / motor / bridge geometry.
# --------------------------------------------------------------------------
def test_wheel_and_ball_constants_match_the_standalone_cad(standalone):
    check_value("TILT_DEG", geom.TILT_DEG, standalone["tilt_deg"],
                STANDALONE_SCAD, "tilt_deg (degrees, not converted)")
    check_mm("WHEEL_DIAMETER_M", geom.WHEEL_DIAMETER_M, standalone, STANDALONE_SCAD, "wheel_d")
    check_mm("WHEEL_WIDTH_M", geom.WHEEL_WIDTH_M, standalone, STANDALONE_SCAD, "wheel_width")
    check_mm("WHEEL_Y_M", geom.WHEEL_Y_M, standalone, STANDALONE_SCAD, "wheel_y")
    check_mm("WHEEL_Z_M", geom.WHEEL_Z_M, standalone, STANDALONE_SCAD, "wheel_z")
    check_mm("NOMINAL_GAP_M", geom.NOMINAL_GAP_M, standalone, STANDALONE_SCAD, "fixed_gap")
    check_mm("BALL_DIAMETER_M", geom.BALL_DIAMETER_M, standalone, STANDALONE_SCAD, "ball_d")
    check_value("BALL_RADIUS_M", geom.BALL_RADIUS_M, standalone["ball_d"] / MM / 2.0,
                STANDALONE_SCAD, "ball_d / 2")
    check_value("WHEEL_RADIUS_M", geom.WHEEL_RADIUS_M, standalone["wheel_d"] / MM / 2.0,
                STANDALONE_SCAD, "wheel_d / 2")
    check_value("WHEEL_HALF_WIDTH_M", geom.WHEEL_HALF_WIDTH_M,
                standalone["wheel_width"] / MM / 2.0, STANDALONE_SCAD, "wheel_width / 2")


def test_tyre_envelope_is_evaluated_from_its_cad_derivation(standalone):
    # tyre_required_radial_envelope = total_closure/2 = (ball_d - fixed_gap)/2.
    check_mm("TYRE_RADIAL_ENVELOPE_M", geom.TYRE_RADIAL_ENVELOPE_M,
             standalone, STANDALONE_SCAD, "tyre_required_radial_envelope")
    check_value("TYRE_RADIAL_ENVELOPE_M", geom.TYRE_RADIAL_ENVELOPE_M,
                (standalone["ball_d"] - standalone["fixed_gap"]) / MM / 2.0,
                STANDALONE_SCAD, "(ball_d - fixed_gap) / 2")


def test_bridge_envelope_is_evaluated_from_its_cad_inputs(standalone, option_a):
    # The standalone bridge is centred on the wheel-axis origin, which is the
    # nip mid-plane, so its CAD-frame X span is nip +/- bridge_depth/2.
    half_depth = standalone["bridge_depth"] / MM / 2.0
    check_value("BRIDGE_X_M[0]", geom.BRIDGE_X_M[0], geom.CAD_NIP_X_M - half_depth,
                STANDALONE_SCAD, "oa_wheel_x - bridge_depth/2")
    check_value("BRIDGE_X_M[1]", geom.BRIDGE_X_M[1], geom.CAD_NIP_X_M + half_depth,
                STANDALONE_SCAD, "oa_wheel_x + bridge_depth/2")
    half_width = standalone["bridge_width"] / MM / 2.0
    check_value("BRIDGE_Y_M[0]", geom.BRIDGE_Y_M[0], -half_width,
                STANDALONE_SCAD, "-bridge_width/2")
    check_value("BRIDGE_Y_M[1]", geom.BRIDGE_Y_M[1], half_width,
                STANDALONE_SCAD, "+bridge_width/2")
    check_mm("BRIDGE_Z_M[0]", geom.BRIDGE_Z_M[0], standalone, STANDALONE_SCAD,
             "bridge_under_z")
    check_value("BRIDGE_Z_M[1]", geom.BRIDGE_Z_M[1],
                (standalone["bridge_under_z"] + standalone["bridge_t"]) / MM,
                STANDALONE_SCAD, "bridge_under_z + bridge_t")


def test_known_bridge_height_conflict_still_matches_both_authorities(standalone, option_a):
    """The two frozen sources disagree by 40 mm; the disagreement is recorded.

    If either source moves, the recorded conflict must move with it rather than
    quietly becoming fiction.
    """

    check_mm("BRIDGE_UNDER_Z_CONFLICT_M[0]", geom.BRIDGE_UNDER_Z_CONFLICT_M[0],
             standalone, STANDALONE_SCAD, "bridge_under_z")
    check_mm("BRIDGE_UNDER_Z_CONFLICT_M[1]", geom.BRIDGE_UNDER_Z_CONFLICT_M[1],
             option_a, OPTION_A_BRIDGE, "oa_bridge_under_z")
    assert geom.BRIDGE_Z_M[0] == geom.BRIDGE_UNDER_Z_CONFLICT_M[0], (
        "authority 1 owns bridge geometry; BRIDGE_Z_M must follow the standalone CAD"
    )


# --------------------------------------------------------------------------
# Authority 2 — Option A layout, cheeks and ramp.
# --------------------------------------------------------------------------
def test_option_a_layout_agrees_with_authority_1(option_a, standalone):
    check_mm("CAD_NIP_X_M", geom.CAD_NIP_X_M, option_a, OPTION_A_SCAD, "oa_wheel_x")
    check_mm("WHEEL_DIAMETER_M", geom.WHEEL_DIAMETER_M, option_a, OPTION_A_SCAD, "oa_wheel_d")
    check_mm("WHEEL_WIDTH_M", geom.WHEEL_WIDTH_M, option_a, OPTION_A_SCAD, "oa_wheel_width")
    check_mm("NOMINAL_GAP_M", geom.NOMINAL_GAP_M, option_a, OPTION_A_SCAD, "oa_gap")
    check_mm("WHEEL_Z_M", geom.WHEEL_Z_M, option_a, OPTION_A_SCAD, "oa_wheel_z")
    check_value("TILT_DEG", geom.TILT_DEG, option_a["oa_wheel_tilt"],
                OPTION_A_SCAD, "oa_wheel_tilt (degrees)")
    # oa_wheel_y is DERIVED in the CAD; evaluate the derivation, do not hardcode.
    check_mm("WHEEL_Y_M", geom.WHEEL_Y_M, option_a, OPTION_A_SCAD, "oa_wheel_y")
    check_value("WHEEL_Y_M", geom.WHEEL_Y_M,
                (option_a["oa_gap"] / 2.0 + option_a["oa_wheel_d"] / 2.0) / MM,
                OPTION_A_SCAD, "oa_gap/2 + oa_wheel_d/2")
    # The two authorities must also agree with each other.
    for scad_name, other_name in (("wheel_d", "oa_wheel_d"), ("wheel_width", "oa_wheel_width"),
                                  ("wheel_y", "oa_wheel_y"), ("wheel_z", "oa_wheel_z"),
                                  ("fixed_gap", "oa_gap"), ("tilt_deg", "oa_wheel_tilt")):
        assert standalone[scad_name] == pytest.approx(option_a[other_name], abs=1e-9), (
            f"AUTHORITY CONFLICT: {STANDALONE_SCAD}:{scad_name} = {standalone[scad_name]} "
            f"but {OPTION_A_SCAD}:{other_name} = {option_a[other_name]}"
        )


def test_cheek_constants_match_option_a(option_a):
    for index, point in enumerate((geom.CHEEK_P0, geom.CHEEK_P1, geom.CHEEK_P2, geom.CHEEK_P3)):
        scad_name = f"oa_cheek_p{index}"
        assert scad_name in option_a, f"{scad_name} missing from {OPTION_A_SCAD}"
        expected = [value / MM for value in option_a[scad_name]]
        assert list(point) == pytest.approx(expected, abs=1e-9), (
            f"TRANSCRIPTION MISMATCH: {GEOMETRY_MODULE}:CHEEK_P{index} = {point} m, but "
            f"{OPTION_A_SCAD}:{scad_name} = {option_a[scad_name]} mm. The CAD is authoritative."
        )
    check_mm("CHEEK_THICKNESS_M", geom.CHEEK_THICKNESS_M, option_a, OPTION_A_SCAD, "oa_cheek_t")
    check_mm("CHEEK_BOTTOM_Z_M", geom.CHEEK_BOTTOM_Z_M, option_a, OPTION_A_SCAD,
             "oa_cheek_bottom_z")
    # oa_cheek_top_z is DERIVED: it equals oa_bridge_under_z from bridge-params.
    check_mm("CHEEK_TOP_Z_M", geom.CHEEK_TOP_Z_M, option_a, OPTION_A_SCAD, "oa_cheek_top_z")
    check_value("CHEEK_TOP_Z_M", geom.CHEEK_TOP_Z_M, option_a["oa_bridge_under_z"] / MM,
                OPTION_A_BRIDGE, "oa_cheek_top_z = oa_bridge_under_z")
    # The mouth and throat stations are the Bezier end points, not new numbers.
    check_value("CHEEK_MOUTH_X_M", geom.CHEEK_MOUTH_X_M, option_a["oa_cheek_p0"][0] / MM,
                OPTION_A_SCAD, "oa_cheek_p0[0]")
    check_value("CHEEK_THROAT_X_M", geom.CHEEK_THROAT_X_M, option_a["oa_cheek_p3"][0] / MM,
                OPTION_A_SCAD, "oa_cheek_p3[0]")
    check_value("CHEEK_THROAT_HALF_Y_M", geom.CHEEK_THROAT_HALF_Y_M,
                option_a["oa_cheek_p3"][1] / MM, OPTION_A_SCAD, "oa_cheek_p3[1]")


def test_ramp_constants_match_option_a(option_a):
    check_mm("RAMP_FRONT_X_M", geom.RAMP_FRONT_X_M, option_a, OPTION_A_SCAD, "oa_ramp_front_x")
    check_mm("RAMP_REAR_X_M", geom.RAMP_REAR_X_M, option_a, OPTION_A_SCAD, "oa_ramp_rear_x")
    check_mm("RAMP_FRONT_Z_M", geom.RAMP_FRONT_Z_M, option_a, OPTION_A_SCAD, "oa_ramp_front_z")
    check_mm("RAMP_REAR_Z_M", geom.RAMP_REAR_Z_M, option_a, OPTION_A_SCAD, "oa_ramp_rear_z")
    check_mm("RAMP_WIDTH_M", geom.RAMP_WIDTH_M, option_a, OPTION_A_SCAD, "oa_ramp_width")
    check_mm("RAMP_WALL_HEIGHT_M", geom.RAMP_WALL_HEIGHT_M, option_a, OPTION_A_SCAD,
             "oa_ramp_wall_h")


def test_ramp_wall_thickness_matches_the_wall_solid_in_the_cad(option_a):
    """The wall thickness is not a named variable: it lives in the geometry.

    ``short_handoff_ramp()`` sweeps ``cube([0.7, 4, ...])`` centred at
    ``oa_ramp_width/2 + 2``, i.e. a 4 mm wall whose inner face sits exactly on
    the 180 mm sheet edge. Parse both numbers out of that solid.
    """

    text = (ROOT / OPTION_A_SCAD).read_text(encoding="utf-8")
    module = text[text.index("module short_handoff_ramp"):]
    module = module[:module.index("\nmodule ")] if "\nmodule " in module else module
    offsets = re.findall(r"oa_ramp_width\s*/\s*2\s*\+\s*([\d.]+)", module)
    thicknesses = re.findall(r"cube\(\[\s*[\d.]+\s*,\s*([\d.]+)\s*,", module)
    assert offsets, f"no wall offset found in {OPTION_A_SCAD}:short_handoff_ramp()"
    assert thicknesses, f"no wall cube found in {OPTION_A_SCAD}:short_handoff_ramp()"
    # The sheet cube is the full ramp width; the wall cubes are the thin ones.
    wall_thickness_mm = min(float(value) for value in thicknesses
                            if float(value) != option_a["oa_ramp_width"])
    offset_mm = float(offsets[0])
    check_value("RAMP_WALL_THICKNESS_M", geom.RAMP_WALL_THICKNESS_M,
                wall_thickness_mm / MM, OPTION_A_SCAD,
                "the cube([0.7, t, ...]) wall solid in short_handoff_ramp()")
    check_value("RAMP_WALL_THICKNESS_M (from its centre offset)",
                geom.RAMP_WALL_THICKNESS_M, offset_mm * 2.0 / MM, OPTION_A_SCAD,
                "wall centred at oa_ramp_width/2 + thickness/2")


def test_chassis_constants_match_option_a_params(option_a):
    check_mm("CHASSIS_TOP_Z_M", geom.CHASSIS_TOP_Z_M, option_a, OPTION_A_PARAMS,
             "chassis_plate_top_z")
    check_mm("CHASSIS_FRONT_X_M", geom.CHASSIS_FRONT_X_M, option_a, OPTION_A_PARAMS,
             "chassis_front_x")


# --------------------------------------------------------------------------
# Constants whose authority is NOT a .scad file.
# --------------------------------------------------------------------------
def test_packaging_shift_matches_the_compact_study_cad():
    """The compact functional shift is CAD, but in the packaging study source."""

    environment = scad_environment(COMPACT_STUDY_SCAD)
    check_value("PACKAGING_SHIFT_X_M['compact']",
                geom.PACKAGING_SHIFT_X_M["compact"],
                environment["functional_shift_x"] / MM,
                COMPACT_STUDY_SCAD, "functional_shift_x")
    # Every other variant is the unshifted Option A layout by definition.
    for variant, shift in geom.PACKAGING_SHIFT_X_M.items():
        if variant != "compact":
            assert shift == 0.0, f"{variant} must carry no functional shift, got {shift}"


def test_base_link_height_matches_the_robot_model():
    """base_link_height has no CAD source: it is a robot-model datum (authority 3).

    The CAD works in the ground frame; base_link is where the URDF puts the
    chassis-plate centre. Guard it against the xacro that owns it.
    """

    text = (ROOT / ROBOT_XACRO).read_text(encoding="utf-8")
    match = re.search(r'<xacro:property name="base_link_height" value="([\d.]+)"/>', text)
    assert match, f"base_link_height not found in {ROBOT_XACRO}"
    check_value("BASE_LINK_HEIGHT_M", geom.BASE_LINK_HEIGHT_M, float(match.group(1)),
                ROBOT_XACRO, "xacro property base_link_height (already in metres)")


def test_unmeasured_bounds_are_declared_as_analysis_allocations():
    """These constants deliberately have NO CAD source, and must say so.

    Tread friction, ball-to-ramp friction and tyre stiffness are physically
    unmeasured swept bounds. They are listed here rather than silently skipped,
    so the guard's coverage is explicit: if one of them ever acquires a
    measurement, this test is where that shows up.
    """

    source = (ROOT / GEOMETRY_MODULE).read_text(encoding="utf-8")
    for name in ("TREAD_FRICTION_BOUNDS", "BALL_RAMP_FRICTION_BOUNDS",
                 "TYRE_FORCE_AT_5MM_BOUNDS_N"):
        assert hasattr(geom, name)
        assert len(getattr(geom, name)) == 3, f"{name} must stay a three-point bound"
    assert "physically unmeasured" in source, (
        "the swept bounds must remain labelled as physically unmeasured"
    )
    # No CAD file may be cited as their source.
    start_marker = "# --- physically unmeasured bounds"
    end_marker = "# --- robot model frame"
    assert start_marker in source and end_marker in source, (
        f"{GEOMETRY_MODULE} no longer separates the unmeasured bounds into their "
        "own labelled block; this guard cannot tell measured from allocated"
    )
    block = source[source.index(start_marker):source.index(end_marker)]
    assert ".scad" not in block, (
        "an unmeasured bound must not claim a CAD source; if it acquired one, "
        "move it into the transcription checks above"
    )


# --------------------------------------------------------------------------
# The CAD sources must still parse and render.
# --------------------------------------------------------------------------
def _openscad_available() -> bool:
    if shutil.which("docker") is None:
        return False
    try:
        return subprocess.run(
            ["docker", "info"], cwd=ROOT, stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL, timeout=60,
        ).returncode == 0
    except (subprocess.SubprocessError, OSError):
        return False


requires_openscad = pytest.mark.skipif(
    not _openscad_available(),
    reason=(
        "OpenSCAD runs through the docker compose 'cad' profile "
        "(docker compose --profile cad run --rm openscad ...); docker is unavailable"
    ),
)


@requires_openscad
@pytest.mark.parametrize("scad", [OPTION_A_SCAD, STANDALONE_SCAD], ids=["option_a", "standalone"])
def test_authoritative_cad_still_renders(scad):
    """A syntax error in either CAD file was previously caught by nothing."""

    result = subprocess.run(
        ["docker", "compose", "--profile", "cad", "run", "--rm",
         "--user", f"{os.getuid()}:{os.getgid()}", "openscad",
         "openscad", "-o", "/dev/null", "--export-format", "binstl", str(scad)],
        cwd=ROOT, capture_output=True, text=True, timeout=900,
    )
    assert result.returncode == 0, (
        f"{scad} failed to render (exit {result.returncode}).\n"
        f"stdout:\n{result.stdout}\nstderr:\n{result.stderr}"
    )
