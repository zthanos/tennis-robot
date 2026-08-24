"""The simulation clock transport must never be split by a launch variant.

gz /clock -> ros_gz bridge -> /clock_raw -> sim_clock_relay_node -> /clock.

sim_clock_relay_node is the ONLY publisher of /clock. A variant that keeps the
bridge but drops the relay leaves /clock_raw with zero subscribers and every
use_sim_time node — including the controller_manager that gz_ros2_control runs
inside the Gazebo process — on an unset clock. Nothing crashes: physics still
advances and controllers still report "active", but every ROS timestamp freezes
at 0. SIM_BENCH_MINIMAL shipped exactly that split.

These are AST checks rather than a launch-description build so they run in the
pure-pytest suite without ROS. `scripts/sim_debug/verify_sim_bench.py` is the
runtime counterpart that proves the property on a live bench.
"""

from __future__ import annotations

import ast
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
LAUNCH_FILE = ROOT / "ros2_ws/src/tennis_robot/launch/sim.launch.py"


def _module() -> ast.Module:
    return ast.parse(LAUNCH_FILE.read_text(encoding="utf-8"), str(LAUNCH_FILE))


def _list_assignments(tree: ast.Module, target: str) -> list[ast.List]:
    found = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Assign) or not isinstance(node.value, ast.List):
            continue
        if any(isinstance(t, ast.Name) and t.id == target for t in node.targets):
            found.append(node.value)
    return found


def _referenced_names(node: ast.List) -> set[str]:
    """Names used directly or splatted into a list literal."""
    names: set[str] = set()
    for element in node.elts:
        value = element.value if isinstance(element, ast.Starred) else element
        if isinstance(value, ast.Name):
            names.add(value.id)
    return names


def test_clock_transport_is_declared_as_one_unit():
    assignments = _list_assignments(_module(), "_sim_clock_transport")
    assert len(assignments) == 1, "_sim_clock_transport must be defined exactly once"
    assert _referenced_names(assignments[0]) == {"bridge", "sim_clock_relay"}


def test_every_node_set_carries_the_whole_clock_transport():
    tree = _module()
    targets = ["_sim_node_actions", "delayed_node_actions"]
    checked = 0
    for target in targets:
        for assignment in _list_assignments(tree, target):
            names = _referenced_names(assignment)
            if not names:
                continue  # e.g. `delayed_node_actions = []`, extended below.
            checked += 1
            assert "_sim_clock_transport" in names or {
                "bridge", "sim_clock_relay"
            } <= names, (
                f"{target} = [{', '.join(sorted(names))}] brings up simulation "
                "nodes without the full clock transport; a bench with no /clock "
                "reports every controller active while freezing ROS time at 0."
            )
    assert checked >= 2, "expected both the full and the bench-minimal node sets"


def test_bridge_is_never_launched_without_the_relay():
    """Guards the specific regression: bridge kept, relay dropped."""
    tree = _module()
    for node in ast.walk(tree):
        if not isinstance(node, ast.Assign) or not isinstance(node.value, ast.List):
            continue
        names = _referenced_names(node.value)
        if "bridge" in names:
            assert "sim_clock_relay" in names, (
                "a node list contains `bridge` without `sim_clock_relay`: "
                "/clock_raw would be published with nothing relaying it to /clock"
            )
