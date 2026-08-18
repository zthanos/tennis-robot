"""Test 12 — static guard: the adaptive generator has NO live wiring.

The adaptive-approach modules are shadow / offline only.  This AST guard fails
if any *live* collection-route module (the pure planner pipeline, the executor,
the node factory, or the controller node) imports them, which would risk the
adaptive candidates leaking into ``plan_collection_route`` or the runtime.

Only the shadow modules themselves, the offline ``scripts/sim_debug`` CLI, and
the tests may import them.
"""

from __future__ import annotations

import ast
from pathlib import Path

_REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
_PACKAGE_DIR = _REPOSITORY_ROOT / "ros2_ws" / "src" / "tennis_robot" / "tennis_robot"

# Modules that must NEVER be imported into live code.  The execution evaluator
# joins them because it reasons with the *physical* capture planes -- wider than
# the corridor the planner allows itself -- and using those to plan would
# quietly widen the planner's own assumptions.  Recording is separate and does
# stay live: collection_execution_trace and collection_execution_recorder carry
# no geometry at all.
_ADAPTIVE_MODULES = {
    "collection_adaptive_approach",
    "collection_capture_geometry",
    "collection_execution_evaluator",
}
# The only source files (outside tests / scripts) allowed to import them: the
# offline modules themselves.
_ALLOWED_SOURCE_FILES = {
    "collection_adaptive_approach.py",
    "collection_capture_geometry.py",
    "collection_execution_evaluator.py",
}


def _imports_adaptive(path: Path) -> list[str]:
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    findings: list[str] = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for alias in node.names:
                tail = alias.name.rsplit(".", 1)[-1]
                if tail in _ADAPTIVE_MODULES:
                    findings.append(f"line {node.lineno}: import {alias.name}")
        elif isinstance(node, ast.ImportFrom):
            module = node.module or ""
            tail = module.rsplit(".", 1)[-1]
            if tail in _ADAPTIVE_MODULES:
                findings.append(f"line {node.lineno}: from {module} import ...")
    return findings


def test_no_live_module_imports_the_adaptive_generator() -> None:
    findings: list[str] = []
    for path in _PACKAGE_DIR.rglob("*.py"):
        if path.name in _ALLOWED_SOURCE_FILES:
            continue
        for detail in _imports_adaptive(path):
            findings.append(f"{path.relative_to(_REPOSITORY_ROOT)}: {detail}")
    assert findings == [], (
        "adaptive shadow modules are wired into live code:\n" + "\n".join(findings)
    )


def test_live_planner_does_not_reference_adaptive_symbols() -> None:
    # Belt-and-braces: the sole planner orchestration entry point must not even
    # mention the adaptive API by name.
    planner = (_PACKAGE_DIR / "collection_route_planner_v2.py").read_text(encoding="utf-8")
    assert "adaptive" not in planner.lower()
    assert "capture_geometry" not in planner.lower()
