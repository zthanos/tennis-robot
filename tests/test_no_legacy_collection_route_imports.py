"""Static guard for the Phase 6D.5 legacy collection-route removal."""

from __future__ import annotations

import ast
from pathlib import Path


_REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
_SEARCH_ROOTS = (
    _REPOSITORY_ROOT / "ros2_ws" / "src",
    _REPOSITORY_ROOT / "scripts",
    _REPOSITORY_ROOT / "tests",
)
_FORBIDDEN_MODULES = {
    "tennis_robot.collect_route_mission",
    "tennis_robot.collection_route_planner",
}
_FORBIDDEN_MODULE_NAMES = {"collect_route_mission", "collection_route_planner"}


def _is_forbidden_module(module: str) -> bool:
    return module in _FORBIDDEN_MODULES or module in _FORBIDDEN_MODULE_NAMES


def _legacy_imports(path: Path) -> list[str]:
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    findings: list[str] = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for alias in node.names:
                if _is_forbidden_module(alias.name):
                    findings.append(f"line {node.lineno}: import {alias.name}")
        elif isinstance(node, ast.ImportFrom):
            module = node.module or ""
            imported_names = {alias.name for alias in node.names}
            if _is_forbidden_module(module):
                findings.append(f"line {node.lineno}: from {module} import ...")
            if module == "tennis_robot" and imported_names & _FORBIDDEN_MODULE_NAMES:
                findings.append(
                    f"line {node.lineno}: imports legacy module from tennis_robot"
                )
            if "CollectRouteMission" in imported_names:
                findings.append(
                    f"line {node.lineno}: imports forbidden CollectRouteMission"
                )
    return findings


def test_no_python_source_imports_legacy_collection_route() -> None:
    findings: list[str] = []
    for search_root in _SEARCH_ROOTS:
        for path in search_root.rglob("*.py"):
            for detail in _legacy_imports(path):
                findings.append(f"{path.relative_to(_REPOSITORY_ROOT)}: {detail}")

    assert findings == [], "legacy collection-route imports remain:\n" + "\n".join(findings)
