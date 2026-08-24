"""No module may define the same top-level name twice.

A scripted edit once duplicated a whole block of RosService methods instead of
replacing it. Python silently kept the LAST definition, so a later edit to the
first copy had no effect at runtime while `grep` still found the new text and
every unit test passed — the tests inject a fake robot port, so they never
exercised the shadowed methods. The live session failed with
"unexpected keyword argument 'publish_at_unix'" against source that plainly
contained it.

This is a cheap structural check over the packages that actually ship.
"""

from __future__ import annotations

import ast
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
PACKAGES = (
    ROOT / "scripts" / "tennis_robot",
    ROOT / "ros2_ws" / "src" / "tennis_robot" / "tennis_robot",
    ROOT / "scripts" / "sim_debug",
)


def _python_files() -> list[Path]:
    files: list[Path] = []
    for package in PACKAGES:
        files.extend(sorted(p for p in package.rglob("*.py")
                            if "__pycache__" not in p.parts))
    return files


def _duplicate_names(body) -> set[str]:
    seen: dict[str, int] = {}
    for node in body:
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
            seen[node.name] = seen.get(node.name, 0) + 1
    # A conditional re-definition is legitimate; a straight repeat in the same
    # body is the shadowing bug.
    return {name for name, count in seen.items() if count > 1}


@pytest.mark.parametrize("path", _python_files(), ids=lambda p: str(p.relative_to(ROOT)))
def test_module_defines_each_name_once(path):
    tree = ast.parse(path.read_text(encoding="utf-8"), str(path))
    module_dups = _duplicate_names(tree.body)
    assert not module_dups, f"{path.name} defines {sorted(module_dups)} more than once"
    for node in ast.walk(tree):
        if isinstance(node, ast.ClassDef):
            class_dups = _duplicate_names(node.body)
            assert not class_dups, (
                f"{path.name}:{node.name} defines {sorted(class_dups)} more than once; "
                "the later definition silently shadows the earlier one"
            )
