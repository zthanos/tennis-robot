#!/usr/bin/env python3
"""Reject QA masks that subtract geometry from a solid intersection.

The mechanical rule is intentionally simple: an exact collision gate operates
on the released/installed solids themselves. Coplanar contact may have zero
volume; no box, mating zone, fastener zone, or declared interface may be
removed before the boolean intersection is measured.
"""
from __future__ import annotations

import re
from pathlib import Path

ROOT = Path(__file__).resolve().parent
SKIP = {"archive", "out"}


def active_scad_files():
    for path in ROOT.rglob("*.scad"):
        if any(part in SKIP for part in path.parts):
            continue
        yield path


def strip_comments(text: str) -> str:
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.S)
    return re.sub(r"//[^\n]*", "", text)


violations: list[str] = []
set_difference_gates = {"qa_cup_ligament", "qa_load_spines", "qa_p288_reservation_overflow"}
for path in active_scad_files():
    raw = path.read_text(errors="replace")
    text = strip_comments(raw)

    # Direct form: difference() { intersection() { ... } mask; }
    if "qa" in path.name.lower() or "exact" in path.name.lower():
        for match in re.finditer(r"difference\s*\(\s*\)\s*\{\s*intersection\s*\(", text):
            line = text.count("\n", 0, match.start()) + 1
            violations.append(f"{path.relative_to(ROOT)}:{line}: intersection wrapped in difference()")

    # Helper form that hid P2-93: a QA-named module whose top-level result is
    # a difference, later passed into an intersection.
    for match in re.finditer(
        r"module\s+([A-Za-z0-9_]*qa[A-Za-z0-9_]*)\s*\([^)]*\)\s*\{\s*difference\s*\(",
        text,
        flags=re.I,
    ):
        if match.group(1) in set_difference_gates:
            continue
        line = text.count("\n", 0, match.start()) + 1
        violations.append(
            f"{path.relative_to(ROOT)}:{line}: QA helper {match.group(1)} starts with difference()"
        )

if violations:
    print("EXACT_SOLID_POLICY FAIL")
    print("\n".join(f"  {item}" for item in violations))
    raise SystemExit(1)

print("EXACT_SOLID_POLICY PASS: no QA intersection masks in active SCAD")
