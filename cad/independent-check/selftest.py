#!/usr/bin/env python3
"""Regression test for scadtree: a part reached through a single-statement
module (`module m() stmt;`) and through a block module must both be found.

  uv run python selftest.py
"""
import shutil
import subprocess
from pathlib import Path

import scadtree
from scene_check import OPENSCAD

BASE = Path(__file__).resolve().parent / "out" / "selftest"
SRC = """module part_a() cube(10);
module wrap_one() translate([20,0,0]) part_a();
module wrap_block() { translate([40,0,0]) part_a(); }
module scene() { wrap_one(); wrap_block(); }
part="x";
if(part=="part_a") part_a();
"""

shutil.rmtree(BASE, ignore_errors=True)
(BASE / "cad" / "a").mkdir(parents=True)
(BASE / "cad" / "a" / "t.scad").write_text(SRC)
(BASE / "cad" / "a" / "part_a.stl").touch()
parts, _, helpers = scadtree.mirror(BASE / "cad", BASE / "src")
wrapper = BASE / "w.scad"
wrapper.write_text(f"use <{BASE / 'src/a/t.scad'}>\n$p=[];$disc=true;$sel=undef;\nscene();\n")
subprocess.run([OPENSCAD, "-o", str(BASE / "w.echo"), str(wrapper)], capture_output=True)
stmts, mods = scadtree.parse_echo((BASE / "w.echo").read_text())
found = scadtree.bodies(stmts, mods, parts, helpers)
assert [n for n, _ in found] == ["part_a", "part_a"], found
assert all(len(p) == 2 for _, p in found), found
print("scadtree selftest PASS:", found)
