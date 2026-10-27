#!/usr/bin/env bash
set -euo pipefail

OPENSCAD=${OPENSCAD:-/snap/bin/openscad-nightly}
src="cad/modular-test-chassis/p277-torsion-node.scad"
out="cad/modular-test-chassis/stl-p277"
mkdir -p "$out"

for part_name in torsion_centre_node brace_drill_jig mpu6050_rail_side_mount; do
  "$OPENSCAD" -o "$out/$part_name.stl" -D "part=\"$part_name\"" "$src"
done
