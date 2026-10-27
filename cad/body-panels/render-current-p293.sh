#!/usr/bin/env bash
# Current P2-93 report renders. PNG only; never exports print geometry.
set -euo pipefail
cd "$(dirname "$0")"
O="${OPENSCAD_BIN:-/snap/bin/openscad-nightly}"
SCAD=current-design-p293.scad
OUT=renders-current-p293
mkdir -p "$OUT"

render() {
  local name=$1 camera=$2 projection=$3
  shift 3
  "$O" -q --render --backend=Manifold -o "$OUT/$name.png" \
    --imgsize=1800,1200 --colorscheme="Tomorrow Night" \
    --projection="$projection" --camera="$camera" "$@" "$SCAD"
}

render current-p293-hero-terracotta \
  80,-20,170,67,0,32,2550 p \
  -D 'view="hero"' -D 'angle=20' -D 'colorway="terracotta"'
render current-p293-section-collection--3deg \
  80,0,180,90,0,0,2200 o \
  -D 'view="section"' -D 'angle=-3' -D 'colorway="terracotta"'
render current-p293-section-launch-20deg \
  80,0,180,90,0,0,2200 o \
  -D 'view="section"' -D 'angle=20' -D 'colorway="terracotta"'
render current-p293-rear-service-open \
  -80,0,185,72,0,210,2250 p \
  -D 'view="rear_service"' -D 'angle=0' -D 'colorway="terracotta"'

echo "renders written to $(pwd)/$OUT"
