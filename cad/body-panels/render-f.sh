#!/usr/bin/env bash
# Concept F (planter tub) renders, PNG only.  Side views from -Y, +X right, +Z up.
set -euo pipefail
cd "$(dirname "$0")"
O="${OPENSCAD_BIN:-/snap/bin/openscad-nightly}"
SCAD=body-panels-f.scad
mkdir -p renders-f
r() { local out=$1 cam=$2 proj=$3; shift 3
  "$O" -q --render --backend=Manifold -o "renders-f/$out.png" --imgsize=1600,1000 \
    --colorscheme="Tomorrow Night" --projection="$proj" --camera="$cam" "$@" "$SCAD"; }
ISO_FR=150,0,150,62,0,35,2750
ISO_RL=150,0,150,62,0,215,2750
SIDE=150,0,190,90,0,0,2400
TOP=150,0,0,0,0,0,2300
for c in terracotta bone; do
  r "F-$c-iso-front-right" $ISO_FR p -D "colorway=\"$c\""
  r "F-$c-iso-rear-left"   $ISO_RL p -D "colorway=\"$c\""
  r "F-$c-side"            $SIDE   o -D "colorway=\"$c\"" -D 'bp_view="side"'
done
r "F-terracotta-top"         $TOP    o -D 'show_turn_circle=true'
r "F-terracotta-panels-only" $ISO_FR p -D 'show_context=false'
r "F-terracotta-panels-rear" $ISO_RL p -D 'show_context=false'
r "F-terracotta-detail"      40,-250,230,75,0,25,1000 p
r "F-terracotta-front"       150,0,180,90,0,90,2100 o
r "F-terracotta-front-detail" 520,-120,250,80,0,60,900 p
for a in -3 0 20; do
  r "F-section-${a}deg" $SIDE o -D 'bp_view="side_section"' -D "bp_angle=$a"
done
echo "renders written to $(pwd)/renders-f"
