#!/usr/bin/env bash
# Concept renders only (PNG).  No STL is written.
# Iso views: front-right (camera +X/-Y) and rear-left (camera -X/+Y).
# Side views look from -Y: +X to the right (FRONT), +Z up.  Top: +X right, +Y up.
set -euo pipefail
cd "$(dirname "$0")"
O="${OPENSCAD_BIN:-/snap/bin/openscad-nightly}"
SCAD=body-panels-concepts.scad
CS="Tomorrow Night"
mkdir -p renders
r() { # out camera projection extra-defines...
  local out=$1 cam=$2 proj=$3; shift 3
  "$O" -q -o "renders/$out.png" --imgsize=1600,1000 --colorscheme="$CS" \
    --projection="$proj" --camera="$cam" "$@" "$SCAD"
}
ISO_FR=150,0,140,62,0,35,2700
ISO_RL=150,0,140,62,0,215,2700
SIDE=150,0,170,90,0,0,2300
TOP=150,0,0,0,0,0,2300
for c in A B C; do
  r "concept-$c-iso-front-right" $ISO_FR p -D "concept=\"$c\""
  r "concept-$c-iso-rear-left"   $ISO_RL p -D "concept=\"$c\""
  r "concept-$c-side"            $SIDE   o -D "concept=\"$c\"" -D 'bp_view="side"'
  r "concept-$c-top"             $TOP    o -D "concept=\"$c\""
done
# Recommended concept B: three operating states, side view.
for a in -3 0 20; do
  r "B-section-${a}deg" $SIDE o -D 'concept="B"' -D 'bp_view="side_section"' -D "bp_angle=$a"
done
# B: panels only (exploded-style without context), temporary zones, keep-outs, sweep overlay.
r "B-panels-only-iso" $ISO_FR p -D 'concept="B"' -D 'show_context=false'
r "B-panels-only-rear" $ISO_RL p -D 'concept="B"' -D 'show_context=false'
r "B-zones-iso"     $ISO_FR p -D 'concept="B"' -D 'bp_view="zones"'
r "B-keepouts-section-20deg" $SIDE o -D 'concept="B"' -D 'bp_view="side_section"' -D 'show_keepouts=true' -D 'bp_angle=20'
r "B-keepouts-iso"  $ISO_FR p -D 'concept="B"' -D 'bp_view="keepouts"' -D 'bp_angle=20'
r "B-sweep-iso"     $ISO_FR p -D 'concept="B"' -D 'bp_view="sweep"'
echo "renders written to $(pwd)/renders"
