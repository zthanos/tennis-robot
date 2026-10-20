#!/usr/bin/env bash
# Concept D renders on the real v3 robot (PNG only, no STL).
# Full Manifold render so cut faces keep their colours.
# Iso: front-right (camera +X/-Y) and rear-left (camera -X/+Y).
# Side: from -Y, +X (FRONT) to the right, +Z up.  Top: +X right, +Y up.
set -euo pipefail
cd "$(dirname "$0")"
O="${OPENSCAD_BIN:-/snap/bin/openscad-nightly}"
SCAD=body-panels-v3.scad
mkdir -p renders-v3
r() { local out=$1 cam=$2 proj=$3; shift 3
  "$O" -q --render --backend=Manifold -o "renders-v3/$out.png" --imgsize=1600,1000 \
    --colorscheme="Tomorrow Night" --projection="$proj" --camera="$cam" "$@" "$SCAD"; }
ISO_FR=150,0,150,62,0,35,2750
ISO_RL=150,0,150,62,0,215,2750
SIDE=150,0,190,90,0,0,2400
TOP=150,0,0,0,0,0,2300
for s in curved faceted; do
  r "D-$s-iso-front-right" $ISO_FR p -D "style=\"$s\""
  r "D-$s-iso-rear-left"   $ISO_RL p -D "style=\"$s\""
  r "D-$s-side"            $SIDE   o -D "style=\"$s\"" -D 'bp_view="side"'
  r "D-$s-top"             $TOP    o -D "style=\"$s\"" -D 'show_turn_circle=true'
  r "D-$s-panels-only"     $ISO_FR p -D "style=\"$s\"" -D 'show_context=false'
done
# E = flow (h3li0-inspired lines on D curved): drift line, pill doors, helix gills
r "E-flow-iso-front-right" $ISO_FR p -D 'style="flow"'
r "E-flow-iso-rear-left"   $ISO_RL p -D 'style="flow"'
r "E-flow-side"            $SIDE   o -D 'style="flow"' -D 'bp_view="side"'
r "E-flow-top"             $TOP    o -D 'style="flow"' -D 'show_turn_circle=true'
r "E-flow-panels-only"     $ISO_FR p -D 'style="flow"' -D 'show_context=false'
r "E-flow-detail-sill"     60,-260,180,78,0,20,900 p -D 'style="flow"' -D 'show_context=false'
for a in -3 0 20; do
  r "D-curved-section-${a}deg" $SIDE o -D 'style="curved"' -D 'bp_view="side_section"' -D "bp_angle=$a"
done
# keep-outs are %-background solids: preview mode so they stay translucent
"$O" -q -o renders-v3/D-curved-keepouts-20deg.png --imgsize=1600,1000 --colorscheme="Tomorrow Night" \
  --projection=p --camera=$ISO_FR -D 'style="curved"' -D 'show_keepouts=true' -D 'bp_angle=20' "$SCAD"
r "D-curved-console-conflict" $ISO_FR p -D 'style="curved"' -D 'show_console=true'
echo "renders written to $(pwd)/renders-v3"
