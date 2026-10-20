#!/usr/bin/env bash
# P2-85 exact-solid probes: every listed part must render EMPTY.
set -uo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
O="${OPENSCAD_BIN:-/snap/bin/openscad-nightly}"
if [[ -n "${P285_OUT:-}" ]]; then
  out="$P285_OUT"; mkdir -p "$out"
else
  out="$(mktemp -d "$here/.p285-qa.XXXXXX")"
  trap 'rm -rf "$out"' EXIT
fi
src="$here/shared-flywheel-p273-bolted-ski.scad"
panel_src="$here/p285-body-panels-qa.scad"
fail=0
run() { # name extra-defines...
  local name="$1"; shift
  local tag="$name$(printf '_%s' "$@" | tr -c 'A-Za-z0-9_' '_')"
  local log="$out/$tag.log"
  "$O" --backend=manifold -o "$out/$tag.stl" -D "p273_part=\"$name\"" "$@" "$src" >"$log" 2>&1
  if grep -Eq "ERROR:|Parser error|Ignoring unknown module" "$log"; then
    echo "ERROR  $name $*"; tail -5 "$log"; fail=1
  elif grep -q "top level object is empty" "$log" || { [[ -s "$out/$tag.stl" ]] && python3 "$here/p285-stl-stats.py" "$out/$tag.stl" | grep -q "vol=-\?0\.0[0-9]* "; }; then
    echo "EMPTY  $name $*"
  elif [[ -s "$out/$tag.stl" ]]; then
    echo "SOLID  $name $*  $(python3 "$here/p285-stl-stats.py" "$out/$tag.stl" | cut -d: -f2)"; fail=1
  else
    echo "ERROR  $name $*"; tail -3 "$log"; fail=1
  fi
}
for g in ${P285_GROW:-0}; do
  G=(-D "p285_grow=$g")
  run qa_p285_ski_bracket "${G[@]}"
  run qa_p285_fast_bracket "${G[@]}"
  run qa_p285_fork_ski "${G[@]}"
  run qa_p285_fork_bracket "${G[@]}"
  for a in 15 20; do
    run qa_adjust_selector "${G[@]}" -D "p273_angle=$a"
    run qa_p285_bracket_selector_coupled "${G[@]}" -D "p273_angle=$a"
  done
  run qa_adjust_hard_coupled "${G[@]}"
  run qa_p285_throat_coupled "${G[@]}"
  run qa_p285_launcher_coupled "${G[@]}"
  run qa_p285_bracket_drive "${G[@]}"
  run qa_p285_bracket_feed "${G[@]}"
  run qa_adjust_drive "${G[@]}"
  run qa_adjust_closed_feed "${G[@]}"
  [[ "$g" == 0 ]] && {
    run qa_p285_bracket_frame
    run qa_p285_fast_frame
    run qa_p285_fork_frame
    run qa_adjust_frame_unintended
    run qa_ground
  }
done

run_panel() {
  local name="$1" log="$out/body-panels-$1.log" stl="$out/body-panels-$1.stl"
  "$O" --backend=manifold -o "$stl" -D "p285_panel_qa=\"$name\"" \
    "$panel_src" >"$log" 2>&1
  if grep -Eq "ERROR:|Parser error|Ignoring unknown module" "$log"; then
    echo "ERROR  body_panels_$name"; tail -5 "$log"; fail=1
  elif grep -q "top level object is empty" "$log"; then
    echo "EMPTY  body_panels_$name"
  elif [[ -s "$stl" ]]; then
    echo "SOLID  body_panels_$name  $(python3 "$here/p285-stl-stats.py" "$stl" | cut -d: -f2)"
    fail=1
  else
    echo "ERROR  body_panels_$name"; tail -3 "$log"; fail=1
  fi
}
run_panel rear
run_panel concept_f
bash "$here/../check-active-chassis-hole-alignment.sh" ski_brackets
exit $fail
