#!/usr/bin/env bash
# P2-85 replacement for export-ski-mount-stls.sh: exports the active ski-mount
# set and fails unless every STL is exactly one body.
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source_scad="$here/shared-flywheel-p273-bolted-ski.scad"
release_dir="$here/stl-ski-mount"
openscad_bin="${OPENSCAD_BIN:-/snap/bin/openscad-nightly}"
component_check="$here/../check-stl-components.py"

if [[ "${DRY_RUN:-0}" == "1" ]]; then
  out_dir="$(mktemp -d "$here/.dry-run-ski.XXXXXX")"
  trap 'rm -rf "$out_dir"' EXIT
else
  out_dir="$release_dir"
  mkdir -p "$out_dir"
fi

parts=(ski print_p285_bracket_left print_p285_bracket_right
       print_p285_fork_left print_p285_fork_right)
outputs=(r75-75-ski x300-ski-bracket-left x300-ski-bracket-right
         p285-ski-wall-fork-left p285-ski-wall-fork-right)
for i in "${!parts[@]}"; do
  "$openscad_bin" -q --backend=manifold -o "$out_dir/${outputs[$i]}.stl" \
    -D "p273_part=\"${parts[$i]}\"" "$source_scad"
  [[ -s "$out_dir/${outputs[$i]}.stl" ]] || {
    echo "Missing ski export: ${outputs[$i]}" >&2; exit 1;
  }
  python3 "$component_check" "$out_dir/${outputs[$i]}.stl" 1
done

bash "$here/../check-active-chassis-hole-alignment.sh" ski_brackets

echo "Validated ${#parts[@]} active P2-85 ski-mount meshes"
