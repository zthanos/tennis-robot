#!/usr/bin/env bash
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source_scad="$here/shared-flywheel-p273-bolted-ski.scad"
release_dir="$here/stl-ski-mount"
openscad_bin="${OPENSCAD_BIN:-/snap/bin/openscad-nightly}"

if [[ "${DRY_RUN:-0}" == "1" ]]; then
  out_dir="$(mktemp -d "$here/.dry-run-ski.XXXXXX")"
  trap 'rm -rf "$out_dir"' EXIT
else
  out_dir="$release_dir"
  mkdir -p "$out_dir"
fi

parts=(ski bracket_left bracket_right boss_spacer)
outputs=(r75-75-ski x300-ski-bracket-left x300-ski-bracket-right ski-boss-spacer)
for i in "${!parts[@]}"; do
  "$openscad_bin" -q -o "$out_dir/${outputs[$i]}.stl" \
    -D "p273_part=\"${parts[$i]}\"" "$source_scad"
  [[ -s "$out_dir/${outputs[$i]}.stl" ]] || {
    echo "Missing ski export: ${outputs[$i]}" >&2; exit 1;
  }
done

echo "Validated ${#parts[@]} active P2-73 ski-mount meshes"
