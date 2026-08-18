#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_dir="$(cd "${script_dir}/../.." && pwd)"
output_dir="${script_dir}/stl-segments"

parts=(
  floor_tile
  management_tray_tile
  side_wall_tile
  rear_wall_tile
  receiving_chute_tile
  side_flange_segment
  rear_flange_segment
  corner_guard
  center_lip
  carry_handle
  drop_strut
  joiner_plate
)

mkdir -p "${output_dir}"

for part_name in "${parts[@]}"; do
  # Compose normally runs this image as "nobody". Match the host user so the
  # generated files can be overwritten by the next export without sudo.
  rm -f "${output_dir}/${part_name}.stl"
  docker compose --project-directory "${repo_dir}" --profile cad run --rm \
    --user "$(id -u):$(id -g)" openscad \
    openscad -D "part=\"${part_name}\"" --export-format binstl \
    -o "cad/basket-bin-v2/stl-segments/${part_name}.stl" \
    cad/basket-bin-v2/print-segments.scad
done

echo "Exported ${#parts[@]} STL files to ${output_dir}"
