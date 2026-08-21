#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_dir="$(cd "${script_dir}/../.." && pwd)"
output_dir="${script_dir}/stl"

parts=(cheek_rear cheek_front cheek_joiner ramp rail_saddle rail_cap)
mkdir -p "${output_dir}"

for part_name in "${parts[@]}"; do
  docker compose --project-directory "${repo_dir}" --profile cad run --rm \
    --user "$(id -u):$(id -g)" openscad \
    openscad -D "part=\"${part_name}\"" --export-format binstl \
    -o "cad/collector-intake-v1/stl/${part_name}.stl" \
    cad/collector-intake-v1/intake-structure.scad
done

echo "Exported ${#parts[@]} intake structure STL files to ${output_dir}"
