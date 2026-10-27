#!/usr/bin/env bash
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source_scad="$here/p293-arch-tilt-release.scad"
release_dir="$here/stl"
openscad_bin="${OPENSCAD_BIN:-/snap/bin/openscad-nightly}"
component_check="$here/../check-stl-components.py"

parts=(
  arch_foot_x240 arch_foot_x300 arch_leg_lower arch_leg_upper arch_joint_sleeve
  arch_brace_x240 arch_brace_x300 arch_crossbar_center arch_crossbar_end
  lidar_mast lidar_crossbar_half sensor_joint_sleeve oak_universal_plate
  tilt_fit0186_carrier tilt_kfl08_base tilt_nut_carriage tilt_stop
  tilt_as5600_pod tilt_as5600_cover
  tilt_magnet_cap
)

if [[ "${DRY_RUN:-0}" == "1" ]]; then
  out_dir="$(mktemp -d "$here/.dry-run.XXXXXX")"
  trap 'rm -rf "$out_dir"' EXIT
else
  out_dir="$release_dir"
  mkdir -p "$out_dir"
fi

if (($#)); then parts=("$@"); fi

for name in "${parts[@]}"; do
  "$openscad_bin" --backend=Manifold -o "$out_dir/$name.stl" \
    -D "part=\"$name\"" "$source_scad"
  [[ -s "$out_dir/$name.stl" ]] || { echo "P2-93 missing STL: $name" >&2; exit 1; }
  python3 "$component_check" "$out_dir/$name.stl" 1
done

# Permanent P2-90e lesson: no large horizontal underside above the bed.
for mesh in "$out_dir"/*.stl; do
  awk -v mesh="$mesh" '
    /facet normal/ { nz=$5; vi=0; next }
    /vertex/ { vi++; x[vi]=$2; y[vi]=$3; z[vi]=$4 }
    /endfacet/ {
      if (nz < -0.99 && vi == 3) {
        za=(z[1]+z[2]+z[3])/3
        if (za > 0.2) {
          ax=x[2]-x[1]; ay=y[2]-y[1]
          bx=x[3]-x[1]; by=y[3]-y[1]
          ar=(ax*by-ay*bx)/2; if(ar<0) ar=-ar
          key=sprintf("%.1f",za); area[key]+=ar
        }
      }
    }
    END {
      max=0; maxz=0
      for(key in area) if(area[key]>max){max=area[key];maxz=key}
      if(max>40){printf "PRINTABILITY FAIL: %s underside %.2f mm2 at Z%s\n",mesh,max,maxz > "/dev/stderr"; exit 1}
      printf "PRINTABILITY PASS: %s max underside %.2f mm2 at Z%s\n",mesh,max,maxz
    }
  ' "$mesh"
done

python3 "$here/p293-stl-stats.py" "$out_dir" "$here/p293-print-estimate.json"
bash "$here/qa-p293.sh"
echo "P2-93 export PASS: ${#parts[@]} one-body STL"
