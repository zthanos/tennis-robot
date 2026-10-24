#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
scad_file="$root_dir/rear-pod-v2.scad"
release_dir="$root_dir/stl-v2"
openscad_bin="${OPENSCAD_BIN:-/snap/bin/openscad-nightly}"
component_check="$root_dir/../check-stl-components.py"

if [[ "${DRY_RUN:-0}" == "1" ]]; then
  out_dir="$(mktemp -d "$root_dir/.dry-run-v2.XXXXXX")"
  trap 'rm -rf "$out_dir"' EXIT
else
  out_dir="$release_dir"
  mkdir -p "$out_dir"
fi

all_parts=(
  base_left base_right pi_shelf vertical_electronics_plate side_frame
  panel_posts corner_tie component_pins component_clips bts_spacers
  upright_mount_fit_coupon control_panel
  rear_foot_wedge_washer side_frame_base_drill_jig
  parkside_dock_left parkside_dock_right
  parkside_drawer_left parkside_drawer_right
  ugreen_post_holder ugreen_clamp_caps ugreen_rear_retainer
)
parts=("${all_parts[@]}")
if (($#)); then
  parts=("$@")
fi
declare -A expected_components=(
  [base_left]=1 [base_right]=1 [pi_shelf]=1
  [vertical_electronics_plate]=1 [side_frame]=1 [panel_posts]=4
  [corner_tie]=1 [component_pins]=27 [component_clips]=32
  [bts_spacers]=18 [upright_mount_fit_coupon]=2 [control_panel]=1
  [rear_foot_wedge_washer]=1 [side_frame_base_drill_jig]=1
  [parkside_dock_left]=1 [parkside_dock_right]=1
  [parkside_drawer_left]=1 [parkside_drawer_right]=1
  [ugreen_post_holder]=1 [ugreen_clamp_caps]=2 [ugreen_rear_retainer]=1
)

for part_name in "${parts[@]}"; do
  "$openscad_bin" -o "$out_dir/${part_name}.stl" \
    -D "part=\"${part_name}\"" "$scad_file"
  [[ -s "$out_dir/${part_name}.stl" ]] || {
    echo "Missing export: $part_name" >&2; exit 1;
  }
  python3 "$component_check" "$out_dir/${part_name}.stl" \
    "${expected_components[$part_name]}"
done

# P2-90d printability regression: in the upright release pose, no horizontal
# downward-facing layer above the bed may exceed 40 mm^2.  This catches the
# former 124.5 mm low bridge and the 18 mm flat pocket/cup shelves, while
# allowing the small tessellated ceilings of the Ø4.5/Ø7 transverse bores.
if [[ -f "$out_dir/side_frame.stl" ]]; then
  awk '
    /facet normal/ { nz=$5; vi=0; next }
    /vertex/ { vi++; x[vi]=$2; y[vi]=$3; z[vi]=$4 }
    /endfacet/ {
      if (nz < -0.99 && vi == 3) {
        za=(z[1]+z[2]+z[3])/3
        if (za > 0.2) {
          ax=x[2]-x[1]; ay=y[2]-y[1]
          bx=x[3]-x[1]; by=y[3]-y[1]
          ar=(ax*by-ay*bx)/2; if (ar < 0) ar=-ar
          key=sprintf("%.1f",za); area[key]+=ar
        }
      }
    }
    END {
      max=0; maxz=0
      for (key in area) if (area[key] > max) { max=area[key]; maxz=key }
      if (max > 40) {
        printf "PRINTABILITY FAIL: side_frame horizontal underside %.2f mm^2 at Z%s\n", max, maxz > "/dev/stderr"
        exit 1
      }
      printf "PRINTABILITY PASS: side_frame max horizontal underside %.2f mm^2 at Z%s\n", max, maxz
    }
  ' "$out_dir/side_frame.stl"
fi

# P2-75 cable parts remain active immutable release assets in stl-v2. They
# have no P2-81 geometry change and are intentionally not rewritten here.
for part_name in cable_bulkhead bulkhead_foot; do
  [[ -f "$release_dir/${part_name}.stl" ]] || {
    echo "Missing retained active asset: $release_dir/${part_name}.stl" >&2
    exit 1
  }
  python3 "$component_check" "$release_dir/${part_name}.stl" 1
done

echo "Validated ${#parts[@]} generated parts; retained cable parts present"
bash "$root_dir/../check-active-chassis-hole-alignment.sh" pod_base corner_ties
qa_side_frame="$release_dir/side_frame.stl"
qa_panel_posts="$release_dir/panel_posts.stl"
[[ -f "$out_dir/side_frame.stl" ]] && qa_side_frame="$out_dir/side_frame.stl"
[[ -f "$out_dir/panel_posts.stl" ]] && qa_panel_posts="$out_dir/panel_posts.stl"
P290C_SIDE_FRAME_STL="$qa_side_frame" P290C_PANEL_POSTS_STL="$qa_panel_posts" \
  bash "$root_dir/../check-active-assemblability.sh" p288
