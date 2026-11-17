#!/usr/bin/env bash
# Common permanent gate for active printable STL assets.
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
mesh="${1:?usage: check-stl-printability.sh MESH [EXPECTED_COMPONENTS]}"
expected_components="${2:-1}"

python3 "$here/check-stl-components.py" "$mesh" "$expected_components"

downward_args=(--max-area 40)
case "$(basename "$mesh")" in
  crown_L.stl)
    downward_args+=(--allow-z "13.9:14.1:designed crown bridge")
    downward_args+=(--allow-z "19.9:20.1:designed crown bridge") ;;
  crown_R.stl)
    downward_args+=(--allow-z "5.9:6.1:designed crown bridge")
    downward_args+=(--allow-z "26.7:26.9:designed crown bridge") ;;
  lidar_half_L.stl|lidar_half_R.stl)
    downward_args+=(--allow-z "16.9:17.1:short tube bridge") ;;
  mast_cap_L.stl|mast_cap_R.stl)
    downward_args+=(--allow-z "13.9:14.1:24 mm cap bridge") ;;
  ramp_entry_launcher_pusher.stl)
    downward_args+=(--allow-z "3.9:4.1:pusher gusset bridge")
    downward_args+=(--allow-z "7.9:8.1:pusher runner bridge") ;;
esac
python3 "$here/check-stl-downward-faces.py" "$mesh" "${downward_args[@]}"

slice_args=(
  --layer-height 0.2 --start-z 0.5
  --min-thickness 1.6 --min-island-area 25
)

# These are horizontal-section artefacts at deliberately curved/oblique
# free edges, not thin load-bearing walls.  Every interval is documented by
# a section image and classification in printability-p297b/README.md.  Any
# failure outside an explicit interval remains fail-closed.
case "$(basename "$mesh")" in
  foot_L.stl|foot_R.stl)
    for range in 11.3:11.7 12.1:12.9 18.5:19.3 23.7:24.5 29.7:30.9 \
                 54.3:54.5 64.7:65.5 73.1:74.7 74.9:76.1 77.1:77.9 \
                 90.7:91.5 91.7:92.3 92.7:93.5 95.5:96.3 101.3:102.1 \
                 103.1:103.9 109.1:109.9 117.7:118.5 118.7:119.5 \
                 121.5:122.3 125.3:128.3; do
      slice_args+=(--allow-z "$range:oblique edge after support-free rotation")
    done
    ;;
  crown_L.stl) slice_args+=(--allow-z "18.9:19.9:transverse-bore exit edge") ;;
  crown_R.stl) slice_args+=(--allow-z "32.7:33.9:transverse-bore exit edge") ;;
  mast_cap_L.stl|mast_cap_R.stl)
    slice_args+=(--allow-z "3.9:8.1:elliptical spigot edge")
    slice_args+=(--allow-z "17.5:18.3:elliptical spigot edge")
    slice_args+=(--allow-z "29.7:30.5:elliptical spigot edge")
    ;;
  oak_mount_x1.stl)
    slice_args+=(--allow-z "0.5:0.7:rounded bed-edge tangent")
    slice_args+=(--allow-z "1.1:3.3:oblique gusset and plate edge transitions")
    slice_args+=(--allow-z "9.1:9.9:ear bore exit edges")
    slice_args+=(--allow-z "15.3:16.5:pointed strut tips")
    ;;
  ramp_entry_launcher_pusher.stl)
    slice_args+=(--allow-z "9.7:14.4:spherical cup free edge")
    ;;
  p297_mega_carrier.stl)
    slice_args+=(--allow-z "0.5:3.1:carrier chamfer/cutout edge")
    ;;
esac

python3 "$here/check-stl-slices.py" "$mesh" "${slice_args[@]}"
