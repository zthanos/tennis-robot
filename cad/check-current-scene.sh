#!/usr/bin/env bash
# Mandatory P2-95 full-scene gate. scene_check re-exports every body whenever
# any CAD source changes (the cache is keyed on a digest of all sources).
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/independent-check"
cd "$here"
exec uv run --with manifold3d --with trimesh --with numpy --with scipy \
  python scene_check.py --jobs "${SCENE_CHECK_JOBS:-6}"
