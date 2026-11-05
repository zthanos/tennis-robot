#!/usr/bin/env bash
# Mandatory P2-95 full-scene gate. Preserve tools and previous output.
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/independent-check"
# scene_check's cache key does not include CAD source content/mtime.
# Keep the old results recoverable and force fresh exports on every gate.
if [[ -d "$here/out/scene" ]]; then
  scene_backup="$(mktemp -d "$here/out/previous-scene.XXXXXX")"
  mv "$here/out/scene" "$scene_backup/scene"
  echo "Previous scene output retained: $scene_backup/scene"
fi
cd "$here"
exec uv run --with manifold3d --with trimesh --with numpy --with scipy \
  python scene_check.py --jobs "${SCENE_CHECK_JOBS:-6}"
