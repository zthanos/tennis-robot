#!/usr/bin/env bash
set -euo pipefail

DISPLAY_SIZE="${DISPLAY_SIZE:-1280x800x24}"
SCAD_FILE="${OPENSCAD_FILE:-/workspace/cad/3d-printable-base/full_robot_concept.scad}"

cleanup() {
  pkill -f "Xvfb ${DISPLAY}" >/dev/null 2>&1 || true
  rm -f "/tmp/.X${DISPLAY#:}-lock"
}

trap cleanup EXIT
cleanup

Xvfb "${DISPLAY}" -screen 0 "${DISPLAY_SIZE}" &

fluxbox >/tmp/fluxbox.log 2>&1 &
x11vnc -display "${DISPLAY}" -forever -shared -nopw -rfbport 5900 >/tmp/x11vnc.log 2>&1 &
websockify --web=/usr/share/novnc 6080 localhost:5900 >/tmp/novnc.log 2>&1 &

echo "OpenSCAD noVNC is available at http://localhost:6081/vnc.html"
if [[ -f "${SCAD_FILE}" ]]; then
  echo "Opening: ${SCAD_FILE}"
  exec openscad "${SCAD_FILE}"
fi

echo "File not found: ${SCAD_FILE}"
echo "Starting OpenSCAD without a preloaded file."
exec openscad
