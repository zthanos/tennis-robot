#!/usr/bin/env bash
set -euo pipefail

WORLD_FILE="${WEBOTS_WORLD:-/workspace/worlds/tennis_court.wbt}"
DISPLAY_SIZE="${DISPLAY_SIZE:-1280x800x24}"

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

echo "Webots noVNC is available at http://localhost:6080/vnc.html"
echo "Opening world: ${WORLD_FILE}"

webots --stdout --stderr --mode=fast "${WORLD_FILE}"
