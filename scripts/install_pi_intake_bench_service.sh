#!/usr/bin/env bash
# Install/update the boot-persistent intake bench panel on the Raspberry Pi.
set -euo pipefail
export LC_ALL=C

if [ "${EUID}" -ne 0 ]; then
    echo "ERROR: run with sudo: sudo $0" >&2
    exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROOT_DIR="${TENNIS_ROBOT_ROOT:-$SCRIPT_DIR}"
SERVICE_USER="${TENNIS_ROBOT_SERVICE_USER:-${SUDO_USER:-}}"
if [ -z "$SERVICE_USER" ] || [ "$SERVICE_USER" = root ]; then
    echo "ERROR: set TENNIS_ROBOT_SERVICE_USER to the non-root Pi account." >&2
    exit 1
fi
SERVICE_GROUP="$(id -gn "$SERVICE_USER")"
TEMPLATE="$ROOT_DIR/config/systemd/tennis-robot-intake-bench.service.in"
TARGET="/etc/systemd/system/tennis-robot-intake-bench.service"

[ -f "$ROOT_DIR/scripts/intake_bench_panel.py" ] || {
    echo "ERROR: intake bench server missing under $ROOT_DIR" >&2
    exit 1
}
[ -f "$ROOT_DIR/scripts/intake_bench_panel.html" ] || {
    echo "ERROR: intake bench HTML missing under $ROOT_DIR" >&2
    exit 1
}
[ -f "$TEMPLATE" ] || { echo "ERROR: service template missing: $TEMPLATE" >&2; exit 1; }
case "$ROOT_DIR" in *[[:space:]@]*)
    echo "ERROR: repository path cannot contain whitespace or '@': $ROOT_DIR" >&2
    exit 1
esac

TEMP_FILE="$(mktemp)"
trap 'rm -f "$TEMP_FILE"' EXIT
sed \
    -e "s|@ROOT@|$ROOT_DIR|g" \
    -e "s|@USER@|$SERVICE_USER|g" \
    -e "s|@GROUP@|$SERVICE_GROUP|g" \
    "$TEMPLATE" > "$TEMP_FILE"
install -m 0644 "$TEMP_FILE" "$TARGET"

systemctl daemon-reload
systemctl enable --now tennis-robot-intake-bench.service
systemctl is-active --quiet tennis-robot-intake-bench.service
echo "Installed $TARGET"
echo "Intake bench: http://10.42.0.1:8082 (or the Pi LAN address on port 8082)"
