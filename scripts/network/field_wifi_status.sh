#!/usr/bin/env bash
# Human-readable, non-secret diagnostics for the field AP.
set -euo pipefail
export LC_ALL=C

CONNECTION="${FIELD_WIFI_CONNECTION:-tennis-robot-field-ap}"
if ! command -v nmcli >/dev/null 2>&1; then
    echo "ERROR: nmcli is unavailable." >&2
    exit 1
fi
if ! nmcli -t -f NAME connection show | grep -Fxq "$CONNECTION"; then
    echo "Wi-Fi mode: not configured"
    echo "Profile: $CONNECTION"
    exit 2
fi

INTERFACE="$(nmcli -g connection.interface-name connection show "$CONNECTION")"
SSID="$(nmcli -g 802-11-wireless.ssid connection show "$CONNECTION")"
MODE="$(nmcli -g 802-11-wireless.mode connection show "$CONNECTION")"
ADDRESS="$(nmcli -g ipv4.addresses connection show "$CONNECTION")"
STATE="$(nmcli -g GENERAL.STATE device show "$INTERFACE" 2>/dev/null || printf 'unavailable')"
ACTIVE="no"
nmcli -t -f NAME connection show --active | grep -Fxq "$CONNECTION" && ACTIVE="yes"
LIVE_ADDRESS="$(ip -4 -o address show dev "$INTERFACE" scope global 2>/dev/null | awk '{print $4}' | paste -sd, -)"

printf 'Wi-Fi mode: %s\n' "$MODE"
printf 'Profile: %s\n' "$CONNECTION"
printf 'Interface: %s\n' "$INTERFACE"
printf 'SSID: %s\n' "$SSID"
printf 'Configured AP address: %s\n' "$ADDRESS"
printf 'Active: %s\n' "$ACTIVE"
printf 'Device state: %s\n' "$STATE"
printf 'Live IPv4 address: %s\n' "${LIVE_ADDRESS:-none}"
