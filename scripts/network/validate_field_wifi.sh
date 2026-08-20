#!/usr/bin/env bash
# Read-only software/on-Pi checks. Client/phone checks remain manual.
set -euo pipefail
export LC_ALL=C

CONNECTION="${FIELD_WIFI_CONNECTION:-tennis-robot-field-ap}"
CONSOLE_PORT="${TENNIS_CONSOLE_PORT:-8081}"
FOXGLOVE_PORT="${TENNIS_FOXGLOVE_PORT:-8765}"
FAILURES=0
INTERFACE="$(nmcli -g connection.interface-name connection show "$CONNECTION" 2>/dev/null || true)"
CONFIGURED_ADDRESS="$(nmcli -g ipv4.addresses connection show "$CONNECTION" 2>/dev/null | cut -d, -f1 || true)"

port_is_non_loopback() {
    local port="$1"
    ss -H -ltn "sport = :$port" 2>/dev/null | awk '{print $4}' | \
        grep -Eq "^(0\\.0\\.0\\.0|\\[::\\]|\\*):${port}$"
}

configured_address_is_live() {
    [ -n "$INTERFACE" ] && [ -n "$CONFIGURED_ADDRESS" ] &&
        ip -4 -o address show dev "$INTERFACE" scope global 2>/dev/null | \
            awk -v wanted="$CONFIGURED_ADDRESS" '$4 == wanted {found=1} END {exit !found}'
}

security_is_protected() {
    case "$(nmcli -g 802-11-wireless-security.key-mgmt connection show "$CONNECTION" 2>/dev/null)" in
        wpa-psk|sae) return 0 ;;
        *) return 1 ;;
    esac
}

check() {
    local description="$1"; shift
    if "$@" >/dev/null 2>&1; then
        printf 'PASS  %s\n' "$description"
    else
        printf 'FAIL  %s\n' "$description"
        FAILURES=$((FAILURES + 1))
    fi
}

check "NetworkManager active" systemctl is-active --quiet NetworkManager
check "field AP profile exists" nmcli connection show "$CONNECTION"
check "field AP profile active" nmcli connection show --active "$CONNECTION"
check "field AP mode is ap" test "$(nmcli -g 802-11-wireless.mode connection show "$CONNECTION" 2>/dev/null)" = ap
check "field AP uses shared IPv4/DHCP" test "$(nmcli -g ipv4.method connection show "$CONNECTION" 2>/dev/null)" = shared
check "field AP autoconnect enabled" test "$(nmcli -g connection.autoconnect connection show "$CONNECTION" 2>/dev/null)" = yes
check "field AP uses WPA2/WPA3 protection" security_is_protected
check "configured AP address is assigned" configured_address_is_live
check "operator console HTTP responds" curl --fail --silent --max-time 3 "http://127.0.0.1:${CONSOLE_PORT}/api/status"
check "operator console listens beyond loopback" port_is_non_loopback "$CONSOLE_PORT"

if ss -H -ltn "sport = :$FOXGLOVE_PORT" 2>/dev/null | grep -q .; then
    check "Foxglove WebSocket listens beyond loopback" port_is_non_loopback "$FOXGLOVE_PORT"
else
    printf 'SKIP  Foxglove WebSocket is optional and not running\n'
fi

"$(dirname "$0")/field_wifi_status.sh"
exit "$FAILURES"
