#!/usr/bin/env bash
# Install/update the persistent NetworkManager field AP profile.
set -euo pipefail
export LC_ALL=C

usage() {
    cat <<'EOF'
Usage: sudo ./scripts/network/install_field_wifi_ap.sh [--config PATH] [--no-activate]

The config file defaults to /etc/tennis-robot/field-wifi.env and must not be
readable by group/other because it contains the Wi-Fi passphrase.
EOF
}

CONFIG_FILE="${FIELD_WIFI_CONFIG:-/etc/tennis-robot/field-wifi.env}"
ACTIVATE=true
while [ "$#" -gt 0 ]; do
    case "$1" in
        --config) [ "$#" -ge 2 ] || { usage >&2; exit 2; }; CONFIG_FILE="$2"; shift 2 ;;
        --no-activate) ACTIVATE=false; shift ;;
        -h|--help) usage; exit 0 ;;
        *) echo "ERROR: unknown argument: $1" >&2; usage >&2; exit 2 ;;
    esac
done

if [ "${EUID}" -ne 0 ]; then
    echo "ERROR: run with sudo (NetworkManager system profiles require root)." >&2
    exit 1
fi
if ! command -v nmcli >/dev/null 2>&1; then
    echo "ERROR: nmcli is unavailable; install/enable NetworkManager first." >&2
    exit 1
fi
if ! command -v dnsmasq >/dev/null 2>&1; then
    echo "ERROR: NetworkManager shared mode needs dnsmasq-base for DHCP/DNS." >&2
    echo "Install it with: sudo apt-get install dnsmasq-base" >&2
    exit 1
fi
if ! systemctl is-active --quiet NetworkManager; then
    echo "ERROR: NetworkManager is not active; refusing to install a profile for an inactive backend." >&2
    exit 1
fi
if [ ! -f "$CONFIG_FILE" ]; then
    echo "ERROR: config not found: $CONFIG_FILE" >&2
    echo "Create it from config/network/field-wifi.env.example with mode 0600." >&2
    exit 1
fi
if find "$CONFIG_FILE" -prune -perm /077 -print -quit | grep -q .; then
    echo "ERROR: $CONFIG_FILE contains a secret and must have mode 0600 (or stricter)." >&2
    exit 1
fi

# The file is root-controlled and intentionally uses simple shell assignments.
# shellcheck disable=SC1090
source "$CONFIG_FILE"

SSID="${FIELD_WIFI_SSID:-TennisRobot}"
PASSPHRASE="${FIELD_WIFI_PASSPHRASE:-}"
CONNECTION="${FIELD_WIFI_CONNECTION:-tennis-robot-field-ap}"
INTERFACE="${FIELD_WIFI_INTERFACE:-}"
ADDRESS="${FIELD_WIFI_ADDRESS:-10.42.0.1/24}"
BAND="${FIELD_WIFI_BAND:-bg}"
CHANNEL="${FIELD_WIFI_CHANNEL:-}"
SECURITY="${FIELD_WIFI_SECURITY:-wpa-psk}"

[ -n "$SSID" ] || { echo "ERROR: FIELD_WIFI_SSID cannot be empty." >&2; exit 1; }
if [ "$PASSPHRASE" = "REPLACE_WITH_A_UNIQUE_RANDOM_PASSPHRASE" ]; then
    PASSPHRASE=""
fi
if [ "${#PASSPHRASE}" -lt 8 ] || [ "${#PASSPHRASE}" -gt 63 ]; then
    echo "ERROR: FIELD_WIFI_PASSPHRASE must contain 8-63 characters." >&2
    exit 1
fi
case "$SECURITY" in wpa-psk|sae) ;; *) echo "ERROR: FIELD_WIFI_SECURITY must be wpa-psk or sae." >&2; exit 1 ;; esac
PMF="disable"
[ "$SECURITY" = "sae" ] && PMF="required"
case "$BAND" in bg|a) ;; *) echo "ERROR: FIELD_WIFI_BAND must be bg (2.4 GHz) or a (5 GHz)." >&2; exit 1 ;; esac
if ! [[ "$ADDRESS" =~ ^([0-9]{1,3}\.){3}[0-9]{1,3}/([0-9]|[12][0-9]|3[0-2])$ ]]; then
    echo "ERROR: FIELD_WIFI_ADDRESS must be an IPv4 CIDR address (for example 10.42.0.1/24)." >&2
    exit 1
fi
IP_PART="${ADDRESS%/*}"
IFS=. read -r IP_A IP_B IP_C IP_D <<< "$IP_PART"
for OCTET in "$IP_A" "$IP_B" "$IP_C" "$IP_D"; do
    if [ "$OCTET" -gt 255 ]; then
        echo "ERROR: FIELD_WIFI_ADDRESS contains an invalid IPv4 octet: $OCTET" >&2
        exit 1
    fi
done

if [ -z "$INTERFACE" ]; then
    INTERFACE="$(nmcli -t -f DEVICE,TYPE device status | awk -F: '$2 == "wifi" {print $1; exit}')"
fi
if [ -z "$INTERFACE" ]; then
    echo "ERROR: no Wi-Fi interface detected; set FIELD_WIFI_INTERFACE explicitly." >&2
    exit 1
fi
if ! nmcli -t -f DEVICE,TYPE device status | awk -F: -v dev="$INTERFACE" '$1 == dev && $2 == "wifi" {found=1} END {exit !found}'; then
    echo "ERROR: $INTERFACE is not a NetworkManager Wi-Fi device." >&2
    exit 1
fi
if [ "$(nmcli -g WIFI-PROPERTIES.AP device show "$INTERFACE" 2>/dev/null)" != "yes" ]; then
    echo "ERROR: $INTERFACE/driver does not report Wi-Fi AP capability." >&2
    exit 1
fi

if nmcli -t -f NAME connection show | grep -Fxq "$CONNECTION"; then
    echo "[field-wifi] updating existing profile: $CONNECTION"
else
    echo "[field-wifi] creating profile: $CONNECTION"
    nmcli connection add type wifi ifname "$INTERFACE" con-name "$CONNECTION" ssid "$SSID"
fi

# ipv4.method=shared makes NetworkManager provide DHCP and local DNS. Supplying
# the address pins the router/service address instead of accepting NM's default.
nmcli connection modify "$CONNECTION" \
    connection.interface-name "$INTERFACE" \
    connection.autoconnect yes \
    connection.autoconnect-priority 999 \
    connection.autoconnect-retries 0 \
    802-11-wireless.ssid "$SSID" \
    802-11-wireless.mode ap \
    802-11-wireless.band "$BAND" \
    802-11-wireless.hidden no \
    802-11-wireless.ap-isolation no \
    802-11-wireless.powersave 2 \
    802-11-wireless-security.key-mgmt "$SECURITY" \
    802-11-wireless-security.proto rsn \
    802-11-wireless-security.pairwise ccmp \
    802-11-wireless-security.group ccmp \
    802-11-wireless-security.pmf "$PMF" \
    802-11-wireless-security.wps-method disabled \
    802-11-wireless-security.psk "$PASSPHRASE" \
    ipv4.method shared \
    ipv4.addresses "$ADDRESS" \
    ipv4.never-default yes \
    ipv6.method disabled

if [ -n "$CHANNEL" ]; then
    [[ "$CHANNEL" =~ ^[0-9]+$ ]] || { echo "ERROR: FIELD_WIFI_CHANNEL must be numeric or empty." >&2; exit 1; }
    nmcli connection modify "$CONNECTION" 802-11-wireless.channel "$CHANNEL"
else
    nmcli connection modify "$CONNECTION" 802-11-wireless.channel ""
fi

# Bringing down the SSH transport underneath provisioning is avoidable. The
# profile is persistent and will still autoconnect at the next boot.
if [ "$ACTIVATE" = true ] && [ -n "${SSH_CONNECTION:-}" ]; then
    SSH_CLIENT_IP="${SSH_CONNECTION%% *}"
    SSH_DEVICE="$(ip route get "$SSH_CLIENT_IP" 2>/dev/null | sed -n 's/.* dev \([^ ]*\).*/\1/p' | head -n1)"
    if [ "$SSH_DEVICE" = "$INTERFACE" ]; then
        echo "[field-wifi] profile installed; activation deferred because SSH uses $INTERFACE."
        echo "[field-wifi] reboot, or activate later with: sudo nmcli connection up '$CONNECTION'"
        ACTIVATE=false
    fi
fi

if [ "$ACTIVATE" = true ]; then
    echo "[field-wifi] activating $CONNECTION on $INTERFACE"
    nmcli connection up "$CONNECTION" ifname "$INTERFACE"
fi

echo "[field-wifi] installed: SSID=$SSID interface=$INTERFACE address=$ADDRESS security=$SECURITY autoconnect=yes"
