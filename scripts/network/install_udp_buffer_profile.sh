#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
SOURCE="$ROOT_DIR/config/network/99-tennis-robot-udp-buffers.conf"
TARGET="/etc/sysctl.d/99-tennis-robot-udp-buffers.conf"

if [ "${EUID}" -ne 0 ]; then
    echo "Run with sudo: sudo $0"
    exit 1
fi

install -m 0644 "$SOURCE" "$TARGET"
sysctl --load "$TARGET"
echo "Installed $TARGET"
