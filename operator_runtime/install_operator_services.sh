#!/usr/bin/env bash
set -euo pipefail
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}"
export DBUS_SESSION_BUS_ADDRESS="${DBUS_SESSION_BUS_ADDRESS:-unix:path=$XDG_RUNTIME_DIR/bus}"
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
mkdir -p "$HOME/.config/systemd/user" "$HOME/.local/share/applications"
install -m 644 "$SCRIPT_DIR"/systemd/*.service "$HOME/.config/systemd/user/"
install -m 755 "$SCRIPT_DIR/JAKA Dual Arm Control.desktop" "$HOME/.local/share/applications/"
DESKTOP_DIR="$(xdg-user-dir DESKTOP 2>/dev/null || true)"
DESKTOP_DIR="${DESKTOP_DIR:-$HOME/Desktop}"
mkdir -p "$DESKTOP_DIR"
install -m 755 "$SCRIPT_DIR/JAKA Dual Arm Control.desktop" "$DESKTOP_DIR/"
systemctl --user daemon-reload
printf '%s\n' 'Installed operator units and Desktop launcher. No services started.'
