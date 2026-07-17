#!/usr/bin/env bash
set -euo pipefail

PROFILE="Profile 2"
IFACE="eno2"

HOST_IP="192.168.0.10"
LEFT_IP="192.168.0.1"
RIGHT_IP="192.168.0.2"

echo "Activating NetworkManager profile: ${PROFILE}"
sudo nmcli connection up "${PROFILE}"

echo
echo "----- ${IFACE} IPv4 address -----"
ip -4 addr show dev "${IFACE}" | grep "inet " || true

if ! ip -4 addr show dev "${IFACE}" | grep -q "inet ${HOST_IP}/24"; then
    echo "ERROR: ${IFACE} does not have ${HOST_IP}/24"
    exit 1
fi

echo
echo "----- route on ${IFACE} -----"
ip route show dev "${IFACE}"

echo
echo "----- ping LEFT ${LEFT_IP} -----"
ping -I "${HOST_IP}" -c 5 "${LEFT_IP}"

echo
echo "----- ping RIGHT ${RIGHT_IP} -----"
ping -I "${HOST_IP}" -c 5 "${RIGHT_IP}"

echo
echo "Robot LAN is ready."
echo "Host  : ${HOST_IP}"
echo "Left  : ${LEFT_IP}"
echo "Right : ${RIGHT_IP}"
