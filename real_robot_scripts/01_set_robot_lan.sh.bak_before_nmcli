#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
source ~/jaka_ws/install/setup.bash

echo "Setting eno2 = 192.168.0.10/24"
sudo ip addr flush dev eno2
sudo ip addr add 192.168.0.10/24 dev eno2
sudo ip link set eno2 up

echo "----- eno2 -----"
ip addr show eno2 | grep "inet "

echo "----- ping LEFT 192.168.0.1 -----"
ping -I 192.168.0.10 -c 10 192.168.0.1

echo "----- ping RIGHT 192.168.0.2 -----"
ping -I 192.168.0.10 -c 10 192.168.0.2
