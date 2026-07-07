#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
source ~/jaka_ws/install/setup.bash

ros2 run jaka_driver jaka_driver --ros-args \
  -r __node:=left_jaka_driver_node \
  -p ip:=192.168.0.1 \
  -p read_only:=false \
  -p auto_enable:=true \
  -p service_prefix:=/left_jaka_driver
