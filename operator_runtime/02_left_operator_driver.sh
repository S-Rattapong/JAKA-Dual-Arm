#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source "${HOME}/jaka_ws/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-1}"
# Fixed guarded equivalent: ros2 run jaka_driver jaka_driver --ros-args
# -r __node:=left_jaka_driver_node -p ip:=192.168.0.1
# -p read_only:=false -p auto_enable:=true -p service_prefix:=/left_jaka_driver
exec python3 "${HOME}/jaka_ws/operator_runtime/driver_guard.py" left
