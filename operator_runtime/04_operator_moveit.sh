#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source "$HOME/jaka_ws/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-1}"
exec ros2 launch jaka_a12_moveit_config dual_moveit_rviz.launch.py use_rviz:=false use_joint_state_publisher_gui:=false
