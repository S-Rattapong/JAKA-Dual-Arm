#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
source ~/jaka_ws/install/setup.bash

xacro_file=$(ros2 pkg prefix --share jaka_a12_moveit_config)/config/dual_jaka_a12.urdf.xacro

ros2 run robot_state_publisher robot_state_publisher --ros-args \
  -p robot_description:="$(xacro "$xacro_file")"
