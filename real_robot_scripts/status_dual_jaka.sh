#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
source ~/jaka_ws/install/setup.bash

echo "----- NODES -----"
ros2 node list | grep jaka || true

echo "----- LEFT STATE -----"
timeout 5 ros2 topic echo --once /left_jaka_driver/robot_states || true

echo "----- RIGHT STATE -----"
timeout 5 ros2 topic echo --once /right_jaka_driver/robot_states || true

echo "----- LEFT JOINT -----"
timeout 5 ros2 topic echo --once /left_jaka_driver/joint_position || true

echo "----- RIGHT JOINT -----"
timeout 5 ros2 topic echo --once /right_jaka_driver/joint_position || true
