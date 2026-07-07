#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
source ~/jaka_ws/install/setup.bash

rviz2 -d ~/jaka_ws/real_robot_configs/dual_jaka_interactive.rviz
