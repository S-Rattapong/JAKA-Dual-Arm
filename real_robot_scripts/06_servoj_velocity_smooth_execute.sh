#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
source ~/jaka_ws/install/setup.bash

python3 ~/jaka_ws/real_robot_scripts/sim_to_real_velocity_limited_servoj_bridge.py \
  --source /joint_states \
  --execute \
  --max-step 0.0005 \
  --period 0.05 \
  --deadband 0.00050 \
  --watchdog-timeout 0.0 \
  --panic-if-error-over 1.2
