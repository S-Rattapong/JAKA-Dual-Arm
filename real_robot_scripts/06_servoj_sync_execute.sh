#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
source ~/jaka_ws/install/setup.bash

python3 ~/jaka_ws/real_robot_scripts/sim_to_real_sync_velocity_servoj_bridge.py \
  --source /joint_states \
  --execute \
  --max-step 0.0008 \
  --period 0.04 \
  --deadband 0.00080 \
  --target-alpha 0.08 \
  --watchdog-timeout 0.0 \
  --panic-if-error-over 2.5
