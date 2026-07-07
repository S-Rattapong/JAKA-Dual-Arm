#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
source ~/jaka_ws/install/setup.bash

python3 ~/jaka_ws/real_robot_scripts/sim_joint_state_to_real_servoj_bridge.py \
  --source /joint_states \
  --execute \
  --max-abs-delta 0.040 \
  --max-step 0.0004 \
  --period 0.05 \
  --watchdog-timeout 0.5
