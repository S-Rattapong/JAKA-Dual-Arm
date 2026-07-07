#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
source ~/jaka_ws/install/setup.bash

URDF=~/jaka_ws/real_robot_configs/dual_jaka_a12_real_twin.urdf

if [ ! -f "$URDF" ]; then
  echo "ERROR: $URDF not found."
  echo "Run D25.2 first."
  exit 1
fi

echo "Starting real twin robot_state_publisher..."
ros2 run robot_state_publisher robot_state_publisher --ros-args \
  -r __node:=real_twin_robot_state_publisher \
  -r /joint_states:=/real_twin/joint_states \
  -r /robot_description:=/real_twin/robot_description \
  -p robot_description:="$(cat $URDF)" &

RSP_PID=$!

sleep 1

echo "Starting real twin joint_state publisher..."
python3 ~/jaka_ws/real_robot_scripts/real_twin_joint_state_publisher.py \
  --urdf "$URDF"

kill $RSP_PID 2>/dev/null
