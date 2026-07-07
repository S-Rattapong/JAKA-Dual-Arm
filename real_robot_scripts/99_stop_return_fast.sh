#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
source ~/jaka_ws/install/setup.bash

echo "[1/3] Stop both JAKA..."
~/jaka_ws/real_robot_scripts/stop_both_jaka.sh || true

echo "[2/3] Disable servo mode..."
ros2 service call /left_jaka_driver/servo_move_enable jaka_msgs/srv/ServoMoveEnable "{enable: false}" || true
ros2 service call /right_jaka_driver/servo_move_enable jaka_msgs/srv/ServoMoveEnable "{enable: false}" || true

echo "[3/3] Return to MAIN REAL HOME..."
python3 ~/jaka_ws/real_robot_scripts/return_main_home.py --vel 0.8 --acc 0.30
