#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
source ~/jaka_ws/install/setup.bash

ros2 service call /left_jaka_driver/stop_move std_srvs/srv/Empty "{}"
ros2 service call /right_jaka_driver/stop_move std_srvs/srv/Empty "{}"
