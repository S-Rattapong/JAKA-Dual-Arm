#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
source ~/jaka_ws/install/setup.bash

ros2 launch jaka_a12_moveit_config dual_gazebo_dynamic_validation_demo.launch.py \
  command_backend:=joint_states \
  enable_gazebo_feedback_validation:=false \
  gazebo_validation_csv_enabled:=false \
  enable_dynamic_object_test:=true \
  dynamic_object_test_mode:=sinusoidal \
  dynamic_object_axis:=x \
  dynamic_object_amplitude:=0.015 \
  dynamic_object_period:=30.0 \
  dynamic_object_start_delay:=5.0 \
  dynamic_object_max_speed:=0.006 \
  dynamic_object_hold_initial_position:=true
