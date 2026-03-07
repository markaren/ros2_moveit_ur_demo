#!/bin/bash
set -e
source /opt/ros/jazzy/setup.bash

# Launch ur_robot_driver connecting to URSim
ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=${UR_TYPE} \
    robot_ip:=${ROBOT_IP} \
	reverse_ip:=172.20.0.3 \
    launch_rviz:=false \
    initial_joint_controller:=scaled_joint_trajectory_controller