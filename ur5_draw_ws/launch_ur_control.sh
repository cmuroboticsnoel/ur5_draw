#!/bin/bash
shopt -s expand_aliases
source ~/.bashrc

source /opt/ros/humble/setup.bash source ${ROS_WS}/install/setup.bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5 robot_ip:=$ROBOT_IP kinematics_params_file:="${WS_PATH}/${CALI_FILENAME}" \
launch_rviz:=false use_fake_hardware:=false headless_mode:=true reverse_ip:=192.168.1.101 script_sender_port:=50002
