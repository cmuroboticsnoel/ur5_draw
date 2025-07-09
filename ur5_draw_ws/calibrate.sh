#!/bin/bash
shopt -s expand_aliases
source ~/.bashrc

source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch ur_calibration calibration_correction.launch.py robot_ip:=$ROBOT_IP target_filename:="${CALI_PATH}/${CALI_FILENAME}"