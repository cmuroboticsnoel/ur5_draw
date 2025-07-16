#!/bin/bash
shopt -s expand_aliases
source ~/.bashrc

source /opt/ros/humble/setup.bash && source ${WS_PATH}/install/setup.bash
ros2 launch ur_calibration calibration_correction.launch.py robot_ip:=$ROBOT_IP target_filename:="${WS_PATH}/${CALI_FILENAME}"
