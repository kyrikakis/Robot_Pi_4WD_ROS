#!/bin/bash
export ROS_DOMAIN_ID=20  && \
source /opt/ros/humble/setup.bash && \
/pi-bno055/loadcal_bno055.sh silent && \
ros2 run bno055 bno055 --ros-args --params-file config/bno055_params.yaml