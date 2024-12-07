#!/bin/bash
export ROS_DOMAIN_ID=20  && \
source /opt/ros/$ROS_DISTRO/setup.bash && \
ros2 run bno055 bno055 --ros-args --params-file config/bno055_params.yaml