#!/bin/bash
export ROS_DOMAIN_ID=20  && \
source /opt/ros/humble/setup.bash && \
source /workspaces/Robot_Pi_4WD_ROS/install/setup.bash && \
ros2 launch yahboomcar_bringup yahboomcar_bringup_launch.py