#!/bin/bash
export ROS_DOMAIN_ID=20  && \
source /opt/ros/$ROS_DISTRO/setup.bash && \
source /workspaces/Robot_Pi_4WD_ROS/install/setup.bash && \
source /uros_ws/install/setup.bash && \
sleep 10 && \
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB_ROS -b 921600 -v4