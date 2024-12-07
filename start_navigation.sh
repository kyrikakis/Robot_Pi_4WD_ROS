#!/bin/bash
export ROS_DOMAIN_ID=20  && \
source /opt/ros/$ROS_DISTRO/setup.bash && \
source /workspaces/Robot_Pi_4WD_ROS/install/setup.bash && \
ros2 launch nav2_bringup navigation_launch.py params_file:='/workspaces/Robot_Pi_4WD_ROS/config/nav2_params.yaml'