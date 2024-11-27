#!/bin/bash
export ROS_DOMAIN_ID=20  && \
source /opt/ros/humble/setup.bash && \
source /workspaces/Robot_Pi_4WD_ROS/install/setup.bash && \
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=/workspaces/Robot_Pi_4WD_ROS/config/slam_toolbox_params.yaml