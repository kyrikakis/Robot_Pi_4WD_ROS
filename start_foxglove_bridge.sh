#!/bin/bash
export ROS_DOMAIN_ID=20  && \
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && \
source /opt/ros/$ROS_DISTRO/setup.bash && \
source /workspaces/Robot_Pi_4WD_ROS/install/setup.bash && \
ros2 launch foxglove_bridge foxglove_bridge_launch.xml