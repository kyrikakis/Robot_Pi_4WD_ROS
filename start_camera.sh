#!/bin/bash
export ROS_DOMAIN_ID=20  && \
source /opt/ros/$ROS_DISTRO/setup.bash && \
source /colcon_ws/install/setup.bash && \
ros2 run camera_ros camera_node --ros-args -p camera:=0 -p width:=1024 -p height:=576 -p role:=video -p format:=BGR888 -p FrameDurationLimits:="[50000,50000]" \
--remap /camera/image_raw/compressed:=/camera/rgb/image_raw/compressed \
--remap /camera/image_raw:=/camera/rgb/image_raw
