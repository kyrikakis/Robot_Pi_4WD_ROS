#!/bin/bash
function cleanup()
{
  echo 'stop camera_ros'
}

trap cleanup INT

export ROS_DOMAIN_ID=20  && \
source /opt/ros/humble/setup.bash && \
source /workspaces/Robot_Pi_4WD_ROS/install/setup.bash && \
ros2 run camera_ros camera_node --ros-args -p camera:=0 -p width:=1024 -p height:=576 -p role:=video -p format:=BGR888 -p FrameDurationLimits:="[50000,50000]"
