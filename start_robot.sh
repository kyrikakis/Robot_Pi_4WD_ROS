#!/bin/bash
function cleanup()
{
  echo 'stop robot'
}

# trap cleanup INT

export ROS_DOMAIN_ID=20  && \
source /opt/ros/humble/setup.bash && \
source /workspaces/Robot_Pi_4WD_ROS/install/setup.bash && \
ros2 launch yahboomcar_bringup yahboomcar_bringup_launch.py & \
ros2 launch nav2_bringup navigation_launch.py & \
ros2 launch slam_toolbox online_async_launch.py