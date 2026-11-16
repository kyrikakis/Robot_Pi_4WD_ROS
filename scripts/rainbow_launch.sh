#!/bin/bash
# Run a rainbow_bringup launch file with the full ROS environment.
# Usage: rainbow_launch.sh <launch file> [launch arguments...]
#   e.g. rainbow_launch.sh base.launch.py tof:=false

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-20}"

source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
for ws in /uros_ws /colcon_ws /workspaces/Robot_Pi_4WD_ROS; do
    if [ -f "$ws/install/local_setup.bash" ]; then
        source "$ws/install/local_setup.bash"
    fi
done

exec ros2 launch rainbow_bringup "$@"
