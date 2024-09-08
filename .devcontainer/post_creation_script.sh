#!/bin/bash
tar -xvf /workspaces/ORB_SLAM3_ROS2_Docker/vocabulary/ORBvoc.txt.tar.gz -C /workspaces/ORB_SLAM3_ROS2_Docker/vocabulary
cd /ORB_SLAM3 && chmod +x build.sh && ./build.sh
cd /workspaces/ORB_SLAM3_ROS2_Docker && colcon build --symlink-install --packages-select orbslam3
echo "source /workspaces/ORB_SLAM3_ROS2_Docker/install/setup.bash" >> ~/.bashrc 