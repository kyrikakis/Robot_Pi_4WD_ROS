#!/bin/bash
# git clone -b mx/set_topic_names_via_param git@github.com:rosblox/ros2_controllers.git && \
# mkdir /colcon_ws/src/ros2_controllers && \
# cp -r ros2_controllers/diff_drive_controller /colcon_ws/src/ros2_controllers/diff_drive_controller && \
# rm -rf ros2_controllers && \
# cd /colcon_ws/src && \
# git clone https://github.com/Slamtec/sllidar_ros2.git && \
# cd /colcon_ws && \
# source /opt/ros/humble/setup.bash && colcon build --symlink-install  --event-handlers console_direct+ && \
# echo "source /colcon_ws/install/setup.bash" >> ~/.bashrc && \
# # \
# source /opt/ros/humble/setup.bash && \
# cd /workspaces/Robot_Pi_4WD_ROS && \
# colcon build --symlink-install && \
# echo "source /workspaces/Robot_Pi_4WD_ROS/install/setup.bash" >> ~/.bashrc
# cd / && git clone git@github.com:fm4dd/pi-bno055.git && cd pi-bno055 && make && \
# sed -i '/BINPATH=\/home\/pi\/pi-bno055/c\BINPATH=\/pi-bno055' loadcal_bno055.sh && \
# sed -i '/CALFILE=\/home\/pi\/pi-bno055-conf\/cal.cfg/c\CALFILE=\/pi-bno055\/bno055.cal' loadcal_bno055.sh && \
# sed -i 's/ndof/ndof_fmc/g' loadcal_bno055.sh && \
# echo "/pi-bno055/loadcal_bno055.sh silent" >> ~/.bashrc
# \
# cd /colcon_ws/src && \
# git clone git@github.com:sensorfusionbox/camera_ros.git && \
# cd /colcon_ws && source /opt/ros/humble/setup.bash && colcon build --symlink-install && \
# echo "source /colcon_ws/install/setup.bash" >> ~/.bashrc && \
# mkdir /root/.ros/camera_info && \
# cp /workspaces/Robot_Pi_4WD_ROS/calibration/left.yaml /root/.ros/camera_info/1__base_axi_pcie_120000_rp1_i2c_80000_imx296_1a_1024x576.yaml && \
# cp /workspaces/Robot_Pi_4WD_ROS/calibration/right.yaml /root/.ros/camera_info/0__base_axi_pcie_120000_rp1_i2c_88000_imx296_1a_1024x576.yaml

# echo "export ROS_DOMAIN_ID=20" >> ~/.bashrc