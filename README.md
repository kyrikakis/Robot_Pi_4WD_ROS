# Robot_Pi_4WD_ROS
Robot_Pi_4WD_ROS

## Usefull commands
```
/pi-bno055/getbno055 -m ndof
watch -n 0.5 /pi-bno055/getbno055 -t inf

as soon as the sensor is fully calibrated...

/pi-bno055/getbno055 -w bno055.cal
/pi-bno055/loadcal_bno055.sh silent
ros2 run bno055 bno055 --ros-args --params-file config/bno055_params.yaml

ros2 launch sllidar_ros2 sllidar_a1_launch.py
ros2 launch pca9685_ros2_control_example diff_drive_example.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap /cmd_vel:=/diff_drive_controller/cmd_vel_unstamped

ros2 run camera_ros camera_node --ros-args -p camera:=1 -p width:=1024 -p height:=576 -p role:=video -p format:=BGR888 -p FrameDurationLimits:="[50000,50000]" -r __node:=right

ros2 run camera_ros camera_node --ros-args -p camera:=0 -p width:=1024 -p height:=576 -p role:=video -p format:=BGR888 -p FrameDurationLimits:="[50000,50000]" -r __node:=left

ros2 launch stereo_image_proc.launch.py approximate_sync:=true approximate_sync_tolerance_seconds:=0.1

ros2 run image_view image_view --ros-args -r image:=/camera/image_raw

docker exec -i yahboom_camera bash -c "/workspaces/Robot_Pi_4WD_ROS/start_camera.sh"

ros2 launch yahboomcar_bringup yahboomcar_bringup_launch.py

ros2 launch nav2_bringup navigation_launch.py

ros2 launch slam_toolbox online_async_launch.py
```