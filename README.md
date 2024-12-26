# Robot_Pi_4WD_ROS
Robot_Pi_4WD_ROS

## Installation
1. Download and install a new pi bookworm image in a micro sd. Proposed a Sundisk Extreme A2 256GB using the pi imager 
2. Edit the imager settings and add your ~/.ssh/id_rsa.pub
3. Connect to your raspberry pi5 into the a monitor through hdmi0 a keyboard and mouse
4. Connect to your pi5 through ssh from you desktop if you skipped adding your ssh pub keys in step 1. write your `~/.ssh/id_rsa.pub` in a usb stick and copy the contents in the pi ~/.ssh/authorized-keys
5. `sudo update && sudo full-upgrade && sudo install -y vim less`
6. Install docker: https://docs.docker.com/engine/install/debian/#install-using-the-repository and https://docs.docker.com/engine/install/linux-postinstall/
7. Add these in `/boot/firmware/config.txt`:
```
camera_auto_detect=0
[all]
dtparam=pciex1_gen=3
power_force_3v3_pwm=1
dtoverlay=arducam-pivariety,cam0
dtoverlay=imx296,cam1
```
8. Run `sudo raspi-config` and enable: i2c, vnc, ssh, pcie3
9. Add an alias to the microros serial port according to https://forums.raspberrypi.com/viewtopic.php?t=90265:
    `echo 'ACTION=="add",ENV{ID_SERIAL}=="Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001",ENV{ID_SERIAL_SHORT}=="0001",SYMLINK+="ttyUSB_ROS\"' >> /etc/udev/rules.d/99-usbserial.rules`
10. Download the source code: `git clone https://github.com/kyrikakis/Robot_Pi_4WD_ROS.git`
11. Run the image: `cd Robot_Pi_4WD_ROS && docker compose up`
12. Install ros2 jazzy to pi5 https://github.com/Ar-Ray-code/rpi-bullseye-ros2. Source ros -> 
    `echo "export ROS_DOMAIN_ID=20" >> ~/.bashrc && echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc`
13. Install a bunch of ros stuff and:
    `echo "source ~/colcon_ws/install/setup.bash" >> ~/.bashrc`

## Usefull commands
```
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

ros2 run rqt_graph rqt_graph
ros2 run tf2_tools view_frames
```

### Speech recognition

Using speech recognition from https://github.com/Uberi/speech_recognition and snowboy https://github.com/kyrikakis/snowboy/tree/master

to train a new model install sox and record three samples of the hotword 3 times record1/2/3 inside a models folder: 
`rec -r 16000 -c 1 -b 16 -e signed-integer -t wav record1.wav`
then run:
`docker run -it -v $(pwd)/model:/snowboy-master/examples/Python/model meowxiik/snowboy-pmdl`
