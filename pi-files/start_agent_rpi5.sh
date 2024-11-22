#!/bin/bash
# WIFI
#docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:humble udp4 --port 8090 -v4
# serial
function cleanup()
{
  docker kill microros
}

trap cleanup INT
docker run -i --name microros  --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:humble serial --dev /dev/ttyUSB_ROS -b 921600 -v4 &
wait
