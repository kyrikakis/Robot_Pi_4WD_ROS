FROM ros:humble-perception-jammy

RUN apt update && apt upgrade -y

ENV ROS_DISTRO humble
ENV LANG en_US.UTF-8

SHELL ["/bin/bash", "-c"] 

RUN mkdir -p /colcon_ws/src

RUN apt update && apt install i2c-tools -y

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-hardware-interface \
    libi2c-dev \
    i2c-tools \
    ros-humble-hardware-interface \
    ros-humble-controller-manager \
    ros-humble-velocity-controllers \
    ros-humble-joint-state-broadcaster \
    ros-humble-robot-state-publisher \
    ros-humble-xacro \
    ros-humble-tf2 \
    ros-humble-tf2-msgs \
    ros-humble-teleop-twist-keyboard \
    gdbserver \
    gdb \
    ros-humble-bno055 \
    ros-humble-robot-localization

# # Install libcamera -Start
RUN apt-get update && apt-get install -y --no-install-recommends \
    apt-utils \
    python3 python3-dev python3-pip git python3-jinja2 \
    libgnutls28-dev openssl libtiff5-dev pybind11-dev liblttng-ust-dev \
    cmake ninja-build \
    python3-yaml python3-ply python3-pip libyaml-dev \
    libglib2.0-dev libgstreamer-plugins-base1.0-dev
    
RUN pip3 install --user meson
ENV PATH="$PATH:/root/.local/bin/"

RUN cd / && git clone https://github.com/raspberrypi/libcamera.git --branch v0.3.1+rpt20240906 && \
    cd /libcamera && \
    meson setup build --buildtype=release \
    -Dpipelines=rpi/vc4,rpi/pisp \
    -Dipas=rpi/vc4,rpi/pisp -Dv4l2=true \
    -Dgstreamer=enabled \
    -Dtest=false \
    -Dlc-compliance=disabled \
    -Dcam=disabled \
    -Dqcam=disabled \
    -Ddocumentation=disabled \
    -Dpycamera=enabled && \
    ninja -C build && \
    ninja -C build install
# Install libcamera -End

# Install rpicam-apps
RUN apt-get update && apt-get install -y --no-install-recommends \
    cmake libboost-program-options-dev libdrm-dev libexif-dev && \
    cd / && git clone https://github.com/raspberrypi/rpicam-apps.git --branch v1.5.1 && \
    cd /rpicam-apps && \
    meson setup build -Denable_libav=disabled -Denable_drm=enabled -Denable_egl=disabled -Denable_qt=disabled -Denable_opencv=disabled -Denable_tflite=disabled && \
    meson compile -C build && \
    /root/.local/bin/meson install -C build && \
    ldconfig

RUN cd /colcon_ws/src && \
    git clone https://github.com/christianrauch/camera_ros.git && \
    cd /colcon_ws && source /opt/ros/humble/setup.bash && colcon build --symlink-install && \
    echo "source /colcon_ws/install/setup.bash" >> ~/.bashrc

RUN apt-get update && apt-get install -y \
    ros-humble-imu-tools \
    ros-humble-joint-state-publisher \
    ros-humble-slam-toolbox \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup

RUN pip3 install rpi-lgpio adafruit-circuitpython-ht16k33

RUN echo "export ROS_DOMAIN_ID=20" >> ~/.bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    mkdir /uros_ws && cd /uros_ws && \
    git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup && \
    rosdep update && rosdep install --from-paths src --ignore-src -y && \
    colcon build && \
    source install/local_setup.bash && \
    ros2 run micro_ros_setup create_agent_ws.sh && \
    ros2 run micro_ros_setup build_agent.sh && \
    source install/local_setup.sh

RUN mkdir -p /workspaces/Robot_Pi_4WD_ROS/
COPY . /workspaces/Robot_Pi_4WD_ROS/

RUN source /opt/ros/humble/setup.bash && \
    cd /workspaces/Robot_Pi_4WD_ROS && \
    colcon build --symlink-install && \
    echo "source /workspaces/Robot_Pi_4WD_ROS/install/setup.bash" >> ~/.bashrc

RUN apt-get update && apt-get install -y \
    supervisor
COPY ./supervisor/* /etc/supervisor/conf.d/

COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x  /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]

USER $USERNAME
# terminal colors with xterm
ENV TERM xterm
WORKDIR /workspaces/Robot_Pi_4WD_ROS
CMD [ "sleep", "infinity" ]