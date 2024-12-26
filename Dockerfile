FROM ros:jazzy-perception-noble

RUN apt update && apt upgrade -y

ENV ROS_DISTRO jazzy
ENV LANG en_US.UTF-8

SHELL ["/bin/bash", "-c"] 

RUN mkdir -p /colcon_ws/src

RUN apt update && apt install i2c-tools -y

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-$ROS_DISTRO-hardware-interface \
    libi2c-dev \
    i2c-tools \
    ros-$ROS_DISTRO-hardware-interface \
    ros-$ROS_DISTRO-controller-manager \
    ros-$ROS_DISTRO-velocity-controllers \
    ros-$ROS_DISTRO-joint-state-broadcaster \
    ros-$ROS_DISTRO-robot-state-publisher \
    ros-$ROS_DISTRO-xacro \
    ros-$ROS_DISTRO-tf2 \
    ros-$ROS_DISTRO-tf2-msgs \
    ros-$ROS_DISTRO-teleop-twist-keyboard \
    gdbserver \
    gdb \
    ros-$ROS_DISTRO-bno055 \
    ros-$ROS_DISTRO-robot-localization

# # Install libcamera -Start
RUN apt-get update && apt-get install -y --no-install-recommends \
    apt-utils \
    python3 python3-dev python3-pip git python3-jinja2 \
    libgnutls28-dev openssl libtiff5-dev pybind11-dev liblttng-ust-dev \
    cmake ninja-build \
    python3-yaml python3-ply python3-pip libyaml-dev \
    libglib2.0-dev libgstreamer-plugins-base1.0-dev
    
RUN pip3 install --break-system-packages --user meson
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
    cd /colcon_ws && source '/opt/ros/jazzy/setup.bash' && colcon build --symlink-install && \
    echo "source /colcon_ws/install/setup.bash" >> ~/.bashrc

RUN apt-get update && apt-get install -y \
    ros-$ROS_DISTRO-imu-tools \
    ros-$ROS_DISTRO-joint-state-publisher \
    ros-$ROS_DISTRO-slam-toolbox \
    ros-$ROS_DISTRO-navigation2 \
    ros-$ROS_DISTRO-nav2-bringup \
    ros-$ROS_DISTRO-spatio-temporal-voxel-layer

RUN pip3 install --break-system-packages rpi-lgpio adafruit-circuitpython-ht16k33

RUN echo "Add Arducam_ppa repositories." && \
    curl -s --compressed "https://arducam.github.io/arducam_ppa/KEY.gpg" | sudo apt-key add - && \
    curl -s --compressed -o /etc/apt/sources.list.d/arducam_list_files.list "https://arducam.github.io/arducam_ppa/arducam_list_files.list" && \
    apt update && apt-get install -y arducam-config-parser-dev arducam-evk-sdk-dev arducam-tof-sdk-dev && \
    pip3 install --break-system-packages ArducamDepthCamera

RUN echo "export ROS_DOMAIN_ID=20" >> ~/.bashrc
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc

RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    mkdir /uros_ws && cd /uros_ws && \
    git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup && \
    rosdep update && rosdep install --from-paths src --ignore-src -y && \
    colcon build && \
    source install/local_setup.bash && \
    ros2 run micro_ros_setup create_agent_ws.sh && \
    ros2 run micro_ros_setup build_agent.sh
    
RUN apt-get update && apt-get install -y \
    vim less

RUN apt-get update && apt-get install -y \
    supervisor

# Install speech-recognition
RUN apt install -y portaudio19-dev swig libatlas-base-dev &&\
    pip3 install --break-system-packages SpeechRecognition[audio] SpeechRecognition[whisper-local] && \
    cd / && git clone https://github.com/kyrikakis/snowboy.git && \
    cd /snowboy/swig/Python3 && make

RUN mkdir -p /workspaces/Robot_Pi_4WD_ROS/
COPY . /workspaces/Robot_Pi_4WD_ROS/

RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    cd /workspaces/Robot_Pi_4WD_ROS && \
    colcon build --symlink-install && \
    echo "source /workspaces/Robot_Pi_4WD_ROS/install/setup.bash" >> ~/.bashrc

COPY ./supervisor/* /etc/supervisor/conf.d

COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x  /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]

USER $USERNAME
# terminal colors with xterm
ENV TERM xterm
WORKDIR /workspaces/Robot_Pi_4WD_ROS
CMD [ "sleep", "infinity" ]