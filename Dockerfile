FROM ros:jazzy-perception-noble

# Everything cloned or pip-installed below is pinned, so a rebuild produces the
# same software as the last one. To update something, change its pin here.
ARG CAMERA_ROS_COMMIT=d54147c18bd433d83ee0b884e4297ba38a06a513
# Must match the hailo_tappas_ros2 container that publishes the detections
ARG HAILO_TAPPAS_ROS2_COMMIT=f50f2a329c4766103e98cf455e58fa67d8289e65
ARG MICRO_ROS_SETUP_COMMIT=b1b18bc64b02da9f1745d111f37fe10f3107cef4
ARG MICRO_ROS_AGENT_COMMIT=af007872b034d1ed31de4815377031350ab0034b
ARG MICRO_ROS_MSGS_COMMIT=4594d9db17db735b1e655141fb4afb4cdcfc5789

ENV ROS_DISTRO=jazzy
ENV LANG=en_US.UTF-8

SHELL ["/bin/bash", "-c"]

# Bring the base image up to the current ROS sync first. ROS packages are
# released in synchronised batches; installing new packages on top of an older
# batch can mix incompatible versions. (Ubuntu and ROS apt packages are the one
# thing not pinned in this file.)
RUN apt-get update && apt-get upgrade -y

RUN mkdir -p /colcon_ws/src

RUN apt-get update && apt-get install -y --no-install-recommends \
    libi2c-dev \
    i2c-tools \
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
    
RUN pip3 install --break-system-packages --user meson==1.7.0
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
    git -C camera_ros checkout $CAMERA_ROS_COMMIT && \
    cd /colcon_ws && source '/opt/ros/jazzy/setup.bash' && colcon build --symlink-install && \
    echo "source /colcon_ws/install/setup.bash" >> ~/.bashrc

RUN apt-get update && apt-get install -y \
    ros-$ROS_DISTRO-joint-state-publisher \
    ros-$ROS_DISTRO-slam-toolbox \
    ros-$ROS_DISTRO-navigation2 \
    ros-$ROS_DISTRO-nav2-bringup \
    ros-$ROS_DISTRO-spatio-temporal-voxel-layer

# rainbow_brain's link to the Mac agent (WebSocket server)
RUN apt-get update && apt-get install -y --no-install-recommends python3-websockets

# Eyes LED matrix (adafruit) and GPIO
# Blinka and gpiod are pinned too: an unpinned Blinka upgrade once needed gpiod
# on the Pi 5 and broke the eyes
RUN pip3 install --break-system-packages rpi-lgpio==0.6 adafruit-circuitpython-ht16k33==4.6.11 \
    Adafruit-Blinka==10.2.0 gpiod==2.6.0

# Arducam ToF SDK, from Arducam's apt repository
RUN apt-get update && apt-get install -y --no-install-recommends curl gnupg && \
    install -d -m 0755 /etc/apt/keyrings && \
    curl -fsSL --compressed https://arducam.github.io/arducam_ppa/KEY.gpg \
        | gpg --dearmor -o /etc/apt/keyrings/arducam.gpg && \
    echo "deb [signed-by=/etc/apt/keyrings/arducam.gpg] https://arducam.github.io/arducam_ppa linux main" \
        > /etc/apt/sources.list.d/arducam.list && \
    apt-get update && apt-get install -y arducam-config-parser-dev arducam-evk-sdk-dev arducam-tof-sdk-dev && \
    pip3 install --break-system-packages ArducamDepthCamera==0.1.21

RUN echo "export ROS_DOMAIN_ID=20" >> ~/.bashrc
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc

RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    mkdir /uros_ws && cd /uros_ws && \
    git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup && \
    git -C src/micro_ros_setup checkout $MICRO_ROS_SETUP_COMMIT && \
    rosdep update && rosdep install --from-paths src --ignore-src -y && \
    colcon build && \
    source install/local_setup.bash && \
    ros2 run micro_ros_setup create_agent_ws.sh && \
    git -C src/uros/micro-ROS-Agent checkout $MICRO_ROS_AGENT_COMMIT && \
    git -C src/uros/micro_ros_msgs checkout $MICRO_ROS_MSGS_COMMIT && \
    ros2 run micro_ros_setup build_agent.sh
    
RUN apt-get update && apt-get install -y \
    vim less

RUN apt-get update && apt-get install -y \
    supervisor

# Text to speech (rainbow_voice)
RUN apt-get update && apt-get install -y espeak-ng mbrola-us2

# Install foxglove bridge
RUN apt-get update && apt install -y \
    ros-$ROS_DISTRO-foxglove-bridge
    # ros-$ROS_DISTRO-rmw-cyclonedds-cpp
# ENV RMW_IMPLEMENTATION rmw_cyclonedds_cpp

# Message definitions for the Hailo face detections, so nodes here and the
# Foxglove bridge can read them. The detector itself runs in the Hailo container.
RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    cd /colcon_ws/src && git clone https://github.com/kyrikakis/hailo_tappas_ros2.git && \
    git -C hailo_tappas_ros2 checkout $HAILO_TAPPAS_ROS2_COMMIT && \
    cd /colcon_ws && colcon build --symlink-install --packages-select hailo_msgs

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

# terminal colors with xterm
ENV TERM=xterm
WORKDIR /workspaces/Robot_Pi_4WD_ROS
CMD [ "sleep", "infinity" ]