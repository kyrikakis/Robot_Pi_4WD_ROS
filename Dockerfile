FROM ros:humble-ros-core

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    libi2c-dev \
    python3-colcon-common-extensions \
    ros-humble-hardware-interface \
    ros-humble-controller-manager \
    ros-humble-velocity-controllers \
    ros-humble-joint-state-broadcaster \
    ros-humble-robot-state-publisher \
    ros-humble-xacro \
    ros-humble-tf2 \
    ros-humble-tf2-msgs \
    && rm -rf /var/lib/apt/lists/*


COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x  /ros_entrypoint.sh
ENV ROS_DISTRO humble
ENV LANG en_US.UTF-8

SHELL ["/bin/bash", "-c"] 

RUN mkdir -p /colcon_ws/src

RUN cd /colcon_ws/src && \
    git clone git@github.com:kyrikakis/pca9685_ros2_control.git && \
    cd /colcon_ws && \
    source /opt/ros/humble/setup.bash && colcon build --symlink-install  --event-handlers console_direct+

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc 
RUN echo "source /colcon_ws/install/setup.bash" >> ~/.bashrc 
ENTRYPOINT ["/ros_entrypoint.sh"]

USER $USERNAME
# terminal colors with xterm
ENV TERM xterm
WORKDIR /colcon_ws
CMD ["bash"]