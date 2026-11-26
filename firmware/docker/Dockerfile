# Build environment for the firmware: ESP-IDF plus what micro-ROS needs to build
# its library (colcon, rosidl's Python deps). Multi-arch: works on the Mac and the Pi.
ARG IDF_IMAGE=espressif/idf:v5.2.5@sha256:47a526b1cf92da1c9243938944f6146c0ff555a3e800f973be3ba0363f5ee773
FROM ${IDF_IMAGE}
ARG PIP_PACKAGES="catkin_pkg==1.1.1 colcon-common-extensions==0.3.0 lark==1.3.1 idf-component-manager==2.4.11"
RUN . $IDF_PATH/export.sh >/dev/null && python -m pip install --no-cache-dir ${PIP_PACKAGES}
