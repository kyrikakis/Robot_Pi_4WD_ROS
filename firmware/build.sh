#!/usr/bin/env bash
# Build the firmware in Docker (Mac or Pi). Output: build/rainbow_esp.bin and
# build/flash_args (bootloader, partition table, app), used by flash.sh.
#
#   ./build.sh            build
#   ./build.sh menuconfig any other idf.py command
set -euo pipefail
cd "$(dirname "$0")"

IMAGE=rainbow-esp-idf:5.2.5
MICRO_ROS_REPO=https://github.com/micro-ROS/micro_ros_espidf_component.git
MICRO_ROS_COMMIT=5cd71f525eedc1e59a48e3a12184ba75b60b05b7  # jazzy, 2026-08-21

docker image inspect "$IMAGE" >/dev/null 2>&1 || docker build -t "$IMAGE" docker

COMP=components/micro_ros_espidf_component
if [ ! -d "$COMP/.git" ]; then
  git clone -q "$MICRO_ROS_REPO" "$COMP"
fi
if [ "$(git -C "$COMP" rev-parse HEAD)" != "$MICRO_ROS_COMMIT" ]; then
  git -C "$COMP" fetch -q origin
  git -C "$COMP" checkout -q "$MICRO_ROS_COMMIT"
  git -C "$COMP" clean -qfdx   # rebuild libmicroros for the new commit
fi

# The component clones its sources at branch heads; clone them at the commits in
# micro_ros.lock instead (same COLCON_IGNOREs as its libmicroros.mk).
SRC=$COMP/micro_ros_src/src
if [ ! -d "$SRC" ]; then
  mkdir -p "$SRC.tmp"
  grep -v '^#' micro_ros.lock | while read -r name url commit; do
    git clone -q "$url" "$SRC.tmp/$name"
    git -C "$SRC.tmp/$name" checkout -q "$commit"
  done
  for f in rosidl/rosidl_typesupport_introspection_cpp rcl_logging/rcl_logging_log4cxx \
           rcl_logging/rcl_logging_spdlog rclc/rclc_examples rcl/rcl_yaml_param_parser \
           ros2_tracing/test_tracetools ros2_tracing/lttngpy; do
    [ -d "$SRC.tmp/$f" ] && touch "$SRC.tmp/$f/COLCON_IGNORE"
  done
  cp -r "$COMP/extra_packages" "$SRC.tmp/extra_packages"
  mv "$SRC.tmp" "$SRC"
fi

VERSION=$(git describe --always --dirty --tags 2>/dev/null || echo unknown)
TTY=$([ -t 0 ] && echo -it || echo -i)
docker run --rm $TTY -v "$PWD":/project -w /project \
  "$IMAGE" idf.py -DRAINBOW_FW_VERSION="$VERSION" "${@:-build}"
