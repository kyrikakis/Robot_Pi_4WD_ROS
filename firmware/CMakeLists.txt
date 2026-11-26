# Rainbow's ESP32-S3 firmware (Yahboom MicroROS board). See README.md.
cmake_minimum_required(VERSION 3.16)

# Set by build.sh from the repo's git describe (the build runs in Docker without .git).
if(DEFINED ENV{RAINBOW_FW_VERSION})
  set(PROJECT_VER "$ENV{RAINBOW_FW_VERSION}")
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(rainbow_esp)
