cmake_minimum_required(VERSION 3.8)
project(rainbow_esp_msgs)

# Messages of the ESP firmware's own topics. Kept apart from rainbow_interfaces:
# firmware/build.sh compiles this package into micro-ROS, so it must stay small and
# depend on nothing.
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/HeadServoGoal.msg"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
