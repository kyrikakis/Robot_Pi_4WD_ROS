cmake_minimum_required(VERSION 3.8)
project(rainbow_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetHeadPose.srv"
  "srv/SetExpression.srv"
  "srv/Say.srv"
  "msg/Place.msg"
  "srv/SavePlace.srv"
  "srv/EditPlace.srv"
  "srv/DeletePlace.srv"
  "srv/ListPlaces.srv"
  "srv/WhereAmI.srv"
  "srv/GetState.srv"
  "action/GoToPlace.action"
  "action/Turn.action"
  "action/Drive.action"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
