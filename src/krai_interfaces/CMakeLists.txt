cmake_minimum_required(VERSION 3.8)
project(krai_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BaseTelemetry.msg"
  "msg/S3Health.msg"
  "msg/PrimitiveStatus.msg"
  "msg/MissionState.msg"
  "srv/ResetOdom.srv"
  "srv/ResetHeading.srv"
  "srv/SetRobotMode.srv"
  "srv/EmergencyStop.srv"
  "srv/ListMissions.srv"
  "action/ExecutePrimitive.action"
  "action/RunMission.action"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
)

ament_package()
