cmake_minimum_required(VERSION 3.8)
project(krai_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY robot missions mechanisms profiles
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
