cmake_minimum_required(VERSION 3.5)
project(mobile_robot_web)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch assets
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
