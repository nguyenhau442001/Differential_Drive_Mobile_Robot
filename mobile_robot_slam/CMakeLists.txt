cmake_minimum_required(VERSION 3.0.2)
project(mobile_robot_slam)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

## Declare this catkin package
catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs
)

## Include directories (if you have any headers)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Add executables here (if any)
# add_executable(slam_node src/slam_node.cpp)
# target_link_libraries(slam_node ${catkin_LIBRARIES})

## Install directories (config, launch, rviz, etc.)
install(DIRECTORY config launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## Install executables (optional)
# install(TARGETS slam_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
