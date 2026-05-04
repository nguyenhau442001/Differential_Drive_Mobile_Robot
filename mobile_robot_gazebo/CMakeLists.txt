cmake_minimum_required(VERSION 3.5)
project(mobile_robot_gazebo)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ros_gz REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})

# Install resources
install(DIRECTORY launch worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
