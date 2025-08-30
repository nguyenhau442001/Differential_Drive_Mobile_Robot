Description: This repository will build and run a mobile robot simulation using Gazebo on ROS1 Noetic

## 1. Environment Setup
Install ROS1 Noetic: https://wiki.ros.org/noetic/Installation/Ubuntu
```bash
source /opt/ros/noetic/setup.bash
sudo apt install python3-roslaunch
sudo apt-get install ros-noetic-joy ros-noetic-teleop-twist-joy \
  ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc \
  ros-noetic-rgbd-launch ros-noetic-rosserial-arduino \
  ros-noetic-rosserial-python ros-noetic-rosserial-client \
  ros-noetic-rosserial-msgs ros-noetic-amcl ros-noetic-map-server \
  ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro \
  ros-noetic-compressed-image-transport ros-noetic-rqt* ros-noetic-rviz \
  ros-noetic-gmapping ros-noetic-navigation ros-noetic-interactive-markers
```

## 2. Create the work space
```bash
mkdir ~/catkin_ws/src && cd ~/catkin_ws/src
git clone git@github.com:nguyenhau442001/Differential_Drive_Mobile_Robot.git && cd ..
catkin_make
source devel/setup.bash
```

## 3. Create the new package
```bash
catkin_create_pkg mobile_robot
cd ~/catkin_ws
catkin_make --pkg mobile_robot
```
