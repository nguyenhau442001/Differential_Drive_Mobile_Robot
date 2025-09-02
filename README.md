# Project Description:

Choose Gazebo and ROS1 Noetic, and build a simulation for a mobile robot.

Requirements:
The simulation should include the following components:
- The robot model can be simple, composed of basic shapes like boxes and cylinders.
- Ensure the kinematics are accurate.
- The robot should be manually controllable via keyboard input.
- The robot should be able to achieve 5m/s velocity within 5s from stand still, and should completely stop within 1s.
- Include a configuration file (or preferably an interactive tool) to modify the robot’s physical parameters (wheelbase, track width, wheel radius, etc.)
- Implement a simple go-to-goal action, where the user can specify a coordinate for the robot to move.
  
  ○ Hint: It does not have to be a fancy algorithm, as long as the robot can reach the proximity of the destination, it will be considered completed.


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
source devel/setup.bash
```

## 4. Execute the launch file
```bash
export DISABLE_ROS1_EOL_WARNINGS=1
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash

# Try below:
# Try to scan the map by using keyboard:
roslaunch mobile_robot spawn_mobile_robot_gazebo.launch
roslaunch mobile_robot gmapping_rviz.launch
rosrun mobile_robot keyboard_teleop.py

# After completing the map, run the commands below to execute navigation.
roslaunch mobile_robot amcl_move_base.launch
```
