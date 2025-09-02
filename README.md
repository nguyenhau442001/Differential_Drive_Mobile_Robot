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
  ros-noetic-gmapping ros-noetic-navigation ros-noetic-interactive-markers ]
  ros-noetic-tf2-tools
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

## 4. Robot description
  Robot Structure Overview:
  - Chassis (base)
  - 4 Caster links (non-driven support wheels)
  - Left & Right drive wheels
  - Lidar sensor
<img width="1824" height="759" alt="image" src="https://github.com/user-attachments/assets/f2a69b68-876a-4dc6-a20b-09cbea5dca7f" />
<img width="1824" height="759" alt="image" src="https://github.com/user-attachments/assets/bc2bd408-ba4a-40ff-aa22-a653fca2b56f" />
<img width="1824" height="759" alt="image" src="https://github.com/user-attachments/assets/b5821ce8-12c4-463e-9612-f24e65e3f2ed" />
<img width="1824" height="759" alt="image" src="https://github.com/user-attachments/assets/b8801daf-88c0-42fd-bf59-0ac4c683639e" />
Transformation:

```bash
rosrun tf view_frames
evince frames.pdf
or
rosrun tf2_tools view_frames.py
evince frames.pdf
```
<img width="1668" height="518" alt="image" src="https://github.com/user-attachments/assets/4cd4ff02-b105-43eb-bfea-ef65a60f88bb" />

## Gmapping
Every new shell has to run the commands below to detect the ROS packages.
```bash
echo 'export DISABLE_ROS1_EOL_WARNINGS=1' >> ~/.bashrc
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
```

```bash
# First terminal: launch the mobile robot in Gazebo (in default, the 10x10 world is being used)
# Choose one in 3 below command
roslaunch mobile_robot_gazebo mobile_robot_empty_world.launch
roslaunch mobile_robot_gazebo mobile_robot_10x10_world.launch
roslaunch mobile_robot_gazebo mobile_robot_20x20_world.launch
```
<img width="1842" height="787" alt="image" src="https://github.com/user-attachments/assets/73d45c8f-ca78-4eba-aee3-7f56daa7a36d" />


```bash
# Second terminal: launch RViz and get ready to scan the map
roslaunch mobile_robot_slam mobile_robot_slam.launch
```
<img width="1842" height="787" alt="image" src="https://github.com/user-attachments/assets/16e2d4af-8863-4b9d-b93c-933b138d0e8b" />

<img width="1842" height="787" alt="image" src="https://github.com/user-attachments/assets/9b732f06-5fa5-4dbb-bd24-c7150c04f626" />

```bash
# Third terminal: scan the map by driving the robot with the keyboard
roslaunch mobile_robot_teleop mobile_robot_teleop_key.launch

hau@hau-VirtualBox:~/catkin_ws$ roslaunch mobile_robot_teleop mobile_robot_teleop_key.launch

Control Your Differential-Drive Mobile Robot!!!
---------------------------
Moving around:
        w
   a    s    d
        x

w/x : increase/decrease linear velocity 
a/d : increase/decrease angular velocity

space key, s : force stop

CTRL-C to quit
```

Fourth terminal: save the map
```bash
rosrun map_server map_saver -f mobile_robot/maps/
```

## Navigation
```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash

# After completing the map, run the commands below to execute navigation.
roslaunch mobile_robot_navigation mobile_robot_navigation.launch
```
<img width="1839" height="876" alt="image" src="https://github.com/user-attachments/assets/21b207db-d197-46dd-814f-11dad260dea4" />

In rviz, click on 2D Pose Estimate and set initial pose estimate of the robot.
To move to a goal, click on 2D Nav Goal to set your goal location and pose.
<img width="1817" height="835" alt="image" src="https://github.com/user-attachments/assets/3aa62a34-f76b-467e-8cc2-cf3bae7c9bd4" />
<img width="1817" height="835" alt="image" src="https://github.com/user-attachments/assets/ca103906-d5ed-46c6-affd-837b079a9dd5" />
<img width="1817" height="835" alt="image" src="https://github.com/user-attachments/assets/aee2a359-069e-47e6-b6ca-138fd3075ada" />


## Draw the graphic
rosrun rqt_plot rqt_plot

