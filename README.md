# Project Description:
Choose Gazebo and ROS1 Noetic, and build a simulation for a mobile robot.
- **mobile_robot_description** → Contains the robot’s geometry and physical description.
- **mobile_robot_gazebo** → Contains the launch files for spawning the robot in Gazebo.
- **mobile_robot_navigation** → Contains the launch files to run path planners, such as DWA.
- **mobile_robot_slam** → Contains the launch files for running SLAM.
- **mobile_robot_teleop** → Contains Python nodes for teleoperation, including keyboard control and differential drive controller.


## 1. Environment Setup
Install ROS1 Noetic: https://wiki.ros.org/noetic/Installation/Ubuntu
```bash
sudo apt install python3-roslaunch
sudo apt-get install ros-noetic-joy ros-noetic-teleop-twist-joy \
  ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc \
  ros-noetic-rgbd-launch ros-noetic-rosserial-arduino \
  ros-noetic-rosserial-python ros-noetic-rosserial-client \
  ros-noetic-rosserial-msgs ros-noetic-amcl ros-noetic-map-server \
  ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro \
  ros-noetic-compressed-image-transport ros-noetic-rqt* ros-noetic-rviz \
  ros-noetic-gmapping ros-noetic-navigation ros-noetic-interactive-markers ]
  ros-noetic-tf2-tools ros-noetic-robot-localization
```

## 2. Clone the source code from gitHub and build the packages
```bash
mkdir ~/catkin_ws/src && cd ~/catkin_ws/src
git clone git@github.com:nguyenhau442001/Differential_Drive_Mobile_Robot.git && cd ..
catkin_make
source devel/setup.bash
source /opt/ros/noetic/setup.bash
```

## 3. Robot description
  Robot Structure Overview:
  - Chassis (base)
  - 4 Caster links (non-driven support wheels)
  - Left & Right drive wheels
  - IMU sensor
  - Lidar sensor

<img width="1824" height="759" alt="image" src="https://github.com/user-attachments/assets/f2a69b68-876a-4dc6-a20b-09cbea5dca7f" />
<img width="1824" height="759" alt="image" src="https://github.com/user-attachments/assets/bc2bd408-ba4a-40ff-aa22-a653fca2b56f" />
<img width="1824" height="759" alt="image" src="https://github.com/user-attachments/assets/b5821ce8-12c4-463e-9612-f24e65e3f2ed" />
<img width="1824" height="759" alt="image" src="https://github.com/user-attachments/assets/b8801daf-88c0-42fd-bf59-0ac4c683639e" />
To view the the transformation, please use the below commands:

```bash
rosrun tf view_frames
evince frames.pdf
or
rosrun tf2_tools view_frames.py
evince frames.pdf
```
<img width="1660" height="355" alt="image" src="https://github.com/user-attachments/assets/f0e7d94b-a6f3-4449-afa0-6563179bcbc4" />


## 4. SLAM
Every new shell has to run the commands below to detect the ROS packages.
```bash
echo 'export DISABLE_ROS1_EOL_WARNINGS=1' >> ~/.bashrc
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
```

```bash
# First terminal: launch the mobile robot in Gazebo (in default, the 10x10 world is being used)
roslaunch mobile_robot_gazebo mobile_robot_10x10_world.launch
```
<img width="1842" height="787" alt="image" src="https://github.com/user-attachments/assets/73d45c8f-ca78-4eba-aee3-7f56daa7a36d" />


```bash
# Second terminal: launch RViz and get ready to scan the map
roslaunch mobile_robot_slam mobile_robot_slam.launch
```

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
rosrun map_server map_saver -f <path_project>/mobile_robot_navigation/maps
```

## 5. Navigation (use DWA algorithm for the path planner)
```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash

# After completing the map, run the commands below to execute navigation.
roslaunch mobile_robot_gazebo mobile_robot_10x10_world.launch
roslaunch mobile_robot_navigation mobile_robot_navigation.launch
```
<img width="1817" height="835" alt="image" src="https://github.com/user-attachments/assets/21b207db-d197-46dd-814f-11dad260dea4" />

In rviz, click on 2D Pose Estimate and set initial pose estimate of the robot.
To move to a goal, click on 2D Nav Goal to set your goal location and pose.
<img width="1817" height="835" alt="image" src="https://github.com/user-attachments/assets/3aa62a34-f76b-467e-8cc2-cf3bae7c9bd4" />
<img width="1817" height="835" alt="image" src="https://github.com/user-attachments/assets/ca103906-d5ed-46c6-affd-837b079a9dd5" />
<img width="1817" height="835" alt="image" src="https://github.com/user-attachments/assets/aee2a359-069e-47e6-b6ca-138fd3075ada" />

## 6. Controller
### Verify the velocity
```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rosrun rqt_plot rqt_plot
roslaunch mobile_robot_gazebo mobile_robot_empty_world.launch
roslaunch mobile_robot_teleop mobile_robot_teleop_key.launch
```

Add two below topics in the plots, just to check the velocity response by increasing the velocity slowly.

/cmd_vel/linear/x          => Set Point (SP)

/odom/twist/twist/linear/x => Process Variable (PV)

To monitor the acceleration, add this topic in rqt_tool tool: /imu/linear_acceleration/x

<img width="1817" height="835" alt="image" src="https://github.com/user-attachments/assets/3ed75f91-36b0-4e06-a09b-962bb9176319" />

---------------
# The case: The robot should be able to achieve 5m/s velocity within 5s from stand still, and should completely stop within 1s.

To meet the requirement, a controller was created to control the robot based on the pilot below:

The motion has three distinct phases:

Phases Explained

- Acceleration phase (t_acc): Velocity increases linearly from 0 → v_target.

- Cruise phase (t_cruise):    Velocity stays constant at v_target.

- Deceleration phase (t_dec): Velocity smoothly decreases to 0 using a cosine function for smooth stopping.

```
velocity
   ^
   |           ┌──────────────┐
   |          /|              |\
   |         / |              | \
   |        /  |              |  \
   |       /   |              |   \
   +-----------------------------------> time
       Accel     Cruise       Decel
```

ROS Node: Trapezoidal Velocity Controller
-----------------------------------------
This node generates a trapezoidal velocity profile and publishes
linear velocity commands to the /cmd_vel topic.

The profile consists of three phases:

    1. Acceleration Phase - Linearly increases velocity from 0 to v_target

    2. Cruise Phase       - Maintains a constant target velocity
  
    3. Deceleration Phase - Smoothly decreases velocity to 0 using a cosine function

Why discretize?
---------------
Instead of directly sending the velocity set point (v_target) to the robot,
we gradually ramp up and ramp down the velocity over time (dt). This prevents
jerky motion and helps maintain robot stability.


Therefore, with this approach, the trapezoid_cmd_vel_node was created to send the desired velocity to the robot.

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch mobile_robot_gazebo mobile_robot_empty_world.launch
roslaunch mobile_robot_teleop trapezoid_profile_controller.launch
```

As you can see, the robot can go from standing still (t = 9) to reaching 5 m/s (t = 14), and then decelerate from 5 m/s (t = 14) to 0 m/s (t = 15).
<img width="1827" height="746" alt="image" src="https://github.com/user-attachments/assets/e92fa2c7-2004-4760-afc4-ae60f8adcf35" />
