# Project Description:
Build a simulation for a mobile robot using **Gazebo Sim (Harmonic)** and **ROS 2 Jazzy**.
- **mobile_robot_description** → Robot geometry and physical description (URDF/xacro).
- **mobile_robot_gazebo** → Launch files for spawning the robot in Gazebo and bridging topics.
- **mobile_robot_navigation2** → Nav2 launch and configuration (AMCL, DWB controller, NavFn planner).
- **mobile_robot_teleop** → Python nodes for teleoperation (keyboard control, trapezoidal velocity controller).


## 1. Environment Setup
Install ROS 2 Jazzy: https://docs.ros.org/en/jazzy/Installation.html
```bash
sudo apt update
sudo apt install \
  ros-jazzy-desktop \
  ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox \
  ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-sim \
  ros-jazzy-teleop-twist-keyboard \
  ros-jazzy-xacro ros-jazzy-tf2-tools \
  ros-jazzy-robot-localization
```

## 2. Clone the source code from GitHub and build the packages
```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone git@github.com:nguyenhau442001/Differential_Drive_Mobile_Robot.git
cd ~/ros2_ws
colcon build --symlink-install
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

## 3. Robot description
  Robot Structure Overview:
  - Chassis (base)
  - 4 Caster links (non-driven support wheels)
  - Left & Right drive wheels
  - IMU sensor
  - Lidar sensor (360°, 1440 samples, 10 m range)

<img width="1824" height="759" alt="image" src="https://github.com/user-attachments/assets/f2a69b68-876a-4dc6-a20b-09cbea5dca7f" />
<img width="1824" height="759" alt="image" src="https://github.com/user-attachments/assets/bc2bd408-ba4a-40ff-aa22-a653fca2b56f" />
<img width="1824" height="759" alt="image" src="https://github.com/user-attachments/assets/b5821ce8-12c4-463e-9612-f24e65e3f2ed" />
<img width="1824" height="759" alt="image" src="https://github.com/user-attachments/assets/b8801daf-88c0-42fd-bf59-0ac4c683639e" />

To view the TF tree, run:

```bash
ros2 run tf2_tools view_frames
evince frames.pdf
```
<img width="1660" height="355" alt="image" src="https://github.com/user-attachments/assets/f0e7d94b-a6f3-4449-afa0-6563179bcbc4" />


## 4. SLAM
SLAM uses **slam_toolbox** (the standard 2D-lidar SLAM stack on ROS 2). Every new shell needs the workspace sourced:
```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

```bash
# First terminal: launch the mobile robot in Gazebo (10x10 world by default)
ros2 launch mobile_robot_gazebo mobile_robot_10x10_world.launch.py
```
<img width="1842" height="787" alt="image" src="https://github.com/user-attachments/assets/73d45c8f-ca78-4eba-aee3-7f56daa7a36d" />


```bash
# Second terminal: launch slam_toolbox to start building the map
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true
```

<img width="1842" height="787" alt="image" src="https://github.com/user-attachments/assets/9b732f06-5fa5-4dbb-bd24-c7150c04f626" />

```bash
# Third terminal: drive the robot with the keyboard to scan the environment
ros2 run mobile_robot_teleop mobile_robot_teleop_key --ros-args -r cmd_vel:=/cmd_vel

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

Fourth terminal: save the map (creates `map.yaml` and `map.pgm`)
```bash
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/Differential_Drive_Mobile_Robot/mobile_robot_navigation2/map/map
```

## 5. Navigation (Nav2 with DWB controller and NavFn planner)
Every new shell needs the workspace sourced:
```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

```bash
# First terminal: launch the mobile robot in Gazebo (10x10 world by default)
ros2 launch mobile_robot_gazebo mobile_robot_10x10_world.launch.py
```

```bash
# Second terminal: launch Nav2 with the saved map
ros2 launch mobile_robot_navigation2 navigation2.launch.py
```
<img width="1817" height="835" alt="image" src="https://github.com/user-attachments/assets/21b207db-d197-46dd-814f-11dad260dea4" />

In RViz, click **2D Pose Estimate** and set the initial pose of the robot.
To move to a goal, click **Nav2 Goal** and set the goal location and pose.
<img width="1817" height="835" alt="image" src="https://github.com/user-attachments/assets/3aa62a34-f76b-467e-8cc2-cf3bae7c9bd4" />
<img width="1817" height="835" alt="image" src="https://github.com/user-attachments/assets/ca103906-d5ed-46c6-affd-837b079a9dd5" />
<img width="1817" height="835" alt="image" src="https://github.com/user-attachments/assets/aee2a359-069e-47e6-b6ca-138fd3075ada" />

## 6. Controller
### Verify the velocity
```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run rqt_plot rqt_plot
ros2 launch mobile_robot_gazebo mobile_robot_empty_world.launch.py
ros2 run mobile_robot_teleop mobile_robot_teleop_key --ros-args -r cmd_vel:=/cmd_vel
```

Add the two topics below to the plot to check the velocity response when increasing velocity slowly:

`/cmd_vel/linear/x`          => Set Point (SP)

`/odom/twist/twist/linear/x` => Process Variable (PV)

To monitor acceleration, add `/imu/linear_acceleration/x` to rqt_plot.

<img width="1817" height="835" alt="image" src="https://github.com/user-attachments/assets/3ed75f91-36b0-4e06-a09b-962bb9176319" />

---------------
# Case study: The robot should reach 5 m/s within 5 s from standstill, and stop completely within 1 s.

To meet this requirement, a controller was created based on the profile below:

The motion has three distinct phases:

- Acceleration phase (`t_acc`): Velocity increases linearly from 0 → `v_target`.
- Cruise phase (`t_cruise`):    Velocity stays constant at `v_target`.
- Deceleration phase (`t_dec`): Velocity decreases smoothly to 0 using a cosine function.

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
linear velocity commands to the `/cmd_vel` topic.

The profile consists of three phases:

  1. Acceleration phase - linearly increases velocity from 0 to `v_target`
  2. Cruise phase       - maintains a constant target velocity
  3. Deceleration phase - smoothly decreases velocity to 0 using a cosine function

Why discretize?
---------------
Instead of sending the velocity set point (`v_target`) directly to the robot,
we ramp velocity up and down over time (`dt`). This prevents jerky motion and
improves robot stability.


The `trapezoid_cmd_vel_node` was created to send the desired velocity to the robot.

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch mobile_robot_gazebo mobile_robot_empty_world.launch.py
ros2 launch mobile_robot_teleop trapezoid_profile_controller.launch.py
```

The robot accelerates from standstill (t = 9) to 5 m/s (t = 14), then decelerates from 5 m/s (t = 14) to 0 m/s (t = 15).
<img width="1827" height="746" alt="image" src="https://github.com/user-attachments/assets/e92fa2c7-2004-4760-afc4-ae60f8adcf35" />
