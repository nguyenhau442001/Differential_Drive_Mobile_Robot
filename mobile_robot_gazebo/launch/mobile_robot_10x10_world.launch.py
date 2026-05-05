from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    x_pos = LaunchConfiguration('x_pos')
    y_pos = LaunchConfiguration('y_pos')
    z_pos = LaunchConfiguration('z_pos')

    declare_x = DeclareLaunchArgument('x_pos', default_value='0.0')
    declare_y = DeclareLaunchArgument('y_pos', default_value='0.0')
    declare_z = DeclareLaunchArgument('z_pos', default_value='0.0')

    pkg_gazebo = get_package_share_directory('mobile_robot_gazebo')
    pkg_description = get_package_share_directory('mobile_robot_description')

    world_file = os.path.join(pkg_gazebo, 'worlds', '10x10.world')
    xacro_file = os.path.join(pkg_description, 'urdf', 'mobile_robot.urdf.xacro')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        # -r  = start running immediately (not paused)
        # without -r, UserCommands system may not advertise /world/default/create
        launch_arguments={'gz_args': '-r ' + world_file}.items()
    )

    state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen'
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '--name', 'mobile_robot',
            '--topic', 'robot_description',
            '--x', x_pos,
            '--y', y_pos,
            '--z', z_pos
        ],
        output='screen'
    )

    # Bridges between ROS 2 and Gazebo transport
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # cmd_vel: ROS → Gz  (keyboard teleop → DiffDrive plugin)
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            # odom: Gz → ROS  (DiffDrive plugin → nav stack)
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            # imu: Gz → ROS  (topic matches <topic> in sensor definition)
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            # laser scan: Gz → ROS  (topic matches <topic> in sensor definition)
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ],
        output='screen'
    )

    # Bridge joint states so robot_state_publisher can publish wheel/caster TF
    joint_state_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/default/model/mobile_robot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        remappings=[
            ('/world/default/model/mobile_robot/joint_state', '/joint_states'),
        ],
        output='screen'
    )

    return LaunchDescription([
        # Force EGL to use X11 platform so the sensor render thread can use
        # llvmpipe (software OpenGL) via GLX instead of failing on EGL device mode
        SetEnvironmentVariable('EGL_PLATFORM', 'x11'),
        declare_x,
        declare_y,
        declare_z,
        gz_sim,
        state_pub,
        spawn_robot,
        ros_gz_bridge,
        joint_state_bridge,
    ])
