"""Spawn N differential-drive robots into a single Gazebo Sim world.

Robots are read from a YAML list (config/robots.yaml). For each robot:
  - robot_state_publisher in its own namespace, with frame_prefix
  - ros_gz_sim create to spawn the entity
  - ros_gz_bridge for cmd_vel / odom / imu / scan (namespaced topics)
  - ros_gz_bridge for joint_states (namespaced model topic -> /<ns>/joint_states)

A single Gazebo Sim instance is started once.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command


def _load_robots(yaml_path):
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)
    return data['robots']


def _robot_actions(robot, xacro_file):
    name = robot['name']
    x = str(robot.get('x', 0.0))
    y = str(robot.get('y', 0.0))
    z = str(robot.get('z', 0.0))
    yaw = str(robot.get('yaw', 0.0))

    # Per-robot URDF (frame prefixed inside the URDF by the gazebo xacro).
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' robot_name:=', name]),
        value_type=str,
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=name,
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'frame_prefix': f'{name}/',
            'use_sim_time': True,
        }],
        output='screen',
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        name=f'spawn_{name}',
        arguments=[
            '--name', name,
            '--topic', f'/{name}/robot_description',
            '--x', x,
            '--y', y,
            '--z', z,
            '--Y', yaw,
        ],
        output='screen',
    )

    # cmd_vel / odom / imu / scan bridge — all topics already namespaced by the xacro.
    sensor_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name=f'bridge_{name}',
        arguments=[
            f'/{name}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            f'/{name}/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            f'/{name}/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            f'/{name}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ],
        output='screen',
    )

    # Joint states: Gazebo emits on /world/<world>/model/<name>/joint_state — remap
    # into the robot's namespace so its RSP picks them up.
    joint_state_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name=f'joint_state_bridge_{name}',
        arguments=[
            f'/world/default/model/{name}/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        remappings=[
            (f'/world/default/model/{name}/joint_state', f'/{name}/joint_states'),
        ],
        output='screen',
    )

    return [rsp, spawn, sensor_bridge, joint_state_bridge]


def generate_launch_description():
    pkg_multi = get_package_share_directory('mobile_robot_multi')
    pkg_gazebo = get_package_share_directory('mobile_robot_gazebo')
    pkg_description = get_package_share_directory('mobile_robot_description')

    default_robots_file = os.path.join(pkg_multi, 'config', 'robots.yaml')
    default_world = os.path.join(pkg_gazebo, 'worlds', '10x10.world')
    xacro_file = os.path.join(pkg_description, 'urdf', 'mobile_robot.urdf.xacro')

    robots_file_arg = DeclareLaunchArgument(
        'robots_file',
        default_value=default_robots_file,
        description='YAML file listing robots (name + pose) to spawn',
    )
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='Path to the SDF/.world file Gazebo loads',
    )

    # Resolve robots_file at parse time so we can loop in Python.
    # If the user overrides on the command line, we still read the override here.
    robots_file = os.environ.get('ROBOTS_FILE', default_robots_file)
    robots = _load_robots(robots_file)

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py',
            )
        ),
        launch_arguments={'gz_args': ['-r ', LaunchConfiguration('world')]}.items(),
    )

    actions = [
        # Force EGL to use X11 platform so sensor render thread can fall back to llvmpipe.
        SetEnvironmentVariable('EGL_PLATFORM', 'x11'),
        robots_file_arg,
        world_arg,
        gz_sim,
    ]
    for r in robots:
        actions.extend(_robot_actions(r, xacro_file))

    return LaunchDescription(actions)
