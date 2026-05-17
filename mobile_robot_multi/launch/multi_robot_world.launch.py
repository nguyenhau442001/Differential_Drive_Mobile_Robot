"""Spawn N differential-drive robots into a single Gazebo Sim world.

Robots are read from a YAML list (config/robots.yaml). For each robot:
  - robot_state_publisher in its own namespace, with frame_prefix
  - ros_gz_sim create to spawn the entity
  - ros_gz_bridge for cmd_vel / odom / imu / scan (namespaced topics)
  - ros_gz_bridge for joint_states (namespaced model topic -> /<ns>/joint_states)
  - static world -> <ns>/odom TF at the spawn pose

A single Gazebo Sim instance is started, and an RViz config is generated on
the fly with one RobotModel/LaserScan/Odometry display group per robot.
"""

import colorsys
import math
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _load_robots(yaml_path):
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)
    return data['robots']


def _grid_robots(n, spacing=1.5):
    """Place N robots on a centered square grid, all facing +X."""
    cols = max(1, math.ceil(math.sqrt(n)))
    rows = math.ceil(n / cols)
    cx = (cols - 1) / 2.0
    cy = (rows - 1) / 2.0
    return [
        {
            'name': f'robot{i + 1}',
            'x': (i % cols - cx) * spacing,
            'y': (i // cols - cy) * spacing,
            'z': 0.0,
            'yaw': 0.0,
        }
        for i in range(n)
    ]


RVIZ_HEADER = """Panels:
  - Class: rviz_common/Displays
    Help Height: 0
    Name: Displays
    Property Tree Widget:
      Expanded:
        - /Global Options1
      Splitter Ratio: 0.5
    Tree Height: 600
  - Class: rviz_common/Selection
    Name: Selection
  - Class: rviz_common/Tool Properties
    Expanded:
      - /Publish Point1
    Name: Tool Properties
    Splitter Ratio: 0.5886790156364441
  - Class: rviz_common/Views
    Expanded:
      - /Current View1
    Name: Views
    Splitter Ratio: 0.3333333432674408
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.5
      Cell Size: 1
      Class: rviz_default_plugins/Grid
      Color: 160; 160; 164
      Enabled: true
      Line Style:
        Line Width: 0.03
        Value: Lines
      Name: Grid
      Normal Cell Count: 0
      Offset:
        X: 0
        Y: 0
        Z: 0
      Plane: XY
      Plane Cell Count: 10
      Reference Frame: <Fixed Frame>
      Value: true
    - Class: rviz_default_plugins/TF
      Enabled: true
      Filter (blacklist): ""
      Filter (whitelist): ""
      Frame Timeout: 15
      Frames:
        All Enabled: true
      Marker Scale: 0.5
      Name: TF
      Show Arrows: false
      Show Axes: true
      Show Names: false
      Update Interval: 0
      Value: true
"""

RVIZ_ROBOT_BLOCK = """    - Class: rviz_common/Group
      Displays:
        - Alpha: 1
          Class: rviz_default_plugins/RobotModel
          Collision Enabled: false
          Description File: ""
          Description Source: Topic
          Description Topic:
            Depth: 5
            Durability Policy: Volatile
            History Policy: Keep Last
            Reliability Policy: Reliable
            Value: /{name}/robot_description
          Enabled: true
          Links:
            All Links Enabled: true
            Expand Joint Details: false
            Expand Link Details: false
            Expand Tree: false
            Link Tree Style: Links in Alphabetic Order
          Mass Properties:
            Inertia: false
            Mass: false
          Name: RobotModel
          TF Prefix: {name}
          Update Interval: 0
          Value: true
          Visual Enabled: true
        - Alpha: 1
          Autocompute Intensity Bounds: true
          Autocompute Value Bounds:
            Max Value: 10
            Min Value: -10
            Value: true
          Axis: Z
          Channel Name: intensity
          Class: rviz_default_plugins/LaserScan
          Color: {color}
          Color Transformer: FlatColor
          Decay Time: 0
          Enabled: true
          Invert Rainbow: false
          Max Color: 255; 255; 255
          Max Intensity: 0
          Min Color: 0; 0; 0
          Min Intensity: 0
          Name: LaserScan
          Position Transformer: XYZ
          Selectable: true
          Size (Pixels): 3
          Size (m): 0.03
          Style: Flat Squares
          Topic:
            Depth: 5
            Durability Policy: Volatile
            Filter size: 10
            History Policy: Keep Last
            Reliability Policy: Best Effort
            Value: /{name}/scan
          Use Fixed Frame: true
          Use rainbow: false
          Value: true
        - Angle Tolerance: 0.1
          Class: rviz_default_plugins/Odometry
          Covariance:
            Orientation:
              Alpha: 0.5
              Color: 255; 255; 127
              Color Style: Unique
              Frame: Local
              Offset: 1
              Scale: 1
              Value: true
            Position:
              Alpha: 0.3
              Color: 204; 51; 204
              Scale: 1
              Value: true
            Value: false
          Enabled: true
          Keep: 50
          Name: Odometry
          Position Tolerance: 0.1
          Shape:
            Alpha: 1
            Axes Length: 0.3
            Axes Radius: 0.03
            Color: {color}
            Head Length: 0.1
            Head Radius: 0.05
            Shaft Length: 0.2
            Shaft Radius: 0.02
            Value: Arrow
          Topic:
            Depth: 5
            Durability Policy: Volatile
            Filter size: 10
            History Policy: Keep Last
            Reliability Policy: Best Effort
            Value: /{name}/odom
          Value: true
      Enabled: true
      Name: {name}
"""

RVIZ_FOOTER = """  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: world
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/FocusCamera
    - Class: rviz_default_plugins/Measure
      Line color: 128; 128; 0
    - Class: rviz_default_plugins/PublishPoint
      Single click: true
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /clicked_point
  Transformation:
    Current:
      Class: rviz_default_plugins/TF
  Value: true
  Views:
    Current:
      Angle: 0
      Class: rviz_default_plugins/TopDownOrtho
      Enable Stereo Rendering:
        Stereo Eye Separation: 0.06
        Stereo Focal Distance: 1
        Swap Stereo Eyes: false
        Value: false
      Invert Z Axis: false
      Name: Current View
      Near Clip Distance: 0.01
      Scale: 80
      Target Frame: <Fixed Frame>
      Value: TopDownOrtho (rviz_default_plugins)
      X: 0
      Y: 0
    Saved: ~
"""


def _color_for(i, n):
    r, g, b = colorsys.hsv_to_rgb(i / max(n, 1), 0.7, 0.9)
    return f'{int(r * 255)}; {int(g * 255)}; {int(b * 255)}'


def _generate_rviz_config(robots, output_path):
    n = len(robots)
    blocks = ''.join(
        RVIZ_ROBOT_BLOCK.format(name=r['name'], color=_color_for(i, n))
        for i, r in enumerate(robots)
    )
    with open(output_path, 'w') as f:
        f.write(RVIZ_HEADER + blocks + RVIZ_FOOTER)
    return output_path


def _spawn_action(robot):
    """The gz `create` Node — kept separate so spawns can be chained sequentially."""
    name = robot['name']
    return Node(
        package='ros_gz_sim',
        executable='create',
        name=f'spawn_{name}',
        arguments=[
            '--name', name,
            '--topic', f'/{name}/robot_description',
            '--x', str(robot.get('x', 0.0)),
            '--y', str(robot.get('y', 0.0)),
            '--z', str(robot.get('z', 0.0)),
            '--Y', str(robot.get('yaw', 0.0)),
        ],
        output='screen',
    )


def _robot_aux_actions(robot, xacro_file):
    """All per-robot actions except the gz spawn — RSP, bridges, static TF."""
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

    # robot_state_publisher's TransformBroadcaster publishes to the absolute
    # /tf, so the namespace does NOT push it to /<ns>/tf — all RSPs share /tf
    # and rely on prefixed frame names to stay disjoint. use_tf_static=False
    # streams fixed-joint transforms on /tf at publish_frequency instead of
    # latching them on /tf_static.
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=name,
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'frame_prefix': f'{name}/',
            'use_sim_time': True,
            'use_tf_static': False,
            'publish_frequency': 30.0,
        }],
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

    # TF: DiffDrive plugin publishes odom -> <name>/chassis on Gazebo's
    # /model/<name>/tf — bridge it onto the global /tf so it joins the rest
    # of the tree (RSP publishes chassis -> wheels there too).
    tf_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name=f'tf_bridge_{name}',
        arguments=[
            f'/model/{name}/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ],
        remappings=[
            (f'/model/{name}/tf', '/tf'),
        ],
        output='screen',
    )

    # Static world -> <name>/odom transform from the spawn pose. Stays global
    # (no namespace) so it lands on the shared /tf_static — each robot owns a
    # unique (world, <name>/odom) frame pair so the latched messages don't
    # collide between publishers.
    world_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=f'world_to_{name}_odom',
        arguments=[
            '--x', x, '--y', y, '--z', z,
            '--yaw', yaw, '--pitch', '0', '--roll', '0',
            '--frame-id', 'world',
            '--child-frame-id', f'{name}/odom',
        ],
        output='screen',
    )

    return [rsp, sensor_bridge, joint_state_bridge, tf_bridge, world_to_odom]


def generate_launch_description():
    pkg_multi = get_package_share_directory('mobile_robot_multi')
    pkg_gazebo = get_package_share_directory('mobile_robot_gazebo')
    pkg_description = get_package_share_directory('mobile_robot_description')

    default_robots_file = os.path.join(pkg_multi, 'config', 'robots.yaml')
    default_world = os.path.join(pkg_gazebo, 'worlds', '10x10.world')
    xacro_file = os.path.join(pkg_description, 'urdf', 'mobile_robot.urdf.xacro')

    # Resolve robot list at parse time so we can loop in Python. If N_ROBOTS is
    # set, auto-arrange that many on a grid; otherwise read from robots.yaml
    # (override path via ROBOTS_FILE env var).
    n_robots_env = os.environ.get('N_ROBOTS')
    if n_robots_env:
        n_robots = int(n_robots_env)
        robots = _grid_robots(n_robots)
        print(f'[multi_robot_world] grid layout: {n_robots} robots')
    else:
        robots_file = os.environ.get('ROBOTS_FILE', default_robots_file)
        robots = _load_robots(robots_file)

    default_rviz_config = _generate_rviz_config(
        robots, '/tmp/mobile_robot_multi.rviz'
    )
    print(f'[multi_robot_world] generated RViz config: {default_rviz_config}')

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
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Whether to launch RViz',
    )
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_config,
        description='RViz config file to load',
    )

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

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen',
    )

    actions = [
        # Force EGL to use X11 platform so sensor render thread can fall back to llvmpipe.
        SetEnvironmentVariable('EGL_PLATFORM', 'x11'),
        robots_file_arg,
        world_arg,
        rviz_arg,
        rviz_config_arg,
        gz_sim,
        rviz,
    ]
    # Aux nodes (RSP, bridges, static TF) fire concurrently at t=0 — they're
    # idempotent against late Gazebo topics and the latched URDFs need to be
    # on the network before any spawn subscribes.
    for r in robots:
        actions.extend(_robot_aux_actions(r, xacro_file))

    # Chain `ros_gz_sim create` calls: each spawn fires only after the previous
    # one's process exits. Gazebo's /world/default/create service is
    # single-threaded and DiffDrive-plugin init per model takes real time, so
    # serialising spawns eliminates the race that drops the last entity at high
    # N. Total launch time becomes the sum of actual spawn durations — no slack.
    spawns = [_spawn_action(r) for r in robots]
    for prev, curr in zip(spawns, spawns[1:]):
        actions.append(RegisterEventHandler(
            OnProcessExit(target_action=prev, on_exit=[curr]),
        ))
    actions.append(spawns[0])

    return LaunchDescription(actions)
