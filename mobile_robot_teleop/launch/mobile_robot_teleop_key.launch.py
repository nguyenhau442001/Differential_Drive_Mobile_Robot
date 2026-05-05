from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mobile_robot_teleop',
            executable='mobile_robot_teleop_key',
            name='mobile_robot_teleop_keyboard',
            output='screen',
            prefix='xterm -e',
        ),
    ])
