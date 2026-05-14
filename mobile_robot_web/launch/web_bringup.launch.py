"""Bring up rosbridge_websocket + a static HTTP server for the 2D dashboard.

Open http://<host>:8000/ in a browser (redirects to /html/dashboard.html).
The page connects to rosbridge over ws://<host>:9090.

The HTTP server's document root is the package's ``assets/`` directory, so
``/css/...``, ``/js/...``, and ``/html/...`` resolve as absolute URLs.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory('mobile_robot_web')
    web_root = os.path.join(pkg_share, 'assets')

    ws_port = LaunchConfiguration('ws_port')
    http_port = LaunchConfiguration('http_port')

    rosbridge_launch = os.path.join(
        get_package_share_directory('rosbridge_server'),
        'launch', 'rosbridge_websocket_launch.xml',
    )

    return LaunchDescription([
        DeclareLaunchArgument('ws_port', default_value='9090',
                              description='rosbridge WebSocket port'),
        DeclareLaunchArgument('http_port', default_value='8000',
                              description='Static HTTP server port for the HTML pages'),

        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(rosbridge_launch),
            launch_arguments={'port': ws_port}.items(),
        ),

        ExecuteProcess(
            cmd=['python3', '-m', 'http.server', http_port, '--directory', web_root],
            output='screen',
        ),
    ])
