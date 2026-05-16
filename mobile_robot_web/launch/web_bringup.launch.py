"""Bring up rosbridge_websocket + a static HTTP server for the dashboard.

Open http://<host>:8000/ in a browser (redirects to /html/dashboard.html).
The page connects to rosbridge over ws://<host>:9090.

A single HTTP server serves:
  - the dashboard assets at /            (this package's ``assets/``)
  - the URDF meshes      at /robot_assets/ (mobile_robot_description's share)

Routing them through one port avoids cross-origin requests entirely — the
3D view's STL fetches are same-origin, so no CORS / preflight / cache
shenanigans are needed.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    pkg_share = get_package_share_directory('mobile_robot_web')
    web_root = os.path.join(pkg_share, 'assets')
    description_share = get_package_share_directory('mobile_robot_description')

    ws_port = LaunchConfiguration('ws_port')
    http_port = LaunchConfiguration('http_port')
    open_browser = LaunchConfiguration('open_browser')
    url = PythonExpression(["'http://localhost:' + '", http_port, "'"])

    rosbridge_launch = os.path.join(
        get_package_share_directory('rosbridge_server'),
        'launch', 'rosbridge_websocket_launch.xml',
    )

    # Single-port HTTP server with a path-prefix route for mesh files.
    # Inlined here so the launch file is self-contained. The handler
    # rewrites incoming /robot_assets/<rel> URLs to point at the
    # description package's share dir; everything else falls through to
    # the default SimpleHTTPRequestHandler logic against web_root.
    # Cache-Control: no-store on EVERY response so the browser never
    # serves a stale JS / HTML / mesh after a code change. Costs nothing
    # for a dev dashboard and eliminates an entire class of "I edited
    # the code but the browser shows the old version" bugs.
    server_script = (
        'import http.server, socketserver, sys, os, urllib.parse\n'
        'web_root, mesh_root, port = sys.argv[1], sys.argv[2], int(sys.argv[3])\n'
        'MESH_PREFIX = "/robot_assets/"\n'
        'class H(http.server.SimpleHTTPRequestHandler):\n'
        '    protocol_version = "HTTP/1.1"\n'
        '    def end_headers(self):\n'
        '        self.send_header("Cache-Control", "no-store, must-revalidate")\n'
        '        super().end_headers()\n'
        '    def translate_path(self, path):\n'
        '        p = urllib.parse.urlsplit(path).path\n'
        '        if p.startswith(MESH_PREFIX):\n'
        '            rel = p[len(MESH_PREFIX):].lstrip("/")\n'
        '            return os.path.join(mesh_root, rel)\n'
        '        return super().translate_path(path)\n'
        'handler = lambda *a, **kw: H(*a, directory=web_root, **kw)\n'
        'socketserver.ThreadingTCPServer.allow_reuse_address = True\n'
        'with socketserver.ThreadingTCPServer(("", port), handler) as srv:\n'
        '    print(f"dashboard on :{port}, meshes under {MESH_PREFIX}", flush=True)\n'
        '    srv.serve_forever()\n'
    )

    return LaunchDescription([
        DeclareLaunchArgument('ws_port', default_value='9090',
                              description='rosbridge WebSocket port'),
        DeclareLaunchArgument('http_port', default_value='8000',
                              description='HTTP server port (serves both HTML and meshes)'),
        DeclareLaunchArgument('open_browser', default_value='true',
                              description='Open the dashboard in the default browser on launch'),

        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(rosbridge_launch),
            launch_arguments={'port': ws_port}.items(),
        ),

        ExecuteProcess(
            cmd=['python3', '-c', server_script, web_root, description_share, http_port],
            output='screen',
        ),

        TimerAction(
            period=1.5,
            actions=[ExecuteProcess(
                cmd=['xdg-open', url],
                output='screen',
                condition=IfCondition(open_browser),
            )],
        ),
    ])
