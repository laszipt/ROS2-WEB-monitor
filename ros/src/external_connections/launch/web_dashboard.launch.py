from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('external_connections')
    
    # Parameters
    port = LaunchConfiguration('port', default='8080')
    search_dir = os.path.abspath(pkg_dir)
    default_web_dir = ''
    while True:
        candidate = os.path.join(search_dir, 'web_monitor')
        if os.path.isdir(candidate):
            default_web_dir = candidate
            break
        parent_dir = os.path.dirname(search_dir)
        if parent_dir == search_dir:
            break
        search_dir = parent_dir
    web_dir = LaunchConfiguration('web_dir')
    
    # Launch arguments
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='8080',
        description='Port for web dashboard server'
    )
    web_dir_arg = DeclareLaunchArgument(
        'web_dir',
        default_value=default_web_dir,
        description='Directory containing the web dashboard files'
    )
    
    # Web server node
    web_server_node = Node(
        package='external_connections',
        executable='web_server',
        name='web_dashboard_server',
        parameters=[{
            'port': port,
            'directory': web_dir
        }],
        output='screen'
    )
    
    return LaunchDescription([
        port_arg,
        web_dir_arg,
        web_server_node
    ])
