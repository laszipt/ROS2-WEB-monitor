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
    web_dir = '/home/tamas/BORS1-ROS2/web_monitor'
    
    # Launch arguments
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='8080',
        description='Port for web dashboard server'
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
        web_server_node
    ])
