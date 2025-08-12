from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
    rosbridge_pkg = get_package_share_directory('rosbridge_server')
    
    # Parameters
    port = LaunchConfiguration('port', default='9090')
    
    # Launch arguments
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='9090',
        description='Port for rosbridge WebSocket server'
    )
    
    # Launch rosbridge directly using Node
    rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        parameters=[{
            'port': port,
            # Accept connections from any network interface
            'address': '0.0.0.0',
            # Disable origin filtering so browsers on any host can connect
            'allowed_origins': ['*']
        }],
        output='screen'
    )
    
    # Launch rosapi node
    rosapi_node = Node(
        package='rosapi',
        executable='rosapi_node',
        name='rosapi',
        output='screen'
    )
    
    return LaunchDescription([
        port_arg,
        rosbridge_node,
        rosapi_node
    ])
