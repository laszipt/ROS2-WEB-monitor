from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('external_connections')
    
    # Launch arguments
    use_rosbridge = LaunchConfiguration('use_rosbridge', default='true')
    use_foxglove = LaunchConfiguration('use_foxglove', default='true')
    use_webserver = LaunchConfiguration('use_webserver', default='true')
    rosbridge_port = LaunchConfiguration('rosbridge_port', default='9090')
    foxglove_port = LaunchConfiguration('foxglove_port', default='8765')
    webserver_port = LaunchConfiguration('webserver_port', default='8080')
    
    # Launch file paths
    rosbridge_launch = os.path.join(pkg_dir, 'launch', 'rosbridge_websocket.launch.py')
    foxglove_launch = os.path.join(pkg_dir, 'launch', 'foxglove_bridge.launch.py')
    webserver_launch = os.path.join(pkg_dir, 'launch', 'web_dashboard.launch.py')
    
    # Launch arguments
    declare_use_rosbridge = DeclareLaunchArgument(
        'use_rosbridge', default_value='true',
        description='Whether to launch the ROS bridge server'
    )
    
    declare_use_foxglove = DeclareLaunchArgument(
        'use_foxglove', default_value='true',
        description='Whether to launch the Foxglove bridge'
    )
    
    declare_use_webserver = DeclareLaunchArgument(
        'use_webserver', default_value='true',
        description='Whether to launch the web dashboard server'
    )
    
    declare_rosbridge_port = DeclareLaunchArgument(
        'rosbridge_port', default_value='9090',
        description='Port for rosbridge WebSocket server'
    )
    
    declare_foxglove_port = DeclareLaunchArgument(
        'foxglove_port', default_value='8765',
        description='Port for Foxglove WebSocket server'
    )
    
    declare_webserver_port = DeclareLaunchArgument(
        'webserver_port', default_value='8080',
        description='Port for web dashboard server'
    )
    
    # Connection manager node
    connection_manager_node = Node(
        package='external_connections',
        executable='connection_manager',
        name='connection_manager',
        parameters=[{
            'rosbridge_port': rosbridge_port,
            'foxglove_port': foxglove_port,
            'webserver_port': webserver_port
        }],
        output='screen'
    )
    
    # Define conditional launch based on parameters
    def launch_connections(context):
        actions = [connection_manager_node]
        
        if context.perform_substitution(use_rosbridge).lower() == 'true':
            actions.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource(rosbridge_launch),
                launch_arguments={'port': context.perform_substitution(rosbridge_port)}.items()
            ))
        
        if context.perform_substitution(use_foxglove).lower() == 'true':
            actions.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource(foxglove_launch),
                launch_arguments={'port': context.perform_substitution(foxglove_port)}.items()
            ))
        
        if context.perform_substitution(use_webserver).lower() == 'true':
            actions.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource(webserver_launch),
                launch_arguments={'port': context.perform_substitution(webserver_port)}.items()
            ))
            
        return actions
    
    return LaunchDescription([
        declare_use_rosbridge,
        declare_use_foxglove,
        declare_use_webserver,
        declare_rosbridge_port,
        declare_foxglove_port,
        declare_webserver_port,
        OpaqueFunction(function=launch_connections)
    ])
