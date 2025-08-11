from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import socket

def _is_port_open(port: int, host: str = '127.0.0.1', timeout: float = 0.5) -> bool:
    try:
        with socket.create_connection((host, int(port)), timeout=timeout):
            return True
    except Exception:
        return False

def generate_launch_description():
    # Parameters
    port = LaunchConfiguration('port', default='8765')
    
    # Launch arguments
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='8765',
        description='Port for Foxglove WebSocket server'
    )
    
    # Foxglove bridge node
    foxglove_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        parameters=[{
            'port': port
        }],
        output='screen'
    )
    
    def launch_if_needed(context):
        selected_port = context.perform_substitution(port)
        if _is_port_open(selected_port):
            return [LogInfo(msg=[f"Foxglove Bridge already running on port {selected_port}. Skipping launch."])]
        return [foxglove_node]
    
    return LaunchDescription([
        port_arg,
        OpaqueFunction(function=launch_if_needed)
    ])
