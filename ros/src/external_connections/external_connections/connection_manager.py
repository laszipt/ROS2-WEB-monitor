#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
import subprocess
import threading
import os
import socket
import netifaces

class ConnectionManager(Node):
    def __init__(self):
        super().__init__('connection_manager')
        
        # Status publishers
        self.rosbridge_status_pub = self.create_publisher(Bool, '/external_connections/rosbridge_status', 10)
        self.foxglove_status_pub = self.create_publisher(Bool, '/external_connections/foxglove_status', 10)
        self.webserver_status_pub = self.create_publisher(Bool, '/external_connections/webserver_status', 10)
        self.connection_info_pub = self.create_publisher(String, '/external_connections/info', 10)
        
        # Parameters
        self.declare_parameter('rosbridge_port', 9090)
        self.declare_parameter('foxglove_port', 8765)
        self.declare_parameter('webserver_port', 8080)
        
        # Timer for health checks
        self.timer = self.create_timer(5.0, self.check_connections)
        
        self.get_logger().info('Connection manager started')
    
    def check_connections(self):
        # Check if each service is running and publish status
        rosbridge_running = self.check_port_in_use(self.get_parameter('rosbridge_port').value)
        foxglove_running = self.check_port_in_use(self.get_parameter('foxglove_port').value)
        webserver_running = self.check_port_in_use(self.get_parameter('webserver_port').value)
        
        self.rosbridge_status_pub.publish(Bool(data=rosbridge_running))
        self.foxglove_status_pub.publish(Bool(data=foxglove_running))
        self.webserver_status_pub.publish(Bool(data=webserver_running))
        
        # Publish connection information
        info = self.get_connection_info(
            rosbridge_running, 
            foxglove_running, 
            webserver_running
        )
        self.connection_info_pub.publish(String(data=info))
    
    def check_port_in_use(self, port):
        """Check if a port is in use by trying to bind to it"""
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            try:
                s.bind(('127.0.0.1', port))
                return False  # Port is free, service NOT running
            except socket.error:
                return True   # Port is in use, service likely running
    
    def get_connection_info(self, rosbridge_running, foxglove_running, webserver_running):
        """Gather connection information for display"""
        try:
            # Get IP address
            ip_addresses = []
            for iface in netifaces.interfaces():
                if iface != 'lo':  # Skip loopback
                    try:
                        addrs = netifaces.ifaddresses(iface).get(netifaces.AF_INET)
                        if addrs:
                            for addr in addrs:
                                ip_addresses.append(addr['addr'])
                    except:
                        pass
                        
            rosbridge_port = self.get_parameter('rosbridge_port').value
            foxglove_port = self.get_parameter('foxglove_port').value
            webserver_port = self.get_parameter('webserver_port').value
            
            info_lines = []
            info_lines.append("BORS External Connections Status:")
            info_lines.append(f"Host IP Addresses: {', '.join(ip_addresses)}")
            info_lines.append("-----------------------------")
            
            if rosbridge_running:
                info_lines.append(f"✅ ROS Bridge WebSocket: ws://<IP>:{rosbridge_port}")
            else:
                info_lines.append("❌ ROS Bridge WebSocket: Not detected")
                
            if foxglove_running:
                info_lines.append(f"✅ Foxglove Bridge: ws://<IP>:{foxglove_port}")
            else:
                info_lines.append("❌ Foxglove Bridge: Not detected")
                
            if webserver_running:
                info_lines.append(f"✅ Web Dashboard: http://<IP>:{webserver_port}")
            else:
                info_lines.append("❌ Web Dashboard: Not detected")
                
            return "\n".join(info_lines)
        except Exception as e:
            self.get_logger().error(f"Error getting connection info: {str(e)}")
            return f"Error gathering connection information: {str(e)}"

def main(args=None):
    rclpy.init(args=args)
    connection_manager = ConnectionManager()
    
    try:
        rclpy.spin(connection_manager)
    except KeyboardInterrupt:
        pass
    finally:
        connection_manager.destroy_node()
        # destruction cleans up; avoid calling shutdown twice

if __name__ == "__main__":
    main()
