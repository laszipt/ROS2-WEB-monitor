#!/usr/bin/env python3

import os
import http.server
import socketserver
import threading
import asyncio
import websockets
import json
import time
import os
import re
import urllib.parse
import traceback  # Added for detailed tracing
from itertools import islice  # For efficient limited file reading
from rclpy.node import Node
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from diagnostic_updater import Updater
from urllib.parse import urlparse, parse_qs
import socket  # For rosbridge reachability test

# Custom TCP Server that allows address reuse
from http.server import ThreadingHTTPServer

# Replace your ReuseAddressTCPServer class with this:
class ReuseAddressThreadingHTTPServer(ThreadingHTTPServer):
    allow_reuse_address = True
    
    def process_request(self, request, client_address):
        """Override to log incoming connection before processing"""
 #       print(f"INCOMING CONNECTION: New connection from {client_address[0]}:{client_address[1]}")
        return super().process_request(request, client_address)


# ROS Node for WebSocket integration
class WebServerNode(Node):  
    def __init__(self):
        super().__init__('web_server')
        # Declare parameters
        self.declare_parameter('port', 8080)
        self.declare_parameter('directory', '/home/tamas/BORS1-ROS2/web_monitor')
        
        # Get parameters
        port = self.get_parameter('port').get_parameter_value().integer_value
        directory = self.get_parameter('directory').get_parameter_value().string_value
        
        # Validate the specified directory to serve files from there (avoid changing global CWD)
        if os.path.isdir(directory):
            self.get_logger().info(f"Serving files from {directory} on port {port}")
        else:
            self.get_logger().error(f"Directory {directory} does not exist")
            return
        
        # Create custom handler with reference to this node and target directory (no os.chdir)
        def create_handler(node, serve_directory):
            class NodeAwareHTTPRequestHandler(http.server.SimpleHTTPRequestHandler):
                protocol_version = 'HTTP/1.1'  # This is crucial!
                def __init__(self, *args, **kwargs):
                    # Ensure SimpleHTTPRequestHandler serves from serve_directory without changing process CWD
                    super().__init__(*args, directory=serve_directory, **kwargs)
                def setup(self):
                    """Called when a request is first received, before it's processed"""
                    super().setup()
                    # This will log the raw request as soon as it's received, before any processing
                def end_headers(self):
                    # Add headers to prevent caching
                    self.send_header('Cache-Control', 'no-store, no-cache, must-revalidate, max-age=0')
                    self.send_header('Pragma', 'no-cache')
                    self.send_header('Expires', '0')
                    # Add CORS headers to allow access from any origin (helpful for testing)
                    self.send_header('Access-Control-Allow-Origin', '*')
                    # Add connection management headers
                    self.send_header('Connection', 'close')
                    #self.send_header('Keep-Alive', 'timeout=5, max=100')
                    super().end_headers()
                
                def guess_type(self, path):
                    # Make sure the MIME types are correct for our files
                    mimetype = super().guess_type(path)
                    if path.endswith('.js'):
                        return 'application/javascript'
                    elif path.endswith('.css'):
                        return 'text/css'
                    return mimetype

                def do_GET(self):
                    try:
                        # Parse URL path
                        parsed_path = urlparse(self.path)
                        path = parsed_path.path
                        
                        # Handle log node list requests
                        if path.startswith('/api/log-nodes'):
                            # Support optional pid query to fetch a specific log file's content
                            parsed = urlparse(self.path)
                            pid_param = parse_qs(parsed.query).get('pid', [''])[0]
                            if pid_param:
                                self.handle_log_nodes_pid_request(pid_param)
                            else:
                                self.handle_log_nodes_request()
                            return
                        
                        # Handle node log content requests
                        if path.startswith('/api/node-log-content'):
                            self.handle_node_log_content_request()
                            return
                        
                        # Handle param files list requests
                        if path.startswith('/api/param-files'):
                            self.handle_param_files_request()
                            return
                        
                        # Handle param file content request
                        if path.startswith('/api/param-file'):
                            self.handle_param_file_content_request()
                            return
                        
                        # Track request for diagnostics
                        node.request_count += 1
                        node.last_request_time = time.time()
                        
                        # Call the parent method
                        super().do_GET()
                        
                    except (BrokenPipeError, ConnectionResetError):
                        # Handle client disconnection gracefully
                        self.close_connection = True
                    except Exception:
                        # Log any other unexpected errors with traceback
                        traceback.print_exc()
                        self.close_connection = True  # Add this line
                        raise
                    
                def handle_log_nodes_request(self):
                    try:
                        # Read the launch.log file
                        ros_log_dir = os.path.join(os.path.expanduser('~'), '.ros', 'log')
                        latest_dir = os.path.join(ros_log_dir, 'latest')
                        
                        if not os.path.exists(latest_dir):
                            self.send_error(404, "ROS log directory not found")
                            return
                        
                        launch_log_path = os.path.join(latest_dir, 'launch.log')
                        
                        if not os.path.exists(launch_log_path):
                            self.send_error(404, "Launch log file not found")
                            return
                        
                        # Read the whole file, but limit to 100 found nodes
                        started_nodes = []
                        with open(launch_log_path, 'r') as f:
                            for line in f:
                                if ": process started with pid [" in line:
                                    pattern = r'\[INFO\] \[([^]]+)\]: process started with pid \[(\d+)\]'
                                    match = re.search(pattern, line)
                                    if match:
                                        node_name = match.group(1)
                                        pid = int(match.group(2))
                                        timestamp = line.split('[INFO]')[0].strip()
                                        is_alive = False
                                        try:
                                            os.kill(pid, 0)
                                            is_alive = True
                                        except OSError:
                                            is_alive = False
                                        # Find the log file for this PID
                                        log_file_path = self.find_log_file_by_pid(ros_log_dir, pid)
                                        started_nodes.append({
                                            'node_name': node_name,
                                            'pid': pid,
                                            'timestamp': timestamp,
                                            'is_alive': is_alive,
                                            'log_file': log_file_path
                                        })
                                        if len(started_nodes) >= 100:
                                            break
                        
                        response_bytes = json.dumps(started_nodes).encode()
                        self.send_response(200)
                        self.send_header('Content-Type', 'application/json')
                        self.send_header('Content-Length', str(len(response_bytes)))
                        self.send_header('Connection', 'close')
                        self.end_headers()
                        self.wfile.write(response_bytes)
                        self.wfile.flush()
                        # Mark connection closed once response is flushed
                        self.close_connection = True
        
                        
                    except Exception as e:
                        print(f"Error in handle_log_nodes_request: {str(e)}")
                        error_bytes = json.dumps({'error': str(e)}).encode()
                        self.send_response(500)
                        self.send_header('Content-Type', 'application/json')
                        self.send_header('Content-Length', str(len(error_bytes)))
                        self.send_header('Connection', 'close')
                        self.end_headers()
                        self.wfile.write(error_bytes)
                        self.wfile.flush()
                        self.close_connection = True
                
                def find_log_file_by_pid(self, ros_log_dir, pid):
                    """Find the log file for a specific PID in the ROS log directory."""
                    # Search through all subdirectories in the .ros/log folder (not just 'latest')
                    for root, dirs, files in os.walk(ros_log_dir):
                        for file in files:
                            if file.endswith('.log'):
                                file_path = os.path.join(root, file)
                                try:
                                    # Check if this log file contains entries for this PID
                                    with open(file_path, 'r') as f:
                                        # Read first few lines to check for PID
                                        for _ in range(10):  # Check first 10 lines
                                            line = f.readline()
                                            if not line:
                                                break
                                            # Look for PID in the log file
                                            if f"[{pid}]" in line:
                                                return file_path
                                except Exception:
                                    # Skip files we can't read
                                    pass
                    return None  # No log file found for this PID

                def handle_node_log_content_request(self):
                    """Handle request for a specific node's log file content."""
                    # Parse query parameters
                    parsed_url = urlparse(self.path)
                    query_params = parse_qs(parsed_url.query)
                    
                    log_file_path = query_params.get('path', [''])[0]
                    
                    if not log_file_path:
                        error_bytes = json.dumps({'error': 'No log file path provided'}).encode()
                        self.send_response(400)
                        self.send_header('Content-Type', 'application/json')
                        self.send_header('Content-Length', str(len(error_bytes)))
                        self.send_header('Connection', 'close')
                        self.end_headers()
                        self.wfile.write(error_bytes)
                        self.wfile.flush()
                        self.close_connection = True
                        return
                    
                    try:
                        # Read only the first 100 lines to keep the payload small
                        max_lines = 100
                        lines = []
                        with open(log_file_path, 'r', errors='replace') as f:
                            for i, line in enumerate(f):
                                if i >= max_lines:
                                    break
                                lines.append(line)
                        content = ''.join(lines)
                        # Indicate truncation if file is longer
                        try:
                            with open(log_file_path, 'r') as f_size_check:
                                for _ in range(max_lines):
                                    next(f_size_check, None)
                                if f_size_check.readline():
                                    content += "\n...[truncated to first 100 lines]"
                        except Exception:
                            pass
                        
                        response = {
                            'log_file': log_file_path,
                            'content': content
                        }
                        
                        response_bytes = json.dumps(response).encode()
                        self.send_response(200)
                        self.send_header('Content-Type', 'application/json')
                        self.send_header('Content-Length', str(len(response_bytes)))
                        self.send_header('Connection', 'close')
                        self.end_headers()
                        self.wfile.write(response_bytes)
                        self.wfile.flush()
                        self.close_connection = True
                    except Exception as e:
                        error_bytes = json.dumps({'error': str(e), 'log_file': log_file_path}).encode()
                        self.send_response(500)
                        self.send_header('Content-Type', 'application/json')
                        self.send_header('Content-Length', str(len(error_bytes)))
                        self.send_header('Connection', 'close')
                        self.end_headers()
                        self.wfile.write(error_bytes)
                        self.wfile.flush()
                        self.close_connection = True
                
                def handle_log_nodes_pid_request(self, pid_str):
                    """Return log file content for a given PID if a matching _PID_ pattern is found in any *.log file name."""
                    try:
                        pid = int(pid_str)
                    except ValueError:
                        self.send_error(400, "Invalid PID format")
                        return

                    ros_log_dir = os.path.join(os.path.expanduser('~'), '.ros', 'log')

                    matched_path = None
                    for root, dirs, files in os.walk(ros_log_dir):
                        for fname in files:
                            if fname.endswith('.log') and f"_{pid}_" in fname:
                                matched_path = os.path.join(root, fname)
                                break
                        if matched_path:
                            break

                    if not matched_path or not os.path.exists(matched_path):
                        self.send_error(404, "Log file for given PID not found")
                        return

                    # Stream only first 100 lines
                    try:
                        with open(matched_path, 'r') as f:
                            content_lines = list(islice(f, 100))
                        if sum(1 for _ in open(matched_path, 'r')) > 100:
                            content_lines.append('\n...[truncated to first 100 lines]')
                        content = ''.join(content_lines)
                    except Exception as e:
                        self.send_error(500, f"Error reading log file: {e}")
                        return

                    response_bytes = json.dumps({'pid': pid, 'content': content}).encode()
                    self.send_response(200)
                    self.send_header('Content-Type', 'application/json')
                    self.send_header('Content-Length', str(len(response_bytes)))
                    self.send_header('Connection', 'close')
                    self.end_headers()
                    self.wfile.write(response_bytes)
                    self.wfile.flush()
                    self.close_connection = True
                
                def handle_param_files_request(self):
                    try:
                        # Determine workspace root (one level above the web root)
                        workspace_root = os.path.abspath(os.path.join(os.getcwd(), '..'))
                        param_root = os.path.join(workspace_root, 'ros', 'src')
                        yaml_files = []
                        for root, _, files in os.walk(param_root):
                            for fname in files:
                                if fname.endswith(('.yaml', '.yml')):
                                    full_path = os.path.join(root, fname)
                                    rel_path = os.path.relpath(full_path, param_root)
                                    yaml_files.append(rel_path)

                        response_bytes = json.dumps({'files': yaml_files}).encode()
                        self.send_response(200)
                        self.send_header('Content-Type', 'application/json')
                        self.send_header('Content-Length', str(len(response_bytes)))
                        self.send_header('Connection', 'close')
                        self.end_headers()
                        self.wfile.write(response_bytes)
                        self.wfile.flush()
                        self.close_connection = True
                    except Exception as e:
                        error_bytes = json.dumps({'error': str(e)}).encode()
                        self.send_response(500)
                        self.send_header('Content-Type', 'application/json')
                        self.send_header('Content-Length', str(len(error_bytes)))
                        self.send_header('Connection', 'close')
                        self.end_headers()
                        self.wfile.write(error_bytes)
                        self.wfile.flush()
                        self.close_connection = True
                
                def handle_param_file_content_request(self):
                    """Return raw content of a YAML parameter file identified by path query param."""
                    parsed_url = urlparse(self.path)
                    query_params = parse_qs(parsed_url.query)
                    rel_path = query_params.get('path', [''])[0]
                    if not rel_path:
                        self.send_error(400, "No path specified")
                        return
                    try:
                        workspace_root = os.path.abspath(os.path.join(os.getcwd(), '..'))
                        param_root = os.path.join(workspace_root, 'ros', 'src')
                        # Normalize and ensure file is within param_root
                        abs_path = os.path.normpath(os.path.join(param_root, rel_path))
                        if not abs_path.startswith(param_root):
                            self.send_error(403, "Access denied")
                            return
                        if not os.path.isfile(abs_path):
                            self.send_error(404, "File not found")
                            return
                        with open(abs_path, 'r') as f:
                            content = f.read()
                        content_bytes = content.encode()
                        self.send_response(200)
                        self.send_header('Content-Type', 'text/plain')
                        self.send_header('Content-Length', str(len(content_bytes)))
                        self.send_header('Connection', 'close')
                        self.end_headers()
                        self.wfile.write(content_bytes)
                        self.wfile.flush()
                        self.close_connection = True
                    except Exception as e:
                        error_bytes = json.dumps({'error': str(e)}).encode()
                        self.send_response(500)
                        self.send_header('Content-Type', 'application/json')
                        self.send_header('Content-Length', str(len(error_bytes)))
                        self.send_header('Connection', 'close')
                        self.end_headers()
                        self.wfile.write(error_bytes)
                        self.wfile.flush()
                        self.close_connection = True
                
                def handle_param_file_save_request(self):
                    """Save the posted content to the given YAML file (path query)."""
                    content_length = int(self.headers.get('Content-Length', 0))
                    body = self.rfile.read(content_length).decode()
                    parsed_url = urlparse(self.path)
                    rel_path = parse_qs(parsed_url.query).get('path', [''])[0]
                    if not rel_path:
                        self.send_error(400, 'No path specified')
                        return
                    try:
                        workspace_root = os.path.abspath(os.path.join(os.getcwd(), '..'))
                        param_root = os.path.join(workspace_root, 'ros', 'src')
                        abs_path = os.path.normpath(os.path.join(param_root, rel_path))
                        if not abs_path.startswith(param_root):
                            self.send_error(403, 'Access denied')
                            return
                        # Ensure directory exists
                        if not os.path.isfile(abs_path):
                            self.send_error(404, 'File not found')
                            return
                        with open(abs_path, 'w') as f:
                            f.write(body)
                        resp = json.dumps({'status':'ok'}).encode()
                        self.send_response(200)
                        self.send_header('Content-Type','application/json')
                        self.send_header('Content-Length',str(len(resp)))
                        self.send_header('Connection','close')
                        self.end_headers()
                        self.wfile.write(resp)
                        self.wfile.flush()
                        self.close_connection=True
                    except Exception as e:
                        err=json.dumps({'error':str(e)}).encode()
                        self.send_response(500)
                        self.send_header('Content-Type','application/json')
                        self.send_header('Content-Length',str(len(err)))
                        self.send_header('Connection','close')
                        self.end_headers()
                        self.wfile.write(err)
                        self.wfile.flush()
                        self.close_connection=True
                
                def handle_kill_node_request(self):
                    try:
                        # Parse query parameters
                        parsed = urlparse(self.path)
                        query_params = parse_qs(parsed.query)
                        
                        # Get node name and PID from query parameters
                        node_name = query_params.get('nodeName', [''])[0]
                        pid_str = query_params.get('pid', [''])[0]
                        
                        if not pid_str:
                            self.send_response(400)
                            self.send_header('Content-Type', 'application/json')
                            self.end_headers()
                            self.wfile.write(json.dumps({'success': False, 'error': 'Missing PID parameter'}).encode())
                            return
                        
                        try:
                            pid = int(pid_str)
                        except ValueError:
                            self.send_response(400)
                            self.send_header('Content-Type', 'application/json')
                            self.end_headers()
                            self.wfile.write(json.dumps({'success': False, 'error': 'Invalid PID format'}).encode())
                            return
                        
                        # Check if process exists
                        import os
                        import signal
                        try:
                            # Send signal 0 to check if process exists
                            os.kill(pid, 0)
                        except OSError:
                            self.send_response(404)
                            self.send_header('Content-Type', 'application/json')
                            self.end_headers()
                            self.wfile.write(json.dumps({'success': False, 'error': f'Process with PID {pid} not found'}).encode())
                            return
                        
                        # Kill the process
                        try:
                            os.kill(pid, signal.SIGTERM)
                            # Log the action
                            node.get_logger().info(f"Terminated node {node_name} with PID {pid}")
                            
                            self.send_response(200)
                            self.send_header('Content-Type', 'application/json')
                            self.end_headers()
                            self.wfile.write(json.dumps({'success': True, 'message': f'Node {node_name} (PID: {pid}) terminated'}).encode())
                        except Exception as e:
                            self.send_response(500)
                            self.send_header('Content-Type', 'application/json')
                            self.end_headers()
                            self.wfile.write(json.dumps({'success': False, 'error': f'Failed to kill process: {str(e)}'}).encode())
                    except Exception as e:
                        self.send_response(500)
                        self.send_header('Content-Type', 'application/json')
                        self.end_headers()
                        self.wfile.write(json.dumps({'success': False, 'error': f'Server error: {str(e)}'}).encode())
                
                def do_POST(self):
                    try:
                        parsed_path = urlparse(self.path)
                        path = parsed_path.path
                        if path.startswith('/api/param-file'):
                            self.handle_param_file_save_request()
                            return
                        elif path.startswith('/api/kill-node'):
                            self.handle_kill_node_request()
                            return
                        # Unknown POST
                        self.send_error(404, 'Not Found')
                    except Exception:
                        traceback.print_exc()
                        self.close_connection=True
                        raise
                
                def copyfile(self, source, outputfile):
                    """Override copyfile to handle broken pipes gracefully"""
                    try:
                        # Use a buffer size that's reasonable
                        buffer_size = 64 * 1024  # 64KB buffer
                        
                        while True:
                            buf = source.read(buffer_size)
                            if not buf:
                                break
                            outputfile.write(buf)
                        
                    except (BrokenPipeError, ConnectionResetError):
                        # Client disconnected while sending data - this is normal
                        pass
                    except Exception:
                        traceback.print_exc()
                        raise
                
                def log_error(self, format, *args):
                    """Override log_error to filter out common connection errors"""
                    # Don't log broken pipe errors as they're normal client disconnections
                    if "Broken pipe" in str(args) or "Connection reset" in str(args):
                        return
                    super().log_error(format, *args)
            
            return NodeAwareHTTPRequestHandler

        handler = create_handler(self, directory)
        
        # Start HTTP server in a separate thread
        self.httpd = ReuseAddressThreadingHTTPServer(("0.0.0.0", port), handler)
        self.server_thread = threading.Thread(target=self.httpd.serve_forever)
        self.server_thread.daemon = True
        self.server_thread.start()
        self.get_logger().info(f"HTTP Server started on port {port}")
        
        # Initialize diagnostic updater
        self.diagnostic_updater = Updater(self)
        self.diagnostic_updater.setHardwareID("none")
        
        # Add diagnostic tasks
        self.diagnostic_updater.add("HTTP Server Status", self.http_server_diagnostic)
        self.diagnostic_updater.add("WebSocket Status", self.websocket_diagnostic)
        
        # Initialize diagnostic variables
        self.http_server_active = True
        # rosbridge reachability flag (updated every diagnostic cycle)
        self.websocket_server_active = False
        self.last_request_time = None
        self.request_count = 0
        self.websocket_connections = 0  # Not used for external ping but kept for consistency
        self.start_time = time.time()
        
        # External rosbridge WebSocket endpoint to monitor
        self.websocket_test_host = "localhost"
        self.websocket_test_port = 9090
        
        # Create diagnostic timer
        self.diagnostic_timer = self.create_timer(1.0, self.diagnostic_updater.update)
        
        # Graceful shutdown is handled via overridden destroy_node(); no need for add_on_shutdown

    # ------------------------------------------------------------------
    # Convenience helpers for a future WebSocket server implementation
    # ------------------------------------------------------------------
    def _ws_started(self):
        """Mark the WebSocket listener as running."""
        self.websocket_server_active = True

    def _ws_stopped(self):
        """Mark the WebSocket listener as stopped."""
        self.websocket_server_active = False
        self.websocket_connections = 0

    def _ws_client_connected(self):
        """Increment connection counter when a client connects."""
        self.websocket_connections += 1

    def _ws_client_disconnected(self):
        """Decrement connection counter when a client disconnects."""
        if self.websocket_connections > 0:
            self.websocket_connections -= 1

    def websocket_diagnostic(self, stat):
        """Diagnostic task for WebSocket status"""
        # Try connecting to rosbridge_websocket (quick TCP connect, 0.5s timeout)
        stat.add("Host", self.websocket_test_host)
        stat.add("Port", str(self.websocket_test_port))
        reachable = False
        try:
            with socket.create_connection((self.websocket_test_host, self.websocket_test_port), timeout=0.5):
                reachable = True
        except (socket.timeout, ConnectionRefusedError, OSError):
            reachable = False

        if reachable:
            stat.summary(DiagnosticStatus.OK, "Rosbridge Websocket available")
        else:
            stat.summary(DiagnosticStatus.ERROR, "Rosbridge Websocket NOT available")
        return stat

    def http_server_diagnostic(self, stat):
        """Diagnostic task for HTTP server status"""
        if self.http_server_active:
            stat.summary(DiagnosticStatus.OK, "HTTP Server is running")
            stat.add("Port", str(self.get_parameter('port').get_parameter_value().integer_value))
            stat.add("Uptime", f"{time.time() - self.start_time:.2f}")
            stat.add("Request Count", str(self.request_count))
            
            if self.last_request_time:
                time_since_last = time.time() - self.last_request_time
                stat.add("Time Since Last Request", f"{time_since_last:.2f}s")
        else:
            stat.summary(DiagnosticStatus.ERROR, "HTTP Server is not running")
            stat.add("Port", str(self.get_parameter('port').get_parameter_value().integer_value))
        return stat

    def destroy_node(self):
        self._on_shutdown()
        return super().destroy_node()

    # ------------------------------------------------------------------
    # Graceful shutdown helpers
    # ------------------------------------------------------------------
    def _on_shutdown(self):
        """Ensure the background HTTP server is stopped before node shutdown."""
        try:
            if hasattr(self, 'httpd'):
                print("Shutting down HTTP server …")
                 # Stop serve_forever loop and close socket
                self.httpd.shutdown()
                self.httpd.server_close()
        except Exception as e:
            print(f"Error while shutting down HTTP server: {e}")

        # Wait for worker thread to exit
        try:
            if hasattr(self, 'server_thread') and self.server_thread.is_alive():
                self.server_thread.join(timeout=3)
        except Exception:
            pass

def main(args=None):
    import rclpy
    rclpy.init(args=args)
    web_server = WebServerNode()
    
    try:
        rclpy.spin(web_server)
    except KeyboardInterrupt:
        pass
    finally:
        web_server.destroy_node()
        # destroy_node() already performs cleanup; avoid calling shutdown twice

if __name__ == "__main__":
    main()
