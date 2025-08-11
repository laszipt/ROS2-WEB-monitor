<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">

</head>
<body>

<h1>🌐 ROS2-WEB-monitor</h1>

<p>A comprehensive web-based monitoring and control interface for ROS2 systems, providing real-time visualization and interaction with ROS2 nodes, topics, parameters, and logs through a simple Python internal web server.</p>

<h2>🚀 Features</h2>

<ul>
    <li><strong>Real-time Monitoring</strong>: Live visualization of ROS2 nodes, topics, parameters, and system logs</li>
    <li><strong>Web-based Interface</strong>: Access your ROS2 system from any web browser without additional software installation</li>
    <li><strong>Interactive Control</strong>: Modify parameters and interact with ROS2 services directly from the web interface</li>
    <li><strong>Lightweight Architecture</strong>: Built with Python's internal web server for minimal system overhead</li>
    <li><strong>External Connections</strong>: Integrated support for external system connections and bridge communications</li>
    <li><strong>Responsive Design</strong>: Modern web interface optimized for both desktop and mobile viewing</li>
</ul>


<h2>🎥 Demo</h2>

<video width="100%" controls>
    <source src="https://github.com/laszipt/ROS2-WEB-monitor/raw/main/Screencast%20from%202025-08-08%2009%3A43%3A21.webm" type="video/webm">
    <p>Your browser doesn't support HTML video. <a href="https://github.com/laszipt/ROS2-WEB-monitor/blob/main/Screencast%20from%202025-08-08%2009%3A43%3A21.webm">Download the video</a> instead.</p>
</video>

<p><em>Screencast demonstration of ROS2-WEB-monitor interface showing real-time monitoring capabilities</em></p>


<h2>📋 Prerequisites</h2>

<ul>
    <li><strong>ROS2</strong>: Humble, Iron, or Rolling distribution</li>
    <li><strong>Python 3.8+</strong></li>
    <li><strong>Web Browser</strong>: Modern browser with WebSocket support</li>
    <li><strong>Network</strong>: Local network access to the ROS2 system</li>
</ul>

<h2>🛠️ Installation</h2>

<h3>Clone the Repository</h3>

<pre><code>git clone https://github.com/laszipt/ROS2-WEB-monitor.git
cd ROS2-WEB-monitor</code></pre>

<h3>Install Dependencies</h3>

<pre><code># Install ROS2 dependencies
rosdep install -i --from-paths . --rosdistro $ROS_DISTRO -y

# Install Python requirements
pip3 install -r requirements.txt</code></pre>

<h3>Build the Package</h3>

<pre><code># Source your ROS2 installation
source /opt/ros/$ROS_DISTRO/setup.bash

# Build the package
colcon build --packages-select ros2_web_monitor
source install/setup.bash</code></pre>

<h2>🚀 Quick Start</h2>

<h3>1. Launch the Web Monitor</h3>

<pre><code># Basic launch
ros2 launch ros2_web_monitor web_monitor.launch.py

# With custom configuration
ros2 launch ros2_web_monitor web_monitor.launch.py \
    port:=8080 \
    host:=0.0.0.0 \
    config_file:=config/monitor_config.yaml</code></pre>

<h3>2. Access the Web Interface</h3>

<p>Open your web browser and navigate to:</p>
<pre><code>http://localhost:8080</code></pre>

<h3>3. Connect External Systems</h3>

<p>The monitor supports external connections through various bridge protocols:</p>

<pre><code># Launch with Foxglove bridge support
ros2 launch ros2_web_monitor foxglove_bridge.launch.py

# Launch with rosbridge WebSocket support
ros2 launch ros2_web_monitor rosbridge_websocket.launch.py</code></pre>

<h2>📖 Usage</h2>

<h3>Web Dashboard Features</h3>

<ul>
    <li><strong>Node Monitor</strong>: View active ROS2 nodes and their status</li>
    <li><strong>Topic Viewer</strong>: Monitor topic data rates, message types, and live data</li>
    <li><strong>Parameter Manager</strong>: View and modify ROS2 parameters in real-time</li>
    <li><strong>Log Viewer</strong>: Stream and filter ROS2 system logs</li>
    <li><strong>Service Interface</strong>: Call ROS2 services directly from the web interface</li>
</ul>

<h3>Configuration</h3>

<p>Create a custom configuration file:</p>

<pre><code># config/monitor_config.yaml
server:
  host: "0.0.0.0"
  port: 8080
  debug: false

monitoring:
  update_rate: 10.0  # Hz
  max_log_entries: 1000
  
topics:
  blacklist:
    - "/rosout"
    - "/parameter_events"
  
external_connections:
  foxglove:
    enabled: true
    port: 8765
  rosbridge:
    enabled: true 
    port: 9090</code></pre>

<h3>API Endpoints</h3>

<p>The web server provides REST API endpoints for programmatic access:</p>

<pre><code># Get all active nodes
curl http://localhost:8080/api/nodes

# Get topic information
curl http://localhost:8080/api/topics

# Get parameter values
curl http://localhost:8080/api/parameters

# Get system logs
curl http://localhost:8080/api/logs?limit=100</code></pre>

<h2>🏗️ Architecture</h2>

<pre><code>┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   Web Browser   │◄──►│   Web Server     │◄──►│   ROS2 System   │
│                 │    │  (Python)        │    │                 │
│  - Dashboard    │    │  - REST API      │    │  - Nodes        │
│  - Controls     │    │  - WebSockets    │    │  - Topics       │
│  - Monitoring   │    │  - Static Files  │    │  - Parameters   │
└─────────────────┘    └──────────────────┘    └─────────────────┘
                              │
                              ▼
                       ┌──────────────────┐
                       │ External Bridges │
                       │  - Foxglove      │
                       │  - rosbridge     │
                       │  - Custom        │
                       └──────────────────┘</code></pre>

<h2>🔧 Development</h2>

<h3>Running in Development Mode</h3>

<pre><code># Enable debug mode
export ROS2_WEB_MONITOR_DEBUG=true

# Run with hot reload
python3 -m ros2_web_monitor.server --dev</code></pre>

<h3>Adding Custom Widgets</h3>

<ol>
    <li>Create a new widget in <code>web/widgets/</code></li>
    <li>Register it in <code>web/js/widget_manager.js</code></li>
    <li>Add styling in <code>web/css/widgets.css</code></li>
</ol>

<h3>Contributing</h3>

<ol>
    <li>Fork the repository</li>
    <li>Create a feature branch</li>
    <li>Make your changes</li>
    <li>Add tests</li>
    <li>Submit a pull request</li>
</ol>

<h2>📁 Project Structure</h2>

<pre><code>ros2_web_monitor/
├── launch/                 # Launch files
│   ├── web_monitor.launch.py
│   ├── foxglove_bridge.launch.py
│   └── rosbridge_websocket.launch.py
├── ros2_web_monitor/       # Python package
│   ├── server.py          # Main web server
│   ├── ros_interface.py   # ROS2 communication
│   └── external_connections/  # Bridge modules
├── web/                   # Web interface
│   ├── index.html
│   ├── css/
│   ├── js/
│   └── widgets/
├── config/                # Configuration files
└── tests/                 # Unit tests</code></pre>

<h2>🤝 Contributing</h2>

<p>We welcome contributions! Please see our <a href="CONTRIBUTING.md">Contributing Guidelines</a> for details.</p>

<h3>Development Setup</h3>

<pre><code># Install development dependencies
pip3 install -r requirements-dev.txt

# Run tests
python3 -m pytest tests/

# Code formatting
black ros2_web_monitor/
flake8 ros2_web_monitor/</code></pre>

<h2>📄 License</h2>

<p>This project is licensed under the MIT License - see the <a href="LICENSE">LICENSE</a> file for details.</p>

<h2>🐛 Troubleshooting</h2>

<h3>Common Issues</h3>

<p><strong>Web interface not loading:</strong></p>
<ul>
    <li>Check if the port is already in use</li>
    <li>Verify firewall settings</li>
    <li>Ensure ROS2 is properly sourced</li>
</ul>

<p><strong>No data in dashboard:</strong></p>
<ul>
    <li>Verify ROS2 nodes are running</li>
    <li>Check network connectivity</li>
    <li>Review browser console for errors</li>
</ul>

<p><strong>External connections failing:</strong></p>
<ul>
    <li>Ensure bridge services are running</li>
    <li>Check port configurations</li>
    <li>Verify WebSocket support in browser</li>
</ul>

<h3>Support</h3>

<ul>
    <li><strong>Issues</strong>: <a href="https://github.com/laszipt/ROS2-WEB-monitor/issues">GitHub Issues</a></li>
    <li><strong>Discussions</strong>: <a href="https://github.com/laszipt/ROS2-WEB-monitor/discussions">GitHub Discussions</a></li>
    <li><strong>Documentation</strong>: <a href="https://github.com/laszipt/ROS2-WEB-monitor/wiki">Wiki</a></li>
</ul>

<h2>🙏 Acknowledgments</h2>

<ul>
    <li><a href="https://ros.org/">ROS2</a> - Robot Operating System</li>
    <li><a href="https://github.com/RobotWebTools/rosbridge_suite">rosbridge_suite</a> - WebSocket bridge</li>
    <li><a href="https://foxglove.dev/">Foxglove Studio</a> - Robotics visualization</li>
</ul>

<hr>

<p align="center">⭐ <strong>Star this repository if you find it useful!</strong></p>

<hr>

<p><em>Note: This README was generated based on the project description and common patterns for ROS2 web monitoring tools. Please verify and update the content according to your actual implementation.</em></p>

</body>
</html>
