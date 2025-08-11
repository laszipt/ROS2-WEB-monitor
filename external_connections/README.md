# External Connections Package

This package provides a unified interface for managing all external connections for the BORS robot system.

## Features

- **ROS Bridge WebSocket Server**: Allows web clients to connect to ROS2 topics
- **Foxglove Bridge**: Enables compatibility with Foxglove Studio visualization tool
- **Web Dashboard Server**: Serves the BORS monitoring dashboard web interface
- **Connection Manager**: Monitors and reports the status of all external connections

## Usage

### Launch all services

```bash
ros2 launch external_connections external_connections.launch.py
```

### Launch with specific options

```bash
# Disable Foxglove bridge
ros2 launch external_connections external_connections.launch.py use_foxglove:=false

# Change ports
ros2 launch external_connections external_connections.launch.py rosbridge_port:=9091 webserver_port:=8081
```

### Available Parameters

- `use_rosbridge`: Enable ROS Bridge (true/false)
- `use_foxglove`: Enable Foxglove Bridge (true/false)
- `use_webserver`: Enable Web Dashboard Server (true/false)
- `rosbridge_port`: Port for ROS Bridge WebSocket (default: 9090)
- `foxglove_port`: Port for Foxglove Bridge (default: 8765)
- `webserver_port`: Port for Web Dashboard (default: 8080)

## Connection Information

The Connection Manager publishes status and connection details to the following topics:

- `/external_connections/rosbridge_status`
- `/external_connections/foxglove_status`
- `/external_connections/webserver_status`
- `/external_connections/info`

## Web Dashboard

Access the web dashboard by opening a browser to:

```
http://<robot-ip>:8080
