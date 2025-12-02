# ROS2 Robot WebSocket Bridge

This ROS2 package provides a bridge node that connects ROS2 `/cmd_vel` commands to a robot's WebSocket API.

## Features

- Subscribes to `/cmd_vel` (geometry_msgs/Twist) and sends commands to robot via WebSocket
- Publishes odometry feedback from robot to `/odom` (nav_msgs/Odometry)
- Handles robot initialization and deinitialization automatically
- Configurable WebSocket URL, command rate, and report frequency

## Building

1. Make sure you have ROS2 installed and sourced:
   ```bash
   source /opt/ros/<ros2-distro>/setup.bash
   ```

2. Build the package:
   ```bash
   cd /path/to/robot-demos-main
   colcon build --packages-select ros2_robot_bridge
   source install/setup.bash
   ```

## Usage

### Basic Usage

Run the node with default parameters:
```bash
ros2 run ros2_robot_bridge robot_websocket_bridge
```

### With Custom Parameters

```bash
ros2 run ros2_robot_bridge robot_websocket_bridge \
  --ros-args \
  -p websocket_url:="ws://192.168.1.100:8439" \
  -p command_rate:=50.0 \
  -p report_frequency:=50
```

### Parameters

- `websocket_url` (string, default: "ws://localhost:8439"): WebSocket URL of the robot
- `command_rate` (double, default: 50.0): Rate in Hz at which commands are sent to the robot
- `report_frequency` (int, default: 50): Report frequency in Hz (options: 1, 50, 100, 250, 500, 1000)

### Topics

#### Subscribed Topics
- `/cmd_vel` (geometry_msgs/msg/Twist): Velocity commands
  - `linear.x` → `speed_x` (m/s) - forward/backward
  - `linear.y` → `speed_y` (m/s) - left/right
  - `angular.z` → `speed_z` (rad/s) - rotation

#### Published Topics
- `/odom` (nav_msgs/msg/Odometry): Odometry feedback from the robot

## Example

1. Start the bridge node:
   ```bash
   ros2 run ros2_robot_bridge robot_websocket_bridge \
     --ros-args -p websocket_url:="ws://10.42.0.253:8439"
   ```

2. Send velocity commands (in another terminal):
   ```bash
   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
     "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}"
   ```

3. View odometry feedback:
   ```bash
   ros2 topic echo /odom
   ```

## Implementation Details

- Uses mongoose WebSocket library for async WebSocket communication
- Uses nanopb (Protocol Buffers) for message encoding/decoding
- Reuses generated protobuf files from the `c/generated/` directory
- Commands are sent at the specified rate (default 50Hz)
- If no `/cmd_vel` messages are received for 1 second, zero velocity is sent
- Robot is automatically initialized on connection and deinitialized on shutdown









