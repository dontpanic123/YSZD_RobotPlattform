# Building the ROS2 Robot Bridge Package

## Prerequisites

1. **ROS2 Installation**: Make sure you have ROS2 installed (tested with Humble, Foxy, or later)
   ```bash
   # Check if ROS2 is installed
   echo $ROS_DISTRO
   ```

2. **Source ROS2**: Source your ROS2 installation
   ```bash
   source /opt/ros/<your-ros2-distro>/setup.bash
   # For example:
   # source /opt/ros/humble/setup.bash
   ```

3. **Install colcon** (if not already installed):
   ```bash
   sudo apt install python3-colcon-common-extensions
   ```

## Build Steps

### Option 1: Build only this package (Recommended)

```bash
cd /home/y1234/Documents/robot_ws/robot-demos-main
source /opt/ros/<your-ros2-distro>/setup.bash
colcon build --packages-select ros2_robot_bridge
source install/setup.bash
```

### Option 2: Build all packages in workspace

```bash
cd /home/y1234/Documents/robot_ws/robot-demos-main
source /opt/ros/<your-ros2-distro>/setup.bash
colcon build
source install/setup.bash
```

### Option 3: Build with verbose output (for debugging)

```bash
cd /home/y1234/Documents/robot_ws/robot-demos-main
source /opt/ros/<your-ros2-distro>/setup.bash
colcon build --packages-select ros2_robot_bridge --cmake-args -DCMAKE_VERBOSE_MAKEFILE=ON
source install/setup.bash
```

## Verify Build

After building, verify the executable was created:

```bash
ls install/ros2_robot_bridge/lib/ros2_robot_bridge/
# Should show: robot_websocket_bridge
```

## Run the Node

```bash
# Make sure you've sourced the install space
source install/setup.bash

# Run with default parameters
ros2 run ros2_robot_bridge robot_websocket_bridge

# Or with custom WebSocket URL
ros2 run ros2_robot_bridge robot_websocket_bridge \
  --ros-args -p websocket_url:="ws://10.42.0.253:8439"
```

## Troubleshooting

### Error: "Could not find a package configuration file provided by 'ament_cmake'"
- Make sure you've sourced ROS2: `source /opt/ros/<distro>/setup.bash`

### Error: "Cannot find mongoose.h" or protobuf headers
- Make sure the `c/` directory exists with the generated protobuf files
- The CMakeLists.txt expects the structure:
  ```
  robot-demos-main/
    ├── c/
    │   ├── mongoose.c, mongoose.h
    │   ├── pb_*.c, pb_*.h
    │   └── generated/
    │       ├── inc/ (protobuf headers)
    │       └── src/ (protobuf sources)
    └── ros2_robot_bridge/
  ```

### Error: Compilation errors with C/C++ mixing
- The code uses C libraries (mongoose, nanopb) with C++ code
- Make sure `extern "C"` blocks are properly used (already included in the code)

### Clean Build
If you need to clean and rebuild:
```bash
rm -rf build/ install/ log/
colcon build --packages-select ros2_robot_bridge
```









