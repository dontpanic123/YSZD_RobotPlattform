cmake_minimum_required(VERSION 3.8)
project(mecanum_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python scripts
install(PROGRAMS
  scripts/robot_node.py
  scripts/remote_control.py
  scripts/path_publisher.py
  scripts/camera_node_fixed.py
  scripts/realsense_camera_node.py
  scripts/apriltag_detector_enhanced.py
  scripts/apriltag_detector_opencv4.py
  scripts/position_calculator.py
  scripts/image_viewer.py
  scripts/waypoint_follower.py
  scripts/waypoint_recorder.py
  scripts/ros2_websocket_bridge.py
  scripts/ultrasonic_sensor_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install URDF files
install(DIRECTORY
  urdf
  DESTINATION share/${PROJECT_NAME}/
)

# Install RViz config files
install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Build C++ executables
add_executable(realsense_dual_camera_node src/realsense_dual_camera_node.cpp)
ament_target_dependencies(realsense_dual_camera_node
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
)

target_link_libraries(realsense_dual_camera_node
  ${OpenCV_LIBS}
  realsense2::realsense2
)

target_include_directories(realsense_dual_camera_node PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)

install(TARGETS
  realsense_dual_camera_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
