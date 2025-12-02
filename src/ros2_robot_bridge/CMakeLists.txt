cmake_minimum_required(VERSION 3.8)
project(ros2_robot_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Path to nanopb and generated protobuf files
# c目录现在位于src目录下（与ros2_robot_bridge同级）
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../c")
    # c目录在src目录下（推荐位置）
    set(C_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../c)
elseif(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../../c")
    # 如果c目录在工作空间根目录
    set(C_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../c)
else()
    message(FATAL_ERROR "Cannot find 'c' directory with mongoose and protobuf files. Expected location: ${CMAKE_CURRENT_SOURCE_DIR}/../c")
endif()

set(GENERATED_INC_DIR ${C_DIR}/generated/inc)
set(GENERATED_SRC_DIR ${C_DIR}/generated/src)

# Include directories
include_directories(
  ${C_DIR}
  ${GENERATED_INC_DIR}
)

# Add nanopb source files (C files)
set(NANOPB_SOURCES
  ${C_DIR}/pb_encode.c
  ${C_DIR}/pb_decode.c
  ${C_DIR}/pb_common.c
)

# Add generated protobuf source files
set(PROTOBUF_SOURCES
  ${GENERATED_SRC_DIR}/public_api_types.pb.c
  ${GENERATED_SRC_DIR}/public_api_up.pb.c
  ${GENERATED_SRC_DIR}/public_api_down.pb.c
)

# Add mongoose WebSocket library
set(MONGOOSE_SOURCES
  ${C_DIR}/mongoose.c
)

# Create executable
add_executable(robot_websocket_bridge
  src/robot_websocket_bridge.cpp
  ${NANOPB_SOURCES}
  ${PROTOBUF_SOURCES}
  ${MONGOOSE_SOURCES}
)

# Compiler definitions
target_compile_definitions(robot_websocket_bridge PRIVATE
  PB_ENABLE_MALLOC=1
  MG_IO_SIZE=65536
)

# Link libraries
ament_target_dependencies(robot_websocket_bridge
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
)

# Install
install(TARGETS
  robot_websocket_bridge
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()









