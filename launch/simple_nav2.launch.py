#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 获取包路径
    pkg_share = FindPackageShare(package='mecanum_robot').find('mecanum_robot')
    
    # 参数文件路径
    params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    
    # 地图文件路径
    map_file = '/home/bd/Documents/Robot/agv_sim/maps/map/test_map.yaml'
    
    # 机器人描述文件
    urdf_file = os.path.join(pkg_share, 'urdf', 'mecanum_robot.urdf')
    
    # 声明启动参数
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # 声明启动参数
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    
    # 启动机器人状态发布器
    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[urdf_file])
    
    # 启动关节状态发布器
    joint_state_publisher_cmd = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}])
    
    # 启动地图服务器
    map_server_cmd = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'yaml_filename': map_file}])
    
    # 启动机器人节点
    robot_node_cmd = Node(
        package='mecanum_robot',
        executable='robot_node.py',
        name='robot_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}])
    
    # 启动遥控节点
    remote_control_cmd = Node(
        package='mecanum_robot',
        executable='remote_control.py',
        name='remote_control',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}])
    
    # 启动TF变换发布器
    static_transform_cmd = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen')
    
    # 启动RViz
    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', '/home/bd/Documents/Robot/agv_sim/rviz/nav2_with_obstacles.rviz'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen')
    
    # 创建启动描述
    ld = LaunchDescription()
    
    # 添加启动参数
    ld.add_action(declare_use_sim_time_cmd)
    
    # 添加节点
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(joint_state_publisher_cmd)
    ld.add_action(robot_node_cmd)
    ld.add_action(remote_control_cmd)
    ld.add_action(static_transform_cmd)
    ld.add_action(map_server_cmd)
    ld.add_action(rviz_cmd)
    
    return ld








