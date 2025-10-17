#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml

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
    params_file_cmd = LaunchConfiguration('params_file')
    map_yaml_file = LaunchConfiguration('map')
    autostart = LaunchConfiguration('autostart')
    bt_loop_duration = LaunchConfiguration('bt_loop_duration')
    bt_loop_rate = LaunchConfiguration('bt_loop_rate')
    enable_bt_loop_logging = LaunchConfiguration('enable_bt_loop_logging')
    log_level = LaunchConfiguration('log_level')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    enable_rviz = LaunchConfiguration('enable_rviz')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    
    # 声明启动参数
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=params_file,
        description='Full path to the ROS2 parameters file to use')
    
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=map_file,
        description='Full path to map yaml file to load')
    
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')
    
    declare_bt_loop_duration_cmd = DeclareLaunchArgument(
        'bt_loop_duration',
        default_value='10',
        description='Duration in ms of each BT loop iteration')
    
    declare_bt_loop_rate_cmd = DeclareLaunchArgument(
        'bt_loop_rate',
        default_value='10',
        description='Rate in Hz of the BT loop')
    
    declare_enable_bt_loop_logging_cmd = DeclareLaunchArgument(
        'enable_bt_loop_logging',
        default_value='false',
        description='Enable logging of the BT loop')
    
    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level')
    
    declare_default_bt_xml_filename_cmd = DeclareLaunchArgument(
        'default_bt_xml_filename',
        default_value='navigate_w_replanning_and_recovery.xml',
        description='Full path to the behavior tree xml file to use')
    
    declare_enable_rviz_cmd = DeclareLaunchArgument(
        'enable_rviz',
        default_value='true',
        description='Whether to start RVIZ')
    
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(pkg_share, 'rviz', 'nav2_with_obstacles.rviz'),
        description='Full path to the RVIZ config file to use')
    
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
                    {'yaml_filename': map_yaml_file}])
    
    # 启动AMCL定位
    amcl_cmd = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_file_cmd])
    
    # 启动代价地图服务器
    global_costmap_cmd = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='global_costmap',
        output='screen',
        parameters=[params_file_cmd])
    
    local_costmap_cmd = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='local_costmap',
        output='screen',
        parameters=[params_file_cmd])
    
    # 启动路径规划器
    planner_server_cmd = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file_cmd])
    
    # 启动控制器
    controller_server_cmd = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file_cmd])
    
    # 启动行为树导航器
    bt_navigator_cmd = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file_cmd,
                    {'default_bt_xml_filename': default_bt_xml_filename,
                     'enable_bt_loop_logging': enable_bt_loop_logging,
                     'bt_loop_duration': bt_loop_duration,
                     'bt_loop_rate': bt_loop_rate}])
    
    # 启动路径平滑器
    smoother_server_cmd = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[params_file_cmd])
    
    # 启动生命周期管理器
    lifecycle_manager_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': autostart},
                    {'node_names': ['map_server',
                                   'amcl',
                                   'global_costmap',
                                   'local_costmap',
                                   'planner_server',
                                   'controller_server',
                                   'bt_navigator',
                                   'smoother_server']}])
    
    # 启动RViz
    rviz_cmd = Node(
        condition=IfCondition(enable_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen')
    
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
    
    # 创建启动描述
    ld = LaunchDescription()
    
    # 添加启动参数
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_bt_loop_duration_cmd)
    ld.add_action(declare_bt_loop_rate_cmd)
    ld.add_action(declare_enable_bt_loop_logging_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_default_bt_xml_filename_cmd)
    ld.add_action(declare_enable_rviz_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    
    # 添加节点
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(joint_state_publisher_cmd)
    ld.add_action(robot_node_cmd)
    ld.add_action(remote_control_cmd)
    ld.add_action(map_server_cmd)
    ld.add_action(amcl_cmd)
    ld.add_action(global_costmap_cmd)
    ld.add_action(local_costmap_cmd)
    ld.add_action(planner_server_cmd)
    ld.add_action(controller_server_cmd)
    ld.add_action(bt_navigator_cmd)
    ld.add_action(smoother_server_cmd)
    ld.add_action(lifecycle_manager_cmd)
    ld.add_action(rviz_cmd)
    
    return ld
