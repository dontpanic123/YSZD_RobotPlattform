#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
import os

def generate_launch_description():
    # 获取包路径
    pkg_share = FindPackageShare(package='mecanum_robot').find('mecanum_robot')
    urdf_file = os.path.join(pkg_share, 'urdf', 'mecanum_robot.urdf')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'mecanum_robot_path.rviz')
    
    return LaunchDescription([
        # 声明启动参数
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='使用仿真时间'
        ),
        DeclareLaunchArgument(
            'urdf_file',
            default_value=urdf_file,
            description='URDF文件路径'
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=rviz_config_file,
            description='RViz配置文件路径'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='是否启动RViz'
        ),
        DeclareLaunchArgument(
            'use_camera',
            default_value='true',
            description='是否启动摄像头'
        ),
        DeclareLaunchArgument(
            'use_apriltag',
            default_value='true',
            description='是否启动AprilTag检测'
        ),
        DeclareLaunchArgument(
            'use_navigation',
            default_value='true',
            description='是否启动导航控制'
        ),
        DeclareLaunchArgument(
            'use_remote_control',
            default_value='false',
            description='是否启动遥控节点'
        ),
        
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': open(urdf_file, 'r').read(),
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        ),
        
        # Joint State Publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        ),
        
        # 麦克纳姆轮机器人节点
        Node(
            package='mecanum_robot',
            executable='robot_node.py',
            name='mecanum_robot_node',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        ),
        
        # 遥控节点 (可选)
        Node(
            package='mecanum_robot',
            executable='remote_control.py',
            name='remote_control_node',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }],
            condition=IfCondition(LaunchConfiguration('use_remote_control'))
        ),
        
        # 路径发布节点
        Node(
            package='mecanum_robot',
            executable='path_publisher.py',
            name='path_publisher_node',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        ),
        
        # 修复版摄像头节点
        Node(
            package='mecanum_robot',
            executable='camera_node_fixed.py',
            name='camera_node',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }],
            condition=IfCondition(LaunchConfiguration('use_camera'))
        ),
        
        # AprilTag检测节点 (OpenCV 4.10兼容版)
        Node(
            package='mecanum_robot',
            executable='apriltag_detector_opencv4.py',
            name='apriltag_detector_node',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }],
            condition=IfCondition(LaunchConfiguration('use_apriltag'))
        ),
        
        # 位置计算节点
        Node(
            package='mecanum_robot',
            executable='position_calculator.py',
            name='position_calculator_node',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }],
            condition=IfCondition(LaunchConfiguration('use_navigation'))
        ),
        
        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }],
            condition=IfCondition(LaunchConfiguration('use_rviz'))
        ),
    ])