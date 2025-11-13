#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # 声明启动参数
        DeclareLaunchArgument(
            'use_state_machine',
            default_value='true',
            description='是否启动状态机节点'
        ),
        DeclareLaunchArgument(
            'use_state_controller',
            default_value='false',
            description='是否启动状态控制器（用于测试）'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='使用仿真时间'
        ),
        
        # 状态机节点
        Node(
            package='mecanum_robot',
            executable='robot_state_machine.py',
            name='robot_state_machine',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }],
            condition=launch.conditions.IfCondition(LaunchConfiguration('use_state_machine'))
        ),
        
        # 状态控制器节点（可选，用于测试）
        Node(
            package='mecanum_robot',
            executable='robot_state_controller.py',
            name='robot_state_controller',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }],
            condition=launch.conditions.IfCondition(LaunchConfiguration('use_state_controller'))
        ),
    ])
