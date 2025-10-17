#!/usr/bin/env python3
"""
测试waypoint follower的转圈问题修复
"""

import subprocess
import time
import os

def test_waypoint_follower():
    """测试waypoint follower"""
    print("🧪 测试Waypoint Follower转圈问题修复")
    print("=" * 50)
    
    # 检查是否有waypoint文件
    waypoints_dir = "/home/bd/Documents/Robot/agv_sim/waypoints"
    if not os.path.exists(waypoints_dir):
        print("❌ Waypoints目录不存在")
        return
    
    # 获取最新的waypoint文件
    waypoint_files = [f for f in os.listdir(waypoints_dir) if f.endswith('.json')]
    if not waypoint_files:
        print("❌ 没有找到waypoint文件")
        return
    
    latest_file = sorted(waypoint_files)[-1]
    waypoint_path = os.path.join(waypoints_dir, latest_file)
    print(f"📁 使用waypoint文件: {latest_file}")
    
    # 设置参数
    print("\n🔧 设置参数:")
    print("  - stop_during_turn: True (转向时停止前进)")
    print("  - turn_stop_angle_threshold: 1.047 (60度)")
    print("  - linear_speed: 0.5")
    print("  - angular_speed: 0.6")
    print("  - orientation_tolerance: 0.15")
    
    # 启动waypoint follower
    print("\n🚀 启动Waypoint Follower...")
    try:
        # 设置参数
        subprocess.run([
            "ros2", "param", "set", "/simple_waypoint_follower", 
            "stop_during_turn", "true"
        ], check=True)
        
        subprocess.run([
            "ros2", "param", "set", "/simple_waypoint_follower", 
            "linear_speed", "0.5"
        ], check=True)
        
        subprocess.run([
            "ros2", "param", "set", "/simple_waypoint_follower", 
            "angular_speed", "0.6"
        ], check=True)
        
        subprocess.run([
            "ros2", "param", "set", "/simple_waypoint_follower", 
            "orientation_tolerance", "0.15"
        ], check=True)
        
        subprocess.run([
            "ros2", "param", "set", "/simple_waypoint_follower", 
            "turn_stop_angle_threshold", "1.047"
        ], check=True)
        
        print("✅ 参数设置完成")
        
        # 设置waypoint文件
        subprocess.run([
            "ros2", "service", "call", "/set_waypoints_file", 
            "std_srvs/srv/Empty"
        ], check=True)
        
        print("✅ Waypoint文件设置完成")
        
        print("\n📊 监控cmd_vel话题...")
        print("观察机器人是否还会转圈")
        print("按Ctrl+C停止监控")
        
        # 监控cmd_vel话题
        subprocess.run([
            "ros2", "topic", "echo", "/cmd_vel"
        ])
        
    except KeyboardInterrupt:
        print("\n⏹️ 停止测试")
    except subprocess.CalledProcessError as e:
        print(f"❌ 错误: {e}")
    except Exception as e:
        print(f"❌ 未知错误: {e}")

if __name__ == "__main__":
    test_waypoint_follower()
