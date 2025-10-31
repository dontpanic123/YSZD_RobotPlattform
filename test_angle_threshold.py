#!/usr/bin/env python3
"""
测试角度阈值逻辑
"""

import subprocess
import time
import os

def test_angle_threshold():
    """测试角度阈值逻辑"""
    print("🧪 测试角度阈值逻辑")
    print("=" * 50)
    
    print("📋 测试场景:")
    print("1. 小角度转向 (< 60度): 减速前进")
    print("2. 大角度转向 (≥ 60度): 停止前进")
    print("3. 正常前进: 全速前进")
    
    print("\n🔧 参数设置:")
    print("  - turn_stop_angle_threshold: 1.047 (60度)")
    print("  - stop_during_turn: True")
    print("  - linear_speed: 0.5")
    print("  - angular_speed: 0.6")
    
    try:
        # 设置参数
        print("\n⚙️ 设置参数...")
        
        subprocess.run([
            "ros2", "param", "set", "/simple_waypoint_follower", 
            "turn_stop_angle_threshold", "1.047"
        ], check=True)
        
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
        
        print("✅ 参数设置完成")
        
        print("\n📊 监控日志输出...")
        print("观察以下日志:")
        print("  - '大角度转向(X度)，停止前进' - 角度差 > 60度")
        print("  - '小角度转向(X度)，减速前进' - 角度差 ≤ 60度")
        print("  - 正常前进时无特殊日志")
        print("\n按Ctrl+C停止监控")
        
        # 监控日志
        subprocess.run([
            "ros2", "topic", "echo", "/rosout", "--filter", 
            "m.name=='simple_waypoint_follower'"
        ])
        
    except KeyboardInterrupt:
        print("\n⏹️ 停止测试")
    except subprocess.CalledProcessError as e:
        print(f"❌ 错误: {e}")
    except Exception as e:
        print(f"❌ 未知错误: {e}")

if __name__ == "__main__":
    test_angle_threshold()










