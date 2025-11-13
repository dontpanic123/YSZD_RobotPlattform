#!/usr/bin/env python3

"""
AprilTag检测输出演示脚本
展示当AprilTag被检测到时的输出格式
"""

import time
import random

def simulate_apriltag_detection():
    """模拟AprilTag检测输出"""
    
    print("🤖 AprilTag检测系统演示")
    print("=" * 50)
    print("📋 系统状态:")
    print("   ✅ 摄像头节点: 运行中")
    print("   ✅ AprilTag检测节点: 运行中 (增强版)")
    print("   ✅ 位置计算节点: 运行中")
    print("   ✅ RViz可视化: 运行中")
    print()
    
    # 模拟检测过程
    for i in range(5):
        print(f"🔍 第 {i+1} 次检测...")
        time.sleep(1)
        
        # 随机决定是否检测到AprilTag
        if random.random() > 0.3:  # 70% 概率检测到
            # 模拟检测结果
            tag_id = random.randint(0, 4)
            distance = random.uniform(0.5, 3.0)
            x = random.uniform(-1.0, 1.0)
            y = random.uniform(-0.5, 0.5)
            z = random.uniform(0.5, 2.0)
            
            print(f"\n🎯 ===== AprilTag 检测结果 =====")
            print(f"🆔 标签ID: {tag_id}")
            print(f"📏 距离: {distance:.2f} 米")
            print(f"📍 位置: X={x:.2f}, Y={y:.2f}, Z={z:.2f}")
            print(f"⏰ 时间戳: {int(time.time())}.{int(time.time() * 1000) % 1000:03d}")
            print(f"🎯 ==============================\n")
            
            # 模拟日志输出
            print(f"[INFO] 🎯 检测到AprilTag ID: {tag_id}, 距离: {distance:.2f}m, 位置: ({x:.2f}, {y:.2f}, {z:.2f})")
            print()
        else:
            print("   ❌ 未检测到AprilTag")
            print()
    
    print("📊 检测统计:")
    print("   📈 总帧数: 500")
    print("   🎯 检测次数: 3")
    print("   📊 检测率: 60.0%")
    print()
    
    print("🎮 控制说明:")
    print("   w/s: 前进/后退")
    print("   a/d: 左移/右移")
    print("   q/e: 左转/右转")
    print("   x: 停止")
    print("   +/-: 调节速度")
    print()
    
    print("🔍 调试命令:")
    print("   ros2 topic echo /apriltag_pose")
    print("   ros2 topic echo /robot_relative_pose")
    print("   ros2 topic echo /apriltag_status")
    print("   ros2 run rqt_image_view rqt_image_view")

if __name__ == "__main__":
    simulate_apriltag_detection()
