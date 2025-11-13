#!/usr/bin/env python3

import cv2
import numpy as np
import sys

def diagnose_camera():
    """诊断摄像头问题"""
    print("🔍 摄像头诊断工具")
    print("=" * 40)
    
    # 检查OpenCV版本
    print(f"📦 OpenCV版本: {cv2.__version__}")
    
    # 检查可用的摄像头
    print("\n📹 检查可用摄像头...")
    available_cameras = []
    
    for camera_id in range(5):  # 检查前5个摄像头
        try:
            cap = cv2.VideoCapture(camera_id)
            if cap.isOpened():
                ret, frame = cap.read()
                if ret and frame is not None:
                    height, width = frame.shape[:2]
                    print(f"✅ 摄像头 {camera_id}: {width}x{height}")
                    available_cameras.append(camera_id)
                else:
                    print(f"❌ 摄像头 {camera_id}: 无法读取图像")
                cap.release()
            else:
                print(f"❌ 摄像头 {camera_id}: 无法打开")
        except Exception as e:
            print(f"❌ 摄像头 {camera_id}: 错误 - {e}")
    
    if not available_cameras:
        print("\n🚨 未找到可用的摄像头！")
        print("💡 请检查:")
        print("   - 摄像头是否正确连接")
        print("   - 摄像头权限是否正确")
        print("   - 其他程序是否正在使用摄像头")
        return False
    
    print(f"\n✅ 找到 {len(available_cameras)} 个可用摄像头: {available_cameras}")
    
    # 测试第一个可用摄像头
    camera_id = available_cameras[0]
    print(f"\n🧪 测试摄像头 {camera_id}...")
    
    try:
        cap = cv2.VideoCapture(camera_id)
        if cap.isOpened():
            # 设置参数
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            cap.set(cv2.CAP_PROP_FPS, 30)
            
            # 获取实际参数
            width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            fps = cap.get(cv2.CAP_PROP_FPS)
            
            print(f"📐 分辨率: {width}x{height}")
            print(f"🎬 帧率: {fps} FPS")
            
            # 测试读取几帧
            success_count = 0
            for i in range(10):
                ret, frame = cap.read()
                if ret and frame is not None:
                    success_count += 1
                    print(f"✅ 帧 {i+1}: 成功读取 {frame.shape}")
                else:
                    print(f"❌ 帧 {i+1}: 读取失败")
            
            print(f"\n📊 成功率: {success_count}/10 ({success_count*10}%)")
            
            if success_count >= 8:
                print("✅ 摄像头工作正常")
                return True
            else:
                print("⚠️ 摄像头工作不稳定")
                return False
        else:
            print("❌ 无法打开摄像头")
            return False
            
    except Exception as e:
        print(f"❌ 测试摄像头时出错: {e}")
        return False
    finally:
        if 'cap' in locals():
            cap.release()
    
    return False

def test_image_processing():
    """测试图像处理功能"""
    print("\n🖼️ 测试图像处理功能...")
    
    try:
        # 创建测试图像
        test_image = np.zeros((480, 640, 3), dtype=np.uint8)
        test_image[:] = (100, 150, 200)  # 蓝色背景
        
        # 添加一些图形
        cv2.rectangle(test_image, (100, 100), (300, 200), (0, 255, 0), 2)
        cv2.circle(test_image, (400, 300), 50, (0, 0, 255), -1)
        
        print(f"✅ 测试图像创建成功: {test_image.shape}")
        
        # 测试图像转换
        from cv_bridge import CvBridge
        bridge = CvBridge()
        
        # 转换为ROS图像消息
        ros_image = bridge.cv2_to_imgmsg(test_image, 'bgr8')
        print(f"✅ ROS图像消息创建成功: {ros_image.width}x{ros_image.height}")
        
        # 测试畸变系数
        dist_coeffs = np.zeros((5, 1), dtype=np.float32)
        print(f"✅ 畸变系数形状: {dist_coeffs.shape}")
        
        # 测试访问畸变系数
        try:
            d_values = [
                float(dist_coeffs[0, 0]), float(dist_coeffs[1, 0]), 
                float(dist_coeffs[2, 0]), float(dist_coeffs[3, 0]), 
                float(dist_coeffs[4, 0])
            ]
            print(f"✅ 畸变系数访问成功: {d_values}")
        except Exception as e:
            print(f"❌ 畸变系数访问失败: {e}")
            return False
        
        return True
        
    except Exception as e:
        print(f"❌ 图像处理测试失败: {e}")
        return False

def main():
    print("🔧 机器人摄像头诊断工具")
    print("=" * 50)
    
    # 检查摄像头
    camera_ok = diagnose_camera()
    
    # 测试图像处理
    processing_ok = test_image_processing()
    
    print("\n📋 诊断结果:")
    print(f"📹 摄像头状态: {'✅ 正常' if camera_ok else '❌ 异常'}")
    print(f"🖼️ 图像处理: {'✅ 正常' if processing_ok else '❌ 异常'}")
    
    if camera_ok and processing_ok:
        print("\n🎉 所有测试通过！摄像头系统工作正常。")
        return 0
    else:
        print("\n⚠️ 发现问题，请检查上述错误信息。")
        return 1

if __name__ == '__main__':
    sys.exit(main())























