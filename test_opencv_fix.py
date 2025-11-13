#!/usr/bin/env python3

"""
测试OpenCV AprilTag API修复
"""

import cv2
import numpy as np

def test_opencv_apriltag_api():
    """测试OpenCV AprilTag API兼容性"""
    
    print("🔍 测试OpenCV AprilTag API兼容性...")
    print(f"OpenCV版本: {cv2.__version__}")
    
    try:
        # 测试ArUco检测器创建
        detector = cv2.aruco.ArucoDetector(
            cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11),
            cv2.aruco.DetectorParameters()
        )
        print("✅ ArUco检测器创建成功")
        
        # 创建测试图像
        test_image = np.ones((480, 640), dtype=np.uint8) * 128
        
        # 测试检测功能
        corners, ids, rejected = detector.detectMarkers(test_image)
        print("✅ 检测功能正常")
        
        # 测试位姿估计API
        if hasattr(cv2.aruco, 'estimatePoseSingleMarkers'):
            print("✅ estimatePoseSingleMarkers 函数存在")
            
            # 创建测试数据
            test_corners = [np.array([[[100, 100], [200, 100], [200, 200], [100, 200]]], dtype=np.float32)]
            test_camera_matrix = np.array([[640, 0, 320], [0, 640, 240], [0, 0, 1]], dtype=np.float32)
            test_dist_coeffs = np.zeros((5, 1), dtype=np.float32)
            test_tag_size = 0.1
            
            try:
                # 测试新的API调用方式
                rvecs, tvecs = cv2.aruco.estimatePoseSingleMarkers(
                    test_corners, test_tag_size, test_camera_matrix, test_dist_coeffs
                )
                print("✅ 新的API调用方式成功")
                print(f"   旋转向量形状: {rvecs.shape}")
                print(f"   平移向量形状: {tvecs.shape}")
                
            except Exception as e:
                print(f"❌ 新API调用失败: {e}")
                
                # 尝试旧的API调用方式
                try:
                    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                        test_corners, test_tag_size, test_camera_matrix, test_dist_coeffs
                    )
                    print("✅ 旧API调用方式成功")
                except Exception as e2:
                    print(f"❌ 旧API调用也失败: {e2}")
        else:
            print("❌ estimatePoseSingleMarkers 函数不存在")
            
    except Exception as e:
        print(f"❌ 测试失败: {e}")
        return False
    
    print("🎯 OpenCV AprilTag API测试完成")
    return True

if __name__ == "__main__":
    test_opencv_apriltag_api()
