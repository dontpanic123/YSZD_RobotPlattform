#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

class ImprovedCameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        
        # 创建发布者
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 10)
        
        # OpenCV桥接
        self.bridge = CvBridge()
        
        # 摄像头参数
        self.camera_id = 0  # 默认摄像头ID
        self.cap = None
        self.camera_info_published = False
        
        # 摄像头内参
        self.camera_matrix = np.array([
            [640, 0, 320],    # fx, 0, cx
            [0, 640, 240],    # 0, fy, cy
            [0, 0, 1]         # 0, 0, 1
        ], dtype=np.float32)
        
        self.dist_coeffs = np.zeros((5, 1), dtype=np.float32)
        
        # 创建定时器
        self.timer = self.create_timer(0.033, self.publish_image)  # 30 FPS
        
        # 初始化摄像头
        self.init_camera()
        
        self.get_logger().info('改进的摄像头节点已启动')
    
    def init_camera(self):
        """初始化摄像头"""
        self.get_logger().info('正在初始化摄像头...')
        
        # 尝试不同的摄像头ID
        for camera_id in [0, 1, 2]:
            try:
                self.get_logger().info(f'尝试打开摄像头 {camera_id}...')
                self.cap = cv2.VideoCapture(camera_id)
                
                if self.cap.isOpened():
                    # 测试读取一帧
                    ret, frame = self.cap.read()
                    if ret and frame is not None:
                        self.camera_id = camera_id
                        self.get_logger().info(f'成功打开摄像头 {camera_id}')
                        
                        # 设置摄像头参数
                        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                        self.cap.set(cv2.CAP_PROP_FPS, 30)
                        
                        # 获取实际分辨率
                        width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                        height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                        
                        # 更新摄像头内参
                        self.camera_matrix[0, 2] = width / 2.0   # cx
                        self.camera_matrix[1, 2] = height / 2.0  # cy
                        self.camera_matrix[0, 0] = width * 0.8    # fx
                        self.camera_matrix[1, 1] = height * 0.8   # fy
                        
                        self.get_logger().info(f'摄像头分辨率: {width}x{height}')
                        self.get_logger().info(f'摄像头内参: {self.camera_matrix}')
                        return
                    else:
                        self.cap.release()
                        self.cap = None
                else:
                    self.get_logger().warn(f'摄像头 {camera_id} 无法打开')
                    
            except Exception as e:
                self.get_logger().error(f'摄像头 {camera_id} 初始化失败: {e}')
                if self.cap is not None:
                    self.cap.release()
                    self.cap = None
        
        self.get_logger().error('所有摄像头都无法打开！')
        self.get_logger().error('请检查：')
        self.get_logger().error('1. 摄像头是否连接')
        self.get_logger().error('2. 摄像头是否被其他程序占用')
        self.get_logger().error('3. 摄像头权限是否正确')
    
    def publish_image(self):
        """发布摄像头图像"""
        if self.cap is None or not self.cap.isOpened():
            return
        
        try:
            # 读取图像
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().warn('无法读取摄像头图像')
                return
            
            if frame is None:
                self.get_logger().warn('摄像头图像为空')
                return
            
            # 转换为ROS图像消息
            ros_image = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
            ros_image.header.stamp = self.get_clock().now().to_msg()
            ros_image.header.frame_id = 'camera_link'
            
            # 发布图像
            self.image_pub.publish(ros_image)
            
            # 发布摄像头信息（只在第一次发布）
            if not self.camera_info_published:
                self.publish_camera_info()
                self.camera_info_published = True
                
        except Exception as e:
            self.get_logger().error(f'图像发布错误: {e}')
    
    def publish_camera_info(self):
        """发布摄像头信息"""
        try:
            camera_info = CameraInfo()
            camera_info.header.stamp = self.get_clock().now().to_msg()
            camera_info.header.frame_id = 'camera_link'
            
            # 设置摄像头参数
            camera_info.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            camera_info.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            
            # 设置内参矩阵
            camera_info.k = self.camera_matrix.flatten().tolist()
            
            # 设置畸变系数
            camera_info.d = self.dist_coeffs.flatten().tolist()
            
            # 设置投影矩阵
            camera_info.p = [
                self.camera_matrix[0, 0], 0, self.camera_matrix[0, 2], 0,
                0, self.camera_matrix[1, 1], self.camera_matrix[1, 2], 0,
                0, 0, 1, 0
            ]
            
            # 设置旋转矩阵（单位矩阵）
            camera_info.r = [1, 0, 0, 0, 1, 0, 0, 0, 1]
            
            self.camera_info_pub.publish(camera_info)
            self.get_logger().info('摄像头信息已发布')
            
        except Exception as e:
            self.get_logger().error(f'摄像头信息发布错误: {e}')
    
    def destroy_node(self):
        """清理资源"""
        if self.cap is not None:
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ImprovedCameraNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()








