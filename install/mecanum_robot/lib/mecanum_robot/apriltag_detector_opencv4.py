#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import Quaternion

class AprilTagDetectorNode(Node):
    def __init__(self):
        super().__init__('apriltag_detector_node')
        
        # 创建发布者和订阅者
        self.image_pub = self.create_publisher(Image, '/apriltag_detection', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/apriltag_pose', 10)
        self.status_pub = self.create_publisher(String, '/apriltag_status', 10)
        
        # 订阅摄像头图像
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        # 订阅摄像头信息
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            10
        )
        
        # TF广播器
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # OpenCV桥接
        self.bridge = CvBridge()
        
        # 摄像头参数
        self.camera_matrix = None
        self.dist_coeffs = None
        self.camera_info_received = False
        
        # AprilTag检测器
        self.detector = cv2.aruco.ArucoDetector(
            cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11),
            cv2.aruco.DetectorParameters()
        )
        
        # AprilTag物理尺寸（米）
        self.tag_size = 0.1  # 10cm x 10cm标签
        
        # 检测到的标签信息
        self.detected_tags = {}
        
        # 创建定时器
        self.timer = self.create_timer(0.1, self.publish_status)
        
        self.get_logger().info('AprilTag检测节点已启动 (OpenCV 4.10兼容版)')
        self.get_logger().info('等待摄像头图像...')
    
    def camera_info_callback(self, msg):
        """处理摄像头信息"""
        if not self.camera_info_received:
            # 提取摄像头内参
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.camera_info_received = True
            self.get_logger().info('摄像头参数已接收')
    
    def image_callback(self, msg):
        """处理摄像头图像"""
        try:
            # 转换图像格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # 检测AprilTag
            self.detect_apriltags(cv_image, msg.header.stamp)
            
            # 发布处理后的图像
            self.publish_processed_image(cv_image)
            
        except Exception as e:
            self.get_logger().error(f'❌ 图像处理错误: {e}')
    
    def detect_apriltags(self, image, timestamp):
        """检测AprilTag"""
        if not self.camera_info_received:
            return
        
        # 转换为灰度图像
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # 检测AprilTag
        corners, ids, rejected = self.detector.detectMarkers(gray)
        
        if ids is not None and len(ids) > 0:
            # 使用solvePnP进行位姿估计
            self.estimate_pose_with_solvePnP(corners, ids, timestamp, image)
        else:
            # 清除旧的检测结果
            self.detected_tags.clear()
    
    def estimate_pose_with_solvePnP(self, corners, ids, timestamp, image):
        """使用solvePnP进行位姿估计"""
        # 定义AprilTag的3D点（以标签中心为原点）
        object_points = np.array([
            [-self.tag_size/2, -self.tag_size/2, 0],
            [self.tag_size/2, -self.tag_size/2, 0],
            [self.tag_size/2, self.tag_size/2, 0],
            [-self.tag_size/2, self.tag_size/2, 0]
        ], dtype=np.float32)
        
        # 处理每个检测到的标签
        for i, tag_id in enumerate(ids.flatten()):
            try:
                # 获取2D角点
                image_points = corners[i][0].astype(np.float32)
                
                # 使用solvePnP进行位姿估计
                success, rvec, tvec = cv2.solvePnP(
                    object_points,
                    image_points,
                    self.camera_matrix,
                    self.dist_coeffs
                )
                
                if success:
                    # 计算距离
                    distance = np.linalg.norm(tvec)
                    
                    # 计算角度
                    angle_x = math.atan2(tvec[0], tvec[2])
                    angle_y = math.atan2(tvec[1], tvec[2])
                    
                    # 存储标签信息
                    self.detected_tags[tag_id] = {
                        'position': tvec.flatten(),
                        'rotation': rvec.flatten(),
                        'distance': distance,
                        'angle_x': angle_x,
                        'angle_y': angle_y,
                        'timestamp': timestamp
                    }
                    
                    # 发布标签位姿
                    self.publish_tag_pose(tag_id, tvec.flatten(), rvec.flatten(), timestamp)
                    
                    # 发布TF变换
                    self.publish_tag_tf(tag_id, tvec.flatten(), rvec.flatten(), timestamp)
                    
                    # 在图像上绘制检测结果
                    self.draw_detection(image, corners[i], tag_id, tvec.flatten(), distance)
                    
                    # 终端输出检测结果
                    print(f"\n🎯 ===== AprilTag 检测结果 =====")
                    print(f"🆔 标签ID: {tag_id}")
                    print(f"📏 距离: {distance:.2f} 米")
                    print(f"📍 位置: X={tvec[0][0]:.2f}, Y={tvec[1][0]:.2f}, Z={tvec[2][0]:.2f}")
                    print(f"⏰ 时间戳: {timestamp.sec}.{timestamp.nanosec//1000000:03d}")
                    print(f"🎯 ==============================\n")
                    
                    # 同时记录到日志
                    self.get_logger().info(
                        f'🎯 检测到AprilTag ID: {tag_id}, '
                        f'距离: {distance:.2f}m, '
                        f'位置: ({tvec[0][0]:.2f}, {tvec[1][0]:.2f}, {tvec[2][0]:.2f})'
                    )
                    
            except Exception as e:
                self.get_logger().error(f'位姿估计错误: {e}')
    
    def publish_tag_pose(self, tag_id, position, rotation, timestamp):
        """发布标签位姿"""
        pose = PoseStamped()
        pose.header.stamp = timestamp
        # 在frame_id中携带标签ID，便于前端解析显示
        pose.header.frame_id = f'apriltag_{int(tag_id)}'
        
        # 位置
        pose.pose.position.x = position[0]
        pose.pose.position.y = position[1]
        pose.pose.position.z = position[2]
        
        # 旋转（从旋转向量转换为四元数）
        rotation_matrix, _ = cv2.Rodrigues(rotation)
        quaternion = self.rotation_matrix_to_quaternion(rotation_matrix)
        
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]
        
        self.pose_pub.publish(pose)
    
    def publish_tag_tf(self, tag_id, position, rotation, timestamp):
        """发布标签TF变换"""
        t = TransformStamped()
        t.header.stamp = timestamp
        t.header.frame_id = 'camera_link'
        t.child_frame_id = f'apriltag_{tag_id}'
        
        # 位置
        t.transform.translation.x = position[0]
        t.transform.translation.y = position[1]
        t.transform.translation.z = position[2]
        
        # 旋转
        rotation_matrix, _ = cv2.Rodrigues(rotation)
        quaternion = self.rotation_matrix_to_quaternion(rotation_matrix)
        
        t.transform.rotation.x = quaternion[0]
        t.transform.rotation.y = quaternion[1]
        t.transform.rotation.z = quaternion[2]
        t.transform.rotation.w = quaternion[3]
        
        self.tf_broadcaster.sendTransform(t)
    
    def draw_detection(self, image, corners, tag_id, position, distance):
        """在图像上绘制检测结果"""
        # 绘制标签轮廓 - 修复OpenCV 4.10兼容性
        try:
            cv2.aruco.drawDetectedMarkers(image, corners, np.array([tag_id]))
        except Exception as e:
            # 如果绘制失败，至少绘制角点
            cv2.polylines(image, [corners.astype(np.int32)], True, (0, 255, 0), 2)
        
        # 绘制坐标轴 - 简化版本避免复杂投影
        try:
            # 获取角点中心作为原点
            center = np.mean(corners, axis=1).astype(np.int32)[0]
            
            # 绘制简单的坐标轴
            axis_length = 30  # 像素长度
            cv2.line(image, tuple(center), tuple(center + [axis_length, 0]), (0, 0, 255), 3)  # X轴 - 红色
            cv2.line(image, tuple(center), tuple(center + [0, axis_length]), (0, 255, 0), 3)  # Y轴 - 绿色
            
        except Exception as e:
            self.get_logger().warn(f'绘制坐标轴失败: {e}')
        
        # 添加文本信息
        text = f"ID: {tag_id}, Dist: {distance:.2f}m"
        cv2.putText(image, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    
    def publish_processed_image(self, image):
        """发布处理后的图像"""
        try:
            msg = self.bridge.cv2_to_imgmsg(image, 'bgr8')
            self.image_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f'发布图像时出错: {e}')
    
    def publish_status(self):
        """发布检测状态"""
        status_msg = String()
        if self.detected_tags:
            status_msg.data = f"检测到 {len(self.detected_tags)} 个标签"
        else:
            status_msg.data = "未检测到标签"
        
        self.status_pub.publish(status_msg)
    
    def rotation_matrix_to_quaternion(self, R):
        """将旋转矩阵转换为四元数"""
        trace = np.trace(R)
        
        if trace > 0:
            s = np.sqrt(trace + 1.0) * 2
            w = 0.25 * s
            x = (R[2, 1] - R[1, 2]) / s
            y = (R[0, 2] - R[2, 0]) / s
            z = (R[1, 0] - R[0, 1]) / s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s
        
        return [x, y, z, w]

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagDetectorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
