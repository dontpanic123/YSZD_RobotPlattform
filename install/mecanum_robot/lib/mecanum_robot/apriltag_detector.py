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
        
        self.get_logger().info('AprilTag检测节点已启动')
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
            self.get_logger().error(f'图像处理错误: {e}')
    
    def detect_apriltags(self, image, timestamp):
        """检测AprilTag"""
        if not self.camera_info_received:
            return
        
        # 转换为灰度图像
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # 检测AprilTag
        corners, ids, rejected = self.detector.detectMarkers(gray)
        
        if ids is not None and len(ids) > 0:
            # 计算标签的位姿
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.tag_size, self.camera_matrix, self.dist_coeffs
            )
            
            # 处理每个检测到的标签
            for i, tag_id in enumerate(ids.flatten()):
                # 计算标签相对于摄像头的位置
                tag_position = tvecs[i][0]
                tag_rotation = rvecs[i][0]
                
                # 计算距离
                distance = np.linalg.norm(tag_position)
                
                # 计算角度
                angle_x = math.atan2(tag_position[0], tag_position[2])
                angle_y = math.atan2(tag_position[1], tag_position[2])
                
                # 存储标签信息
                self.detected_tags[tag_id] = {
                    'position': tag_position,
                    'rotation': tag_rotation,
                    'distance': distance,
                    'angle_x': angle_x,
                    'angle_y': angle_y,
                    'timestamp': timestamp
                }
                
                # 发布标签位姿
                self.publish_tag_pose(tag_id, tag_position, tag_rotation, timestamp)
                
                # 发布TF变换
                self.publish_tag_tf(tag_id, tag_position, tag_rotation, timestamp)
                
                # 在图像上绘制检测结果
                self.draw_detection(image, corners[i], tag_id, tag_position, distance)
                
                # 终端输出检测结果
                print(f"\n🎯 ===== AprilTag 检测结果 =====")
                print(f"🆔 标签ID: {tag_id}")
                print(f"📏 距离: {distance:.2f} 米")
                print(f"📍 位置: X={tag_position[0]:.2f}, Y={tag_position[1]:.2f}, Z={tag_position[2]:.2f}")
                print(f"⏰ 时间戳: {timestamp.sec}.{timestamp.nanosec//1000000:03d}")
                print(f"🎯 ==============================\n")
                
                # 同时记录到日志
                self.get_logger().info(
                    f'🎯 检测到AprilTag ID: {tag_id}, '
                    f'距离: {distance:.2f}m, '
                    f'位置: ({tag_position[0]:.2f}, {tag_position[1]:.2f}, {tag_position[2]:.2f})'
                )
        else:
            # 清除旧的检测结果
            self.detected_tags.clear()
    
    def publish_tag_pose(self, tag_id, position, rotation, timestamp):
        """发布标签位姿"""
        pose = PoseStamped()
        pose.header.stamp = timestamp
        pose.header.frame_id = 'camera_link'
        
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
        # 绘制绿色边框的标签轮廓
        cv2.aruco.drawDetectedMarkers(image, [corners], [tag_id])
        
        # 绘制更粗的绿色边框
        corners_int = np.int32(corners[0])
        cv2.polylines(image, [corners_int], True, (0, 255, 0), 3)
        
        # 绘制坐标轴
        cv2.aruco.drawAxis(
            image, self.camera_matrix, self.dist_coeffs,
            np.zeros(3), np.zeros(3), self.tag_size * 0.5
        )
        
        # 计算标签中心点
        center_x = int(np.mean(corners[0][:, 0]))
        center_y = int(np.mean(corners[0][:, 1]))
        
        # 在标签中心绘制绿色圆点
        cv2.circle(image, (center_x, center_y), 5, (0, 255, 0), -1)
        
        # 添加ID标签（绿色背景）
        id_text = f'ID: {tag_id}'
        text_size = cv2.getTextSize(id_text, cv2.FONT_HERSHEY_SIMPLEX, 0.8, 2)[0]
        text_x = center_x - text_size[0] // 2
        text_y = center_y - 20
        
        # 绘制绿色背景
        cv2.rectangle(image, 
                     (text_x - 5, text_y - text_size[1] - 5), 
                     (text_x + text_size[0] + 5, text_y + 5), 
                     (0, 255, 0), -1)
        
        # 绘制黑色文字
        cv2.putText(image, id_text, (text_x, text_y), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 2)
        
        # 添加距离信息
        dist_text = f'Dist: {distance:.2f}m'
        cv2.putText(image, dist_text, (text_x, text_y + 25), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        # 在图像顶部添加检测状态
        status_text = f'AprilTag Detected: ID {tag_id}'
        cv2.putText(image, status_text, (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        
        # 添加位置信息
        pos_text = f'Position: ({position[0]:.2f}, {position[1]:.2f}, {position[2]:.2f})'
        cv2.putText(image, pos_text, (10, 60), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    
    def publish_processed_image(self, image):
        """发布处理后的图像"""
        try:
            ros_image = self.bridge.cv2_to_imgmsg(image, 'bgr8')
            self.image_pub.publish(ros_image)
        except Exception as e:
            self.get_logger().error(f'图像发布错误: {e}')
    
    def rotation_matrix_to_quaternion(self, R):
        """将旋转矩阵转换为四元数"""
        trace = np.trace(R)
        if trace > 0:
            s = math.sqrt(trace + 1.0) * 2
            w = 0.25 * s
            x = (R[2, 1] - R[1, 2]) / s
            y = (R[0, 2] - R[2, 0]) / s
            z = (R[1, 0] - R[0, 1]) / s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s
        
        return [x, y, z, w]
    
    def publish_status(self):
        """发布检测状态"""
        status_msg = String()
        if self.detected_tags:
            tag_ids = list(self.detected_tags.keys())
            status_msg.data = f"检测到 {len(tag_ids)} 个AprilTag: {tag_ids}"
        else:
            status_msg.data = "未检测到AprilTag"
        
        self.status_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = AprilTagDetectorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
