#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageViewerNode(Node):
    def __init__(self):
        super().__init__('image_viewer_node')
        
        # 创建订阅者
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        # OpenCV桥接
        self.bridge = CvBridge()
        
        self.get_logger().info('图像查看器节点已启动')
        self.get_logger().info('等待摄像头图像...')
    
    def image_callback(self, msg):
        """处理摄像头图像"""
        try:
            # 转换图像格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # 显示图像
            cv2.imshow('Camera Feed', cv_image)
            
            # 按 'q' 键退出
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.get_logger().info('用户退出')
                cv2.destroyAllWindows()
                rclpy.shutdown()
                
        except Exception as e:
            self.get_logger().error(f'图像处理错误: {e}')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ImageViewerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()



















