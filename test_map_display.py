#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class MapDisplayTest(Node):
    def __init__(self):
        super().__init__('map_display_test')
        
        # 订阅地图话题
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        
        # 创建图像发布者
        self.image_pub = self.create_publisher(Image, '/map_image', 10)
        self.bridge = CvBridge()
        
        self.get_logger().info('地图显示测试节点已启动')
    
    def map_callback(self, msg):
        self.get_logger().info(f'收到地图数据: {msg.info.width}x{msg.info.height}, 分辨率: {msg.info.resolution}')
        
        # 将地图转换为图像
        width = msg.info.width
        height = msg.info.height
        
        # 创建图像数组
        map_data = np.array(msg.data, dtype=np.int8).reshape((height, width))
        
        # 将占用栅格转换为图像
        # -1: 未知, 0: 自由, 100: 占用
        image = np.zeros((height, width, 3), dtype=np.uint8)
        
        # 未知区域 - 灰色
        image[map_data == -1] = [128, 128, 128]
        # 自由区域 - 白色
        image[map_data == 0] = [255, 255, 255]
        # 占用区域 - 黑色
        image[map_data == 100] = [0, 0, 0]
        
        # 发布图像
        try:
            ros_image = self.bridge.cv2_to_imgmsg(image, "bgr8")
            ros_image.header = msg.header
            self.image_pub.publish(ros_image)
            self.get_logger().info('地图图像已发布到 /map_image')
        except Exception as e:
            self.get_logger().error(f'图像转换错误: {e}')

def main():
    rclpy.init()
    node = MapDisplayTest()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()




