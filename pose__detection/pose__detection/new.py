#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
import cv2
import pcl
import numpy as np
import os
import time

class ImageSaverNode(Node):
    def __init__(self):
        super().__init__('image_saver_node')
        
        # Define topics
        self.rgb_topic = '/camera/rgb/intel_realsense_r200_rgb/image_raw'
        self.depth_topic = '/intel_realsense_r200_depth/depth/image_raw'
        self.pointcloud_topic = '/intel_realsense_r200_depth/points'
        
        # Set up subscriptions
        self.rgb_subscription = self.create_subscription(
            Image,
            self.rgb_topic,
            self.rgb_callback,
            10
        )
        self.depth_subscription = self.create_subscription(
            Image,
            self.depth_topic,
            self.depth_callback,
            10
        )
        self.pointcloud_subscription = self.create_subscription(
            PointCloud2,
            self.pointcloud_topic,
            self.pointcloud_callback,
            10
        )
        
        # Set up publishers for the saved images
        self.rgb_publisher = self.create_publisher(Image, '/saved_rgb_image', 10)
        self.depth_publisher = self.create_publisher(Image, '/saved_depth_image', 10)
        
        # Utilities
        self.bridge = CvBridge()
        self.count = 0
        self.timer = self.create_timer(20.0, self.save_images)

        # Create directories for saved files
        os.makedirs('saved_images', exist_ok=True)
        os.makedirs('saved_pointclouds', exist_ok=True)
        
        self.get_logger().info('ImageSaverNode started and waiting for data...')

    def rgb_callback(self, msg):
        # Convert ROS image to OpenCV format
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.get_logger().info('RGB image received.')

    def depth_callback(self, msg):
        # Convert ROS depth image to OpenCV format
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, "16UC1")
        self.get_logger().info('Depth image received.')

    def pointcloud_callback(self, msg):
        # Convert ROS PointCloud2 message to PCL format
        cloud_arr = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, 3)
        self.pointcloud = pcl.PointCloud(cloud_arr)
        self.get_logger().info('Point cloud received.')

    def save_images(self):
        if self.count < 3:
            # Save RGB image
            rgb_filename = f'saved_images/rgb_image_{self.count}.png'
            cv2.imwrite(rgb_filename, self.rgb_image)
            self.get_logger().info(f'Saved RGB image to {rgb_filename}')
            
            # Save Depth image
            depth_filename = f'saved_images/depth_image_{self.count}.png'
            cv2.imwrite(depth_filename, self.depth_image)
            self.get_logger().info(f'Saved Depth image to {depth_filename}')
            
            # Save Point cloud
            pointcloud_filename = f'saved_pointclouds/pointcloud_{self.count}.pcd'
            pcl.save(self.pointcloud, pointcloud_filename)
            self.get_logger().info(f'Saved Point Cloud to {pointcloud_filename}')

            # Publish the saved images
            rgb_msg = self.bridge.cv2_to_imgmsg(self.rgb_image, "bgr8")
            depth_msg = self.bridge.cv2_to_imgmsg(self.depth_image, "16UC1")
            self.rgb_publisher.publish(rgb_msg)
            self.depth_publisher.publish(depth_msg)
            self.get_logger().info(f'Published saved images.')

            self.count += 1
        else:
            self.get_logger().info('Completed three instances. Shutting down.')
            self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ImageSaverNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
