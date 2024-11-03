import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
#import pcl_ros
import pcl.pcl_visualization
import numpy as np
import cv2
from pcl import PointCloud_PointXYZ
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

class GraspPointDetectorNode(Node):
    def __init__(self):
        super().__init__('grasp_point_detector')
        self.subscription = self.create_subscription(Image, 'camera/depth', self.depth_callback, 10)
        self.publisher = self.create_publisher(PointCloud2, 'pose_estimation', 10)
        self.bridge = CvBridge()

    def depth_callback(self, msg):
        try:
            # Convert the ROS Image message to a NumPy array
            cv_depth_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")

            # Perform pose estimation using PCL
            point_cloud = self.depth_to_point_cloud(cv_depth_image)

            # Publish the pose estimation result
            self.publisher.publish(self.convert_to_pointcloud2(point_cloud))
        except Exception as e:
            self.get_logger().error(f"Error processing depth image: {e}")

    def depth_to_point_cloud(self, depth_image):
        # Assuming the depth image is already in meters and contains the Z-axis depth
        height, width = depth_image.shape
        point_cloud = []
        
        fx, fy = 525.0, 525.0  # Focal length
        cx, cy = width // 2, height // 2  # Principal point
        
        for v in range(height):
            for u in range(width):
                z = depth_image[v, u]
                if z == 0:  # Skip invalid points
                    continue
                x = (u - cx) * z / fx
                y = (v - cy) * z / fy
                point_cloud.append([x, y, z])

        return np.array(point_cloud)

    def convert_to_pointcloud2(self, points):
        header = self.get_clock().now().to_msg()
        cloud_data = pc2.create_cloud_xyz32(header, points)
        return cloud_data

def main(args=None):
    rclpy.init(args=args)
    node = GraspPointDetectorNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
