# yolov5_inference/yolov5_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import torch
import cv2
import numpy as np
from datetime import datetime

class YOLOv5Node(Node):
    def __init__(self, model_path, image_topic, save_dir, max_instances=5):
        super().__init__('yolov5_node')
        self.bridge = CvBridge()
        self.model = torch.hub.load('ultralytics/yolov5', 'custom', path=model_path)
        self.subscription = self.create_subscription(
            Image, image_topic, self.listener_callback, 10)
        self.save_dir = save_dir
        self.instance_count = 0
        self.max_instances = max_instances
        self.get_logger().info("YOLOv5 Node has been started.")

    def listener_callback(self, msg):
        # Convert ROS image message to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Perform YOLOv5 inference
        results = self.model(cv_image)

        # Draw bounding boxes and labels on the image
        img_with_boxes = np.squeeze(results.render())  # results.render() returns a list

        # Save the image with detections
        timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        save_path = f"{self.save_dir}/detected_{timestamp}.jpg"
        cv2.imwrite(save_path, img_with_boxes)
        self.get_logger().info(f"Saved detection result to {save_path}")

        # Increment and check instance count
        self.instance_count += 1
        if self.instance_count >= self.max_instances:
            self.get_logger().info("Max instances reached, shutting down node.")
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)

    model_path = "/home/arashmaan/my_work/src/yolo/best.pt"  # Change to your YOLOv5 model path
    image_topic = "/camera/rgb/intel_realsense_r200_rgb/image_raw"        # Change to your camera image topic
    save_dir = "/home/arashmaan/Pictures/"        # Change to your save directory

    yolov5_node = YOLOv5Node(model_path, image_topic, save_dir)
    rclpy.spin(yolov5_node)

    yolov5_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

