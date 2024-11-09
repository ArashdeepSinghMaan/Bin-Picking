#  Robot Arm bin picking using Depth Camera, YOLOv5,GPD, ROS2
This project presents an integrated robotic bin-picking system that uses advanced computer vision techniques and ROS2 for real-time object detection, pose estimation, and grasp planning. The system utilizes YOLO for detecting objects in RGB images, Point Cloud Library (PCL) for estimating the orientation and position  from depth data, and Grasp Pose Detection (GPD) calculate grasp points. These components enable the robot to perform efficient and reliable pick-and-place tasks. The entire process is modularized within ROS2, ensuring smooth communication MoveIt! is used for motion planning and execution.
Robot Arm bin picking using Depth Camera, YOLOv5,GPD
Steps to Build this project on your system
1. mycobot_gazebo is a package containing description for robotic arm. copy in src and built it.
2. basic_robot contains launch files to get robot in the bin-picking world. copy in src and built it.
3. pose__estimation package has a node to which saves depth images and point clouds from camera feed. copy in src and built it.
4. yolo package has a node which detects objects from rgb image of camera. copy in src and built it.
5. mycobot_moveit_config_manual_setup contains moveit configuration files.

For GPD Refer to https://github.com/atenpas/gpd

intel realsense folder has files for camera model, you can place it in ~/.gazebo/models folder.

