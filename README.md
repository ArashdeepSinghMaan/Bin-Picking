# Bin-Picking
Robot Arm bin picking using Depth Camera, YOLOv5,GPD
Steps to Build this project on your system
1. mycobot_gazebo is a package containing description for robotic arm. copy in src and built it.
2. basic_robot contains launch files to get robot in the bin-picking world. copy in src and built it.
3. pose__estimation package has a node to which saves depth images and point clouds from camera feed. copy in src and built it.
4. yolo package has a node which detects objects from rgb image of camera. copy in src and built it.
5. 

For GPD Refer to https://github.com/atenpas/gpd

