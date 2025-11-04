# 🤖 Robot Arm Bin-Picking using Depth Camera, YOLOv5, GPD, and ROS2 — *Ongoing Project*

This project presents an integrated **robotic bin-picking system** developed using **ROS2** that combines **RGB-D vision**, **deep learning-based object detection**, **3D pose estimation**, and **grasp planning** for industrial automation.
The system enables a robot manipulator to autonomously **detect, localize, grasp, and place** objects from a bin in real-time.

---

## 🧠 Overview

This project integrates:

* **YOLOv5** for real-time object detection from RGB camera input.
* **Point Cloud Library (PCL)** for estimating object orientation and position from depth data.
* **Grasp Pose Detection (GPD)** for computing optimal grasp points.
* **MoveIt2** for motion planning and execution of pick-and-place trajectories.

All modules are designed as **independent ROS2 nodes**, ensuring modularity, scalability, and real-time performance.
The system is tested in **Gazebo** simulation using **MyCobot** and **UR5** robotic arms, with an **Intel RealSense depth camera** providing RGB-D data.

---

## 🏷️ Project Workflow

### 🧩 System Architecture
Pipeline Overview:

The RGB-D camera captures color and depth data.

The YOLOv5 Node detects and classifies objects from the RGB stream.

The Pose Estimation Node uses PCL to determine the 6-DOF pose of detected objects.

The GPD Node generates grasp poses from point clouds.

The MoveIt2 Node plans and executes the robot’s pick-and-place motions.

---

## 🧱 Steps to Build and Run the Project

### 1️⃣ **Setup ROS2 Workspace**

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

---

### 2️⃣ **Add and Build Required Packages**

#### 🔹 mycobot_gazebo

Contains the **URDF/Xacro** files and simulation description of the **MyCobot** robotic arm.
Copy this package into `src` and build it.

```bash
colcon build --packages-select mycobot_gazebo
```

---

#### 🔹 basic_robot

Contains **launch files** to spawn the robot arm inside the **bin-picking Gazebo world**.
Copy this package into `src` and build it.

```bash
colcon build --packages-select basic_robot
```

To launch the simulation:

```bash
ros2 launch basic_robot bin_picking.launch.py
```

---

#### 🔹 pose__detection

Contains a **ROS2 node** that subscribes to the **camera topics** to save **depth images and point clouds** from the camera feed.
Copy this package into `src` and build it.

```bash
colcon build --packages-select pose__detection
```

Run the node:

```bash
ros2 run pose__detection ope
```

---

#### 🔹 yolo

Implements a **YOLOv5-based object detection node** that processes RGB images and publishes detections.
The node subscribes to `/camera/image_raw` and publishes annotated detections.
Copy this package into `src` and build it.

```bash
colcon build --packages-select yolo
```

Run YOLO node:

```bash
ros2 run yolo yolov5
```

🔗 YOLO Node Source: [yolo/yolov5.py](https://github.com/ArashdeepSinghMaan/Bin-Picking/blob/main/yolo/yolo/yolov5.py)

---

#### 🔹 mycobot_moveit_config_manual_setup

Contains **MoveIt2 configuration files** for motion planning, kinematics, and grasp execution.
Copy this folder into `src` and rebuild.

Launch MoveIt2:

```bash
ros2 launch mycobot_moveit_config_manual_setup moveit_planning_execution.launch.py
```

📙 MoveIt2 Documentation: [https://moveit.picknik.ai](https://moveit.picknik.ai)

---

#### 🔹 Intel RealSense Model

The **intel_realsense_r200** folder contains the **depth camera model** used in Gazebo.
To make the camera available globally, place it in your Gazebo models folder:

```bash
cp -r intel_realsense_r200 ~/.gazebo/models/
```

Then launch Gazebo — the camera will be available for spawning in the world.

---

### 🔹 Grasp Pose Detection (GPD)

For grasp planning, this project uses the **GPD** library:

> Detects 6-DOF grasp poses in 3D point clouds.

📘 Reference Repository: [https://github.com/atenpas/gpd](https://github.com/atenpas/gpd)

---

## ⚙️ Software Dependencies

### 🧬 ROS2 Packages

* `ros-humble-cv-bridge`
* `ros-humble-image-transport`
* `ros-humble-pcl-ros`
* `ros-humble-moveit`
* `gazebo-ros-pkgs`

### 🧉 Python Dependencies

```bash
torch
opencv-python
numpy
matplotlib
pclpy
rclpy
```

Install all dependencies:

```bash
pip install -r requirements.txt
```

---

## 🎯 Modules Summary

| Module                   | Function                                    | Topics/Subscribers                                     | Status         |
| ------------------------ | ------------------------------------------- | ------------------------------------------------------ | -------------- |
| **YOLOv5 Node**          | Object detection from RGB stream            | `/camera/image_raw` → `/object_detection/detections`   | ✅ Working      |
| **Pose Estimation Node** | 3D pose from depth map & RGB bounding boxes | `/camera/depth/points`, `/object_detection/detections` | ⚙️ In Progress |
| **GPD Node**             | Grasp point computation from PCL data       | `/pose_estimation/poses`                               | ⚙️ Planned     |
| **MoveIt2 Node**         | Pick-and-place trajectory planning          | `/grasping/grasp_points`                               | 🔄 Planned     |

---

## 🧪 Simulation Environment

### 🧱 Gazebo World Setup

* The world contains a **bin filled with objects** of varying sizes and shapes.
* The **MyCobot arm** and **Intel RealSense camera** are spawned using launch files.
* Point cloud and image data are visualized in **RViz2**.

<p align="center">
  <img src="docs/images/gazebo_robot.png" alt="Gazebo Bin Picking Setup" width="600">
</p>

---

## 🧮 Project Methodology

### **1️⃣ Object Detection**

* RGB camera feed processed through YOLOv5.
* Publishes bounding boxes and confidence scores.

### **2️⃣ Pose Estimation**

* Uses PCL to segment and map objects in 3D.
* Outputs 6-DOF pose: *(x, y, z, roll, pitch, yaw)*.

### **3️⃣ Grasp Pose Detection (GPD)**

* Generates stable grasp poses from point cloud data.
* Publishes grasp candidates for MoveIt2.

### **4️⃣ Motion Planning**

* MoveIt2 computes collision-free pick-and-place trajectories.
* Executes grasp and placement tasks.

---

## 📊 Current Progress

| Task                   | Description                                   | Status         |
| ---------------------- | --------------------------------------------- | -------------- |
| Simulation Environment | Gazebo world with robot and bin               | ✅ Done         |
| Camera Integration     | Intel RealSense RGB-D data publishing         | ✅ Done         |
| YOLOv5 Detection Node  | RGB image detection and visualization         | ✅ Done         |
| Pose Estimation Node   | Point cloud alignment and object localization | ⚙️ In Progress |
| GPD Integration        | Grasp point generation                        | 🔄 Planned     |
| MoveIt2 Pick & Place   | Motion planning execution                     | 🔄 Planned     |

---

## 📘 References

* [YOLOv5 Documentation](https://github.com/ultralytics/yolov5)
* [Point Cloud Library](https://pointclouds.org/)
* [MoveIt2 Documentation](https://moveit.picknik.ai/humble/index.html)
* [Grasp Pose Detection (GPD)](https://github.com/atenpas/gpd)
* [Intel RealSense SDK](https://github.com/IntelRealSense/librealsense)
* [ROS2 Tutorials](https://docs.ros.org/en/humble/index.html)

---

## 🧽 Future Work

* Combine **Object Detection** and **Pose Estimation** nodes for synchronized perception.
* Integrate **GPD** for real grasp detection and evaluation.
* Use **MoveIt Task Constructor** for advanced manipulation sequences.
* Optimize for **real-time performance on hardware (Jetson/PC)**.

---

## 🏁 Conclusion

This project demonstrates a modular and extensible approach to **robotic bin picking** using **ROS2**, **YOLOv5**, **PCL**, and **GPD**.
By combining computer vision with motion planning, it lays the groundwork for deploying efficient and reliable **autonomous manipulation systems** in industrial environments.

---

## 👨‍💻 Authors

* **Muskan Suman** – IIT Jodhpur
* **Arashdeep Singh** – IIT Jodhpur

📍 *Robotics and Mobility Systems, IDRP — Indian Institute of Technology Jodhpur*

---


