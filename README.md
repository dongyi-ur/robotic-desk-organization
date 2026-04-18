# robotic-desk-organization

## 1. Project Overview

Desktop organization is a representative and challenging task for service robots operating in real-world environments. The main difficulties arise from the diversity of objects (e.g., small items, planar rigid objects, and deformable objects) and the variety of task objectives (e.g., sorting, storing, and stacking).

In this project, we propose a **task-oriented framework for robotic desktop organization**, which leverages environmental constraints (e.g., table edges, book boundaries) and inter-object interactions to enable robust manipulation of heterogeneous objects.

We design three types of **environment-assisted manipulation primitives**:

- **Contact-based grasping**: suitable for small objects and thin paper;
- **Push-to-grasp strategy**: utilizes table edges or book boundaries to grasp planar rigid objects such as rulers;
- **Pry-to-grasp strategy**: designed for deformable objects such as books.

Based on a perception module (YOLO + SAM2.1 + point cloud processing), combined with the above manipulation primitives and auxiliary actions, we develop a task planner. The robotic system is capable of completing the full pipeline of desktop organization in real-world scenarios, including object detection, grasping, and orderly placement.

The figure below illustrates an example of the initial and goal states of the organization task (corresponding to Fig. 1 in the paper). For more details, please refer to the paper (link provided at the end).

> ![Fig. 1: Initial and goal states of the desktop organization task](https://github.com/dongyi-ur/robotic-desk-organization/blob/main/Fig0.jpg)
> *Figure: Initial state (left) and organized state (right) of the desktop organization task.*  
> *(The initial scene includes pens, erasers, lead cases, rulers, set squares, paper, books, etc. After organization, small items are placed in a pen holder, rulers are inserted into the holder or placed next to books, and paper and books are neatly stacked.)*

---

## 2. Repository Structure

This section introduces the main functional packages included in this project. It covers ROS communication packages for the robot, gripper, and camera, as well as hand–eye calibration, perception, and task planning modules specifically developed for this work.

### **UR Robot (UR5e)**

- **Universal_Robots_ROS_Driver**: Meta-package for controlling the UR robot.
- **fmauch_universal_robot**: Meta-package containing the UR robot description.

Tutorial: https://github.com/UniversalRobots/Universal_Robots_ROS_Driver

### **Gripper (Rochu)**

- **serial_msgs**: Communication package for controlling the gripper.

### **Camera (Intel RealSense D415)**

- **realsense-ros**: ROS package for Intel RealSense cameras.
- **ddynamic_reconfigure**: Allows dynamic parameter tuning for camera nodes.

Tutorial: http://neutron.manoonpong.com/perception-vision-realsense-set-up-tutorial/

### **Eye-to-Hand Calibration**

- **easy_handeye**: Automated, hardware-independent hand–eye calibration package.
- **aruco_ros**: ROS wrapper for ArUco marker detection.
- **vision_visp**: Provides visual servoing algorithms as ROS components.

### **Perception**

- **object_keypoint_msgs**: Message definitions for visual information of different object types.
- **ultralytics_ros**: Includes object pose estimation, keypoint detection, and environment constraint perception (e.g., table edges).

### **Task Planning**

- **ur_smach**: A multi-primitive-based task planner for desktop organization.

---

## 3. Usage

First, start the camera and run the perception modules:

```bash
$ cd ultralytics_ws/
$ source devel/setup.bash

# Launch the camera
$ roslaunch realsense2_camera rs_camera.launch align_depth:=true enable_pointcloud:=true

# Object pose and keypoint detection
$ rosrun ultralytics_ros yolo_ros_node1.py

# Environment constraint detection (e.g., table edges)
$ export PYTHONPATH="/home/dongyi/anaconda3/envs/yolo_ros/lib/python3.8/site-packages:$PYTHONPATH"
$ rosrun ultralytics_ros desktop_detection_node.py

```

Then, start the gripper and robot communication interfaces, publish the hand–eye calibration results, and run the task planner:

```bash
$ cd ur_ws_organize/
$ source devel/setup.bash

# Gripper communication
$ roslaunch serial_msgs gripper_control.launch 

# Robot driver
$ roslaunch ur_robot_driver ur5e_work_all.launch

# Publish hand–eye calibration results
$ roslaunch easy_handeye publish.launch

# Task planner
$ rosrun ur_smach TaskPlanner.py

# Start the task
$ rostopic pub /tidy_task_command std_msgs/String "start"

```

---

## 4. Resources

- Video：https://youtu.be/48cGp702p5k
- Paper：ArXiv link (to be added)

---

## 5. Contact

If you have any questions about this project, feel free to contact:

📧 dongyi@nuaa.edu.cn