# robotic-desk-organization

## 1. Project Overview

Desktop organization is a representative and challenging task for service robots operating in real-world environments. The main difficulties arise from the diversity of objects (e.g., small items, thin rigid objects, and deformable objects) and the variety of task objectives (e.g., sorting, storing, and stacking).

In this project, we propose a **task-oriented framework for robotic desktop organization**, which leverages environmental constraints (e.g., table edges, book edges) and inter-object interactions to enable robust manipulation of heterogeneous objects.

We design three types of **environment-assisted manipulation primitives**:

- **Contact grasping**: suitable for small objects and thin paper;
- **Push grasping**: utilizes table or book edges to grasp planar rigid objects such as rulers;
- **Pry grasping**: designed for planar deformable objects such as books.

Based on a perception module (YOLO + SAM2.1 + point cloud processing), combined with the above manipulation primitives and auxiliary actions, we develop a task planner. The robotic system is capable of completing the full pipeline of desktop organization in real-world scenarios, including object detection, grasping, and orderly placement.

The figure below illustrates an example of the initial and goal states of the organization task (corresponding to Fig. 1 in the paper). For more details, please refer to the paper (link provided at the end).

> ![Fig. 1: Initial and goal states of the desktop organization task](https://github.com/manipulation20/robotic-desk-organization/blob/main/Fig0.jpg)
> *Figure: Initial state (left) and organized state (right) of the desktop organization task.*  
> *(The initial scene includes pens, erasers, lead cases, rulers, set squares, paper, books, etc. After organization, small items are placed in a pen holder, rulers are inserted into the holder, and paper and books are neatly stacked.)*

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

## 4. Experimental Scenarios

To systematically evaluate the proposed framework under realistic desktop organization conditions, we designed **36 distinct experimental scenarios**. These scenarios vary in the number of object categories, spatial arrangements, and inter-object relationships, covering representative clutter patterns commonly encountered in office and study environments.

### Naming Convention

Each scenario is labeled as **`Cxyz`**, where:

- **`x`** = number of object categories involved (range: 2–5)  
- **`y`** = combination index within that category count  
- **`z`** = layout instance index (1–3)  

**Example:** `C311` denotes the first layout instance of the first combination involving three object categories.

### Object Categories

The following object types are considered, consistent with Section III of the paper:

| Category | Included Objects |
|----------|------------------|
| Small objects | Pens, erasers, lead cases |
| Thin rigid objects | Straight rulers, 30° triangular rulers, 45° triangular rulers |
| Planar deformable objects | Paper sheets (60–80 GSM), books (various thicknesses) |

Each scenario includes **2 to 5 categories**, with objects initialized at arbitrary poses and occasional overlaps (e.g., rulers placed on paper or books).

### Full Set of 36 Scenarios

The figure below presents all 36 experimental scenarios used in the full framework evaluation (Section IV-E). Each subfigure illustrates the initial configuration of a desktop organization task. The scenarios are arranged in order of increasing complexity (from `C2xx` to `C5xx`). For each scenario, the robot is required to sort small items into a pen holder, arrange rulers neatly, and stack paper and books, following the same goal state definition as in Fig. 1.

![36 experimental scenarios for desktop organization](https://github.com/manipulation20/robotic-desk-organization/blob/main/Fig1.png)

*Figure: All 36 experimental scenarios evaluated in Section IV-E. Each subfigure is labeled as `Cxyz`, where `x` denotes the number of object categories (2–5), `y` the combination index, and `z` the layout instance (1–3). The scenes include various combinations of pens, erasers, lead cases, rulers (straight and triangular), paper sheets, and books. Objects may be placed directly on the table, partially overlapping, or stacked on other objects (e.g., rulers on books). These scenarios capture typical desk clutter and are used to evaluate the success rate and robustness of the proposed multi-primitive task planner.*

---

## 5. Resources

- Video：https://youtu.be/48cGp702p5k
- Paper：ArXiv link (to be added)

---

## 6. Contact

If you have any questions about this project, feel free to contact:

📧 xxx
