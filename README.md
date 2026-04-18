# robotic-desk-organization

## 1. 项目简介

书桌桌面整理是服务机器人在真实环境中面临的一项典型挑战，主要难点在于物体种类繁多（小物件、平面刚体、可变形物体）以及操作目标多样（如收纳、堆叠）。本工作提出了一种**面向任务的书桌桌面整理框架**，通过结合环境约束（如桌沿、书本边缘）和物体间接触，实现对异构物体的鲁棒操作。

我们设计了三类**环境辅助的操作基元**：

- **接触式抓取**：适用于小物件和薄纸；
- **推‑抓取策略**：利用桌沿或书本边缘抓取尺子等平面刚体；
- **撬‑抓取策略**：适用于书本等可变形物体。

基于视觉感知模块（YOLO + SAM2.1 + 点云处理），并结合以上操作原语与辅助原语，定义了任务规划器。机器人系统能够在真实场景中完成从识别、抓取到规整放置的完整桌面整理流程。下图展示了整理任务的初始与目标状态示例（对应论文 Fig. 1）。具体的方法介绍可以参考论文（链接见末尾）。

> ![Fig. 1: 桌面整理任务的初始状态与整理后状态](https://github.com/dongyi-ur/robotic-desk-organization/blob/main/Fig0.jpg)

> *图：桌面整理任务的初始状态（左）与整理后状态（右）*  

> （初始场景包含笔、橡皮、铅芯盒、直尺、三角尺、纸张、书本等；整理后小物件放入笔筒，尺子放入笔筒或置于书本旁，纸张和书本堆叠整齐）

## 2. 项目内容

此部分主要介绍不同功能包及其实现的功能，主要包括机器人、夹爪、相机的ROS通讯控制功能包，和手眼标定功能包，以及针对本文任务专门开发的视觉功能包和操作规划功能包。

**UR robot (UR5e)**
- **Universal_Robots_ROS_Driver:** UR robot driver meta-package.
- **fmauch_universal_robot:** UR robot description meta-package.

Tutorial: https://github.com/UniversalRobots/Universal_Robots_ROS_Driver

**Gripper (Rochu)**
- **serial_msgs:** 夹爪通讯功能包.

**Camera (Realsense D415)**
- **realsense-ros:** a package for using Intel RealSense cameras with ROS.
- **ddynamic_reconfigure:** a package that allows modifying parameters of a ROS node of the camera.

Tutorial: http://neutron.manoonpong.com/perception-vision-realsense-set-up-tutorial/

**Eye to hand calibration**
- **easy_handeye:** an automated, hardware-independent Hand-Eye Calibration package.
- **aruco_ros:** a software package and ROS wrappers of the Aruco Augmented Reality marker detector library.
- **vision_visp:** a package which provides visual servoing platform algorithms as ROS components. 

**Perception**
- **object_keypoint_msgs:** 不同类型物体的视觉信息传输格式.
- **ultralytics_ros:** 包括物体位姿和关键点计算、环境约束（如桌面边缘）感知节点.

**Task plannning**
- **ur_smach:** 基于多原语的书桌桌面整理规划器.

## 3. 项目实施

首先开通相机，并运行视觉算法

```
$ cd ultralytics_ws/
$ source devel/setup.bash

# 启动相机
$ roslaunch realsense2_camera rs_camera.launch align_depth:=true enable_pointcloud:=true

# 物体的位姿和关键点获取
$ rosrun ultralytics_ros yolo_ros_node1.py

# 环境约束检测（桌面边缘）
$ export PYTHONPATH="/home/dongyi/anaconda3/envs/yolo_ros/lib/python3.8/site-packages:$PYTHONPATH"
$ rosrun ultralytics_ros desktop_detection_node.py

```

然后开启夹爪和机械臂通讯接口，发布手眼标定结果，并运行任务规划器

```
$ cd ur_ws_organize/
$ source devel/setup.bash

# 夹爪通讯
$ roslaunch serial_msgs gripper_control.launch 

# 机器人通讯
$ roslaunch ur_robot_driver ur5e_work_all.launch

# 发布手眼标定结果
$ roslaunch easy_handeye publish.launch

# 任务规划器
$ rosrun ur_smach TaskPlanner.py

# 开始运行
$ rostopic pub /tidy_task_command std_msgs/String "start"

```

## 4. 参考资料

1）视频：https://youtu.be/48cGp702p5k
2）论文：ArXiv链接

如何有关于项目的问题，欢迎咨询邮箱：dongyi@nuaa.edu.cn

