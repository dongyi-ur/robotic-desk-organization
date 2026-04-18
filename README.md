# robotic-desk-organization

## 1. 项目简介

桌面整理是服务机器人在真实环境中面临的一项典型挑战，主要难点在于物体种类繁多（小物件、平面刚体、可变形物体）以及操作目标多样（如收纳、堆叠）。本工作提出了一种**面向任务的桌面整理框架**，通过结合环境约束（如桌沿、书本边缘）和物体间接触，实现对异构物体的鲁棒操作。

我们设计了三类**环境辅助的操作基元**：

- **接触式抓取**：适用于小物件和薄纸；
- **推‑抓取策略**：利用桌沿或书本边缘抓取尺子等平面刚体；
- **撬‑抓取策略**：适用于书本等可变形物体。

结合视觉感知模块（YOLO + SAM2.1 + 点云处理）与任务规划器，系统能够在真实场景中完成从识别、抓取到规整放置的完整桌面整理流程。下图展示了整理任务的初始与目标状态示例（对应论文 Fig. 1）。

> ![Fig. 1: 桌面整理任务的初始状态与整理后状态](docs/fig1_initial_final.png)
> *图：桌面整理任务的初始状态（左）与整理后状态（右）*  
> （初始场景包含笔、橡皮、铅芯盒、直尺、三角尺、纸张、书本等；整理后小物件放入笔筒，尺子放入笔筒或置于书本旁，纸张和书本堆叠整齐）

## 2. 项目内容

此部分主要介绍不同功能包及其实现的功能：

**UR robot (UR5e)**
- **Universal_Robots_ROS_Driver:** UR robot driver meta-package.
- **fmauch_universal_robot:** UR robot description meta-package.

Tutorial: https://github.com/UniversalRobots/Universal_Robots_ROS_Driver

**Camera (Realsense D415)**
- **realsense-ros:** a package for using Intel RealSense cameras with ROS.
- **ddynamic_reconfigure:** a package that allows modifying parameters of a ROS node of the camera.

Tutorial: http://neutron.manoonpong.com/perception-vision-realsense-set-up-tutorial/

**Eye to hand calibration**
- **easy_handeye:** an automated, hardware-independent Hand-Eye Calibration package.
- **aruco_ros:** a software package and ROS wrappers of the Aruco Augmented Reality marker detector library.
- **vision_visp:** a package which provides visual servoing platform algorithms as ROS components. 

**Perception**
- **darknet_ros:** a ROS package developed for object detection in camera images.
- **ur_vision:** the perception algorithm for this shoe packaging task.

**Task plannning**
- **ur_demo:** demos about path planning using MoveIt.
- **ur_grasping:** the task panning algorithm for this shoe packaging task.