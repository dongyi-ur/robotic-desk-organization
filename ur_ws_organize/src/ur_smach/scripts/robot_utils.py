#!/usr/bin/python

import numpy
import rospy, sys
# import smach
# import smach_ros
# from smach import CBState, State
# from smach import State

# from turtlesim.msg import Pose
# from geometry_msgs import Pose
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import String
import moveit_commander
from moveit_commander import MoveGroupCommander

from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from copy import deepcopy
import tf
import tf2_ros
import tf2_geometry_msgs  # **Do not use geometry_msgs. Use this instead for PoseStamped
# sys.path.append('/usr/lib/python3/dist-packages') # pykdl

import math
from math import cos, sin
from tf.transformations import quaternion_from_euler

from threading import Thread

_tf_initialized = False
_tfBuffer = None

def get_tf_buffer():
    global _tf_initialized, _tfBuffer
    if not _tf_initialized:
        if not rospy.core.is_initialized():
            raise RuntimeError("ROS not initialized. Call rospy.init_node() first")
        
        rospy.loginfo("Initializing TF system...")
        _tfBuffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        tf2_ros.TransformListener(_tfBuffer)
        _tf_initialized = True
        rospy.loginfo("TF system ready")
    return _tfBuffer

# pose3d_camera ##################################################################################################################

def pose3d(center3d, object_angle):

    center_pose = PoseStamped()
    center_pose.header.frame_id = "camera_color_frame"
    center_pose.header.stamp = rospy.Time.now()
    center_pose.pose.position = center3d
    
    roll = 0
    pitch = 0
    yaw = math.radians(object_angle) # -90?
    q = quaternion_from_euler(roll, pitch, yaw)

    # center_pose.pose.orientation.x = q[0]
    # center_pose.pose.orientation.y = q[1]
    # center_pose.pose.orientation.z = q[2]
    # center_pose.pose.orientation.w = q[3]
    center_pose.pose.orientation = Quaternion(*q)

    return center_pose

# setTargetRotation: control the place pose of long object (such as pen) ##################################################################################################################

def setTargetRotation(pose):

    pose0 = tf2_geometry_msgs.PoseStamped()
    # realEnd.pose = msg.pose
    pose0.header.frame_id = "camera_color_frame"
    pose0.header.stamp = rospy.Time.now()
    pose0.pose = pose.pose
    # pose0.pose.position.z = paper_pose.pose.position.z + 0.003

    pose1 = tf2_geometry_msgs.PoseStamped()
    # realEnd.pose = msg.pose
    pose1.header.frame_id = "camera_color_frame"
    pose1.header.stamp = rospy.Time.now()

    pose0_to_camera = tf.transformations.quaternion_matrix([pose0.pose.orientation.x, pose0.pose.orientation.y, pose0.pose.orientation.z, pose0.pose.orientation.w]) # the order of xyzw
    pose0_to_camera[:, 3] = [pose0.pose.position.x, pose0.pose.position.y, pose0.pose.position.z, 1]

    # roll1 = math.radians(35)
    # for single shoe
    # roll1 = sign * math.acos((0.22-width_height[0])/0.22)
    # for a pair of shoes
    roll0 = 0 # angle # math.pi/6
    pitch0 = -80*math.pi/180 # -math.pi/2
    yaw0 = 0
    q = quaternion_from_euler(roll0, pitch0, yaw0)

    pose1_to_pose0 = tf.transformations.quaternion_matrix(q) # the order of xyzw
    pose1_to_pose0 [:, 3] = [0, 0, 0, 1]

    pose1_to_camera = numpy.dot(pose0_to_camera, pose1_to_pose0)

    pose1.pose.position.x = pose1_to_camera[0, 3]
    pose1.pose.position.y = pose1_to_camera[1, 3]
    pose1.pose.position.z = pose1_to_camera[2, 3]

    q = tf.transformations.quaternion_from_matrix(pose1_to_camera)
    pose1.pose.orientation.x = q[0]
    pose1.pose.orientation.y = q[1]
    pose1.pose.orientation.z = q[2]
    pose1.pose.orientation.w = q[3]

    return pose1

# rotationOpen: control the fingertip **************************************************************************************************************#

def rotaionOpen(pose, angle, rotationOpen_directi):

    pose0 = tf2_geometry_msgs.PoseStamped()
    # realEnd.pose = msg.pose
    pose0.header.frame_id = "camera_color_frame"
    
    pose0.header.stamp = rospy.Time.now()
    pose0.pose = pose.pose
    # pose0.pose.position.z = paper_pose.pose.position.z + 0.003

    pose1 = tf2_geometry_msgs.PoseStamped()
    # realEnd.pose = msg.pose
    pose1.header.frame_id = "camera_color_frame"
    pose1.header.stamp = rospy.Time.now()

    pose0_to_camera = tf.transformations.quaternion_matrix([pose0.pose.orientation.x, pose0.pose.orientation.y, pose0.pose.orientation.z, pose0.pose.orientation.w]) # the order of xyzw
    pose0_to_camera[:, 3] = [pose0.pose.position.x, pose0.pose.position.y, pose0.pose.position.z, 1]

    # roll1 = math.radians(35)
    # for single shoe
    # roll1 = sign * math.acos((0.22-width_height[0])/0.22)
    # for a pair of shoes
    if rotationOpen_directi:
        roll0 = angle # math.pi/6 
    else:
        roll0 = -angle # math.pi/6
    # roll0 = 5*math.pi/180 # math.pi/6
    pitch0 = 0
    yaw0 = 0
    q = quaternion_from_euler(roll0, pitch0, yaw0)

    pose1_to_pose0 = tf.transformations.quaternion_matrix(q) # the order of xyzw
    pose1_to_pose0 [:, 3] = [0, 0, 0, 1]

    pose1_to_camera = numpy.dot(pose0_to_camera, pose1_to_pose0)

    pose1.pose.position.x = pose1_to_camera[0, 3]
    pose1.pose.position.y = pose1_to_camera[1, 3]
    pose1.pose.position.z = pose1_to_camera[2, 3]

    q = tf.transformations.quaternion_from_matrix(pose1_to_camera)
    pose1.pose.orientation.x = q[0]
    pose1.pose.orientation.y = q[1]
    pose1.pose.orientation.z = q[2]
    pose1.pose.orientation.w = q[3]

    pose2 = tf2_geometry_msgs.PoseStamped()
    # realEnd.pose = msg.pose
    pose2.header.frame_id = "camera_color_frame"
    pose2.header.stamp = rospy.Time.now()

    pose1_to_camera = tf.transformations.quaternion_matrix([pose1.pose.orientation.x, pose1.pose.orientation.y, pose1.pose.orientation.z, pose1.pose.orientation.w]) # the order of xyzw
    pose1_to_camera[:, 3] = [pose1.pose.position.x, pose1.pose.position.y, pose1.pose.position.z, 1]

    # roll = math.radians(25)
    roll1 = 0
    pitch1 = 0
    yaw1 = 0
    q = quaternion_from_euler(roll1, pitch1, yaw1)

    pose2_to_pose1 = tf.transformations.quaternion_matrix(q) # the order of xyzw
    if rotationOpen_directi:
        pose2_to_pose1 [:, 3] = [0, -0.0473, 0.0085, 1]
    else:
        pose2_to_pose1 [:, 3] = [0, +0.0473, 0.0085, 1]
    # pose2_to_pose1 [:, 3] = [0, -0.0473, 0.0085, 1] # z = 0.0074 (-70kPa) !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    pose2_to_camera = numpy.dot(pose1_to_camera, pose2_to_pose1)

    pose2.pose.position.x = pose2_to_camera[0, 3]
    pose2.pose.position.y = pose2_to_camera[1, 3]
    pose2.pose.position.z = pose2_to_camera[2, 3]

    q = tf.transformations.quaternion_from_matrix(pose2_to_camera)
    pose2.pose.orientation.x = q[0]
    pose2.pose.orientation.y = q[1]
    pose2.pose.orientation.z = q[2]
    pose2.pose.orientation.w = q[3]

    return pose2

# rotationOpen: control the fingertip **************************************************************************************************************#
### Especially for the graspPose of Ruler/Triangle primitive. Difference is opposite rotation angle.

def rotaionOpenGrasp1(pose, angle, rotationOpen_directi):

    pose0 = tf2_geometry_msgs.PoseStamped()
    # realEnd.pose = msg.pose
    pose0.header.frame_id = "camera_color_frame"
    
    pose0.header.stamp = rospy.Time.now()
    pose0.pose = pose.pose
    pose0.pose.position.z += 0.005

    pose1 = tf2_geometry_msgs.PoseStamped()
    # realEnd.pose = msg.pose
    pose1.header.frame_id = "camera_color_frame"
    pose1.header.stamp = rospy.Time.now()

    pose0_to_camera = tf.transformations.quaternion_matrix([pose0.pose.orientation.x, pose0.pose.orientation.y, pose0.pose.orientation.z, pose0.pose.orientation.w]) # the order of xyzw
    pose0_to_camera[:, 3] = [pose0.pose.position.x, pose0.pose.position.y, pose0.pose.position.z, 1]

    # roll1 = math.radians(35)
    # for single shoe
    # roll1 = sign * math.acos((0.22-width_height[0])/0.22)
    # for a pair of shoes
    if rotationOpen_directi:
        roll0 = -angle # math.pi/6 
    else:
        roll0 = angle # math.pi/6
    # roll0 = 5*math.pi/180 # math.pi/6
    pitch0 = 0
    yaw0 = 0
    q = quaternion_from_euler(roll0, pitch0, yaw0)

    pose1_to_pose0 = tf.transformations.quaternion_matrix(q) # the order of xyzw
    pose1_to_pose0 [:, 3] = [0, 0, 0, 1]

    pose1_to_camera = numpy.dot(pose0_to_camera, pose1_to_pose0)

    pose1.pose.position.x = pose1_to_camera[0, 3]
    pose1.pose.position.y = pose1_to_camera[1, 3]
    pose1.pose.position.z = pose1_to_camera[2, 3]

    q = tf.transformations.quaternion_from_matrix(pose1_to_camera)
    pose1.pose.orientation.x = q[0]
    pose1.pose.orientation.y = q[1]
    pose1.pose.orientation.z = q[2]
    pose1.pose.orientation.w = q[3]

    # pose2 = tf2_geometry_msgs.PoseStamped()
    # # realEnd.pose = msg.pose
    # pose2.header.frame_id = "camera_color_frame"
    # pose2.header.stamp = rospy.Time.now()

    # pose1_to_camera = tf.transformations.quaternion_matrix([pose1.pose.orientation.x, pose1.pose.orientation.y, pose1.pose.orientation.z, pose1.pose.orientation.w]) # the order of xyzw
    # pose1_to_camera[:, 3] = [pose1.pose.position.x, pose1.pose.position.y, pose1.pose.position.z, 1]

    # # roll = math.radians(25)
    # roll1 = 0
    # pitch1 = 0
    # yaw1 = 0
    # q = quaternion_from_euler(roll1, pitch1, yaw1)

    # pose2_to_pose1 = tf.transformations.quaternion_matrix(q) # the order of xyzw
    # if rotationOpen_directi:
    #     pose2_to_pose1 [:, 3] = [0, -0.0473, 0.0085, 1]
    # else:
    #     pose2_to_pose1 [:, 3] = [0, +0.0473, 0.0085, 1]
    # # pose2_to_pose1 [:, 3] = [0, -0.0473, 0.0085, 1] # z = 0.0074 (-70kPa) !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    # pose2_to_camera = numpy.dot(pose1_to_camera, pose2_to_pose1)

    # pose2.pose.position.x = pose2_to_camera[0, 3]
    # pose2.pose.position.y = pose2_to_camera[1, 3]
    # pose2.pose.position.z = pose2_to_camera[2, 3]

    # q = tf.transformations.quaternion_from_matrix(pose2_to_camera)
    # pose2.pose.orientation.x = q[0]
    # pose2.pose.orientation.y = q[1]
    # pose2.pose.orientation.z = q[2]
    # pose2.pose.orientation.w = q[3]

    return pose1

def rotaionOpenGrasp2(pose, angle, rotationOpen_directi): # connect with graspPose0 better from the fingertip, real performance just so so 
    ### Especially for the graspPose of Ruler/Triangle primitive. 

    pose0 = tf2_geometry_msgs.PoseStamped()
    # realEnd.pose = msg.pose
    pose0.header.frame_id = "camera_color_frame"
    
    pose0.header.stamp = rospy.Time.now()
    pose0.pose = pose.pose
    # pose0.pose.position.z = paper_pose.pose.position.z + 0.003

    pose1 = tf2_geometry_msgs.PoseStamped()
    # realEnd.pose = msg.pose
    pose1.header.frame_id = "camera_color_frame"
    pose1.header.stamp = rospy.Time.now()

    pose0_to_camera = tf.transformations.quaternion_matrix([pose0.pose.orientation.x, pose0.pose.orientation.y, pose0.pose.orientation.z, pose0.pose.orientation.w]) # the order of xyzw
    pose0_to_camera[:, 3] = [pose0.pose.position.x, pose0.pose.position.y, pose0.pose.position.z, 1]

    # roll1 = math.radians(35)
    # for single shoe
    # roll1 = sign * math.acos((0.22-width_height[0])/0.22)
    # for a pair of shoes
    roll0 = 0
    pitch0 = 0
    yaw0 = 0
    q = quaternion_from_euler(roll0, pitch0, yaw0)

    pose1_to_pose0 = tf.transformations.quaternion_matrix(q) # the order of xyzw
    # pose1_to_pose0 [:, 3] = [0, 0, 0, 1]
    if rotationOpen_directi:
        pose1_to_pose0 [:, 3] = [0, +0.0473, 0, 1]
    else:
        pose1_to_pose0 [:, 3] = [0, -0.0473, 0, 1]

    pose1_to_camera = numpy.dot(pose0_to_camera, pose1_to_pose0)

    pose1.pose.position.x = pose1_to_camera[0, 3]
    pose1.pose.position.y = pose1_to_camera[1, 3]
    pose1.pose.position.z = pose1_to_camera[2, 3]

    q = tf.transformations.quaternion_from_matrix(pose1_to_camera)
    pose1.pose.orientation.x = q[0]
    pose1.pose.orientation.y = q[1]
    pose1.pose.orientation.z = q[2]
    pose1.pose.orientation.w = q[3]

    pose2 = tf2_geometry_msgs.PoseStamped()
    # realEnd.pose = msg.pose
    pose2.header.frame_id = "camera_color_frame"
    pose2.header.stamp = rospy.Time.now()

    pose1_to_camera = tf.transformations.quaternion_matrix([pose1.pose.orientation.x, pose1.pose.orientation.y, pose1.pose.orientation.z, pose1.pose.orientation.w]) # the order of xyzw
    pose1_to_camera[:, 3] = [pose1.pose.position.x, pose1.pose.position.y, pose1.pose.position.z, 1]

    # roll = math.radians(25)
    if rotationOpen_directi:
        roll1 = -angle # math.pi/6 
    else:
        roll1 = angle # math.pi/6
    pitch1 = 0
    yaw1 = 0
    q = quaternion_from_euler(roll1, pitch1, yaw1)

    pose2_to_pose1 = tf.transformations.quaternion_matrix(q) # the order of xyzw
    if rotationOpen_directi:
        pose2_to_pose1 [:, 3] = [0, -0.0473, 0.0085, 1]
    else:
        pose2_to_pose1 [:, 3] = [0, +0.0473, 0.0085, 1]
    # pose2_to_pose1 [:, 3] = [0, -0.0473, 0.0085, 1] # z = 0.0074 (-70kPa) !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    pose2_to_camera = numpy.dot(pose1_to_camera, pose2_to_pose1)

    pose2.pose.position.x = pose2_to_camera[0, 3]
    pose2.pose.position.y = pose2_to_camera[1, 3]
    pose2.pose.position.z = pose2_to_camera[2, 3]

    q = tf.transformations.quaternion_from_matrix(pose2_to_camera)
    pose2.pose.orientation.x = q[0]
    pose2.pose.orientation.y = q[1]
    pose2.pose.orientation.z = q[2]
    pose2.pose.orientation.w = q[3]

    return pose2

# rotationClose: for pry action**************************************************************************************************************#

def rotaionClose(pose, angle):

    pose0 = tf2_geometry_msgs.PoseStamped()
    pose0.header.frame_id = "camera_color_frame"
    pose0.header.stamp = rospy.Time.now()
    pose0.pose = pose.pose
    # pose0.pose.position.z = paper_pose.pose.position.z + 0.003

    pose1 = tf2_geometry_msgs.PoseStamped()
    pose1.header.frame_id = "camera_color_frame"
    pose1.header.stamp = rospy.Time.now()

    pose0_to_camera = tf.transformations.quaternion_matrix([pose0.pose.orientation.x, pose0.pose.orientation.y, pose0.pose.orientation.z, pose0.pose.orientation.w]) # the order of xyzw
    pose0_to_camera[:, 3] = [pose0.pose.position.x, pose0.pose.position.y, pose0.pose.position.z, 1]

    # if rotationOpen_directi:
    #     roll0 = angle # math.pi/6 
    # else:
    #     roll0 = -angle # math.pi/6
    roll0 = angle # math.pi/6
    pitch0 = 0
    yaw0 = 0
    q = quaternion_from_euler(roll0, pitch0, yaw0)

    pose1_to_pose0 = tf.transformations.quaternion_matrix(q) # the order of xyzw
    pose1_to_pose0 [:, 3] = [0, 0, 0, 1]

    pose1_to_camera = numpy.dot(pose0_to_camera, pose1_to_pose0)

    pose1.pose.position.x = pose1_to_camera[0, 3]
    pose1.pose.position.y = pose1_to_camera[1, 3]
    pose1.pose.position.z = pose1_to_camera[2, 3]

    q = tf.transformations.quaternion_from_matrix(pose1_to_camera)
    pose1.pose.orientation.x = q[0]
    pose1.pose.orientation.y = q[1]
    pose1.pose.orientation.z = q[2]
    pose1.pose.orientation.w = q[3]

    return pose1

# transformCoor ##################################################################################################################

# from camera_color_frame to base_link frame
def transformCoor(msg, max_retries=3):
    tfBuffer = get_tf_buffer()  # 获取或初始化Buffer

    for attempt in range(max_retries):
        try:
            # 使用最新可用变换（不指定特定时间）
            transform = tfBuffer.lookup_transform(
                "base_link", 
                "camera_color_frame",
                rospy.Time(0),  # 最新可用时间
                rospy.Duration(1.0)
            )
            
            # 应用变换到目标坐标系
            pose_in_camera = PoseStamped()
            pose_in_camera.pose = msg.pose
            pose_in_camera.header.frame_id = "camera_color_frame"
            pose_in_camera.header.stamp = rospy.Time.now()
            
            # 使用transform进行坐标变换
            pose_in_base = tf2_geometry_msgs.do_transform_pose(pose_in_camera, transform)
            return pose_in_base.pose
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"TF转换尝试 {attempt+1}/{max_retries} 失败: {str(e)}")
            rospy.sleep(0.1)  # 短暂等待后重试
    
    rospy.logerr("坐标变换失败，达到最大重试次数")
    return None

# transform_end ##################################################################################################################

# trasfer the object coordinates to moveit control coordinates (ee_link)
def transform_end(pos):

    # eelink_to_robot, object_to_robot, toolend_to_object, toolend_to_eelink
    
    realEnd = Pose()
    # print("tuble or list is ok", pos.orientation, pos.position) # or tset it in visionInterface.py

    # object_to_robot = tf.transformations.quaternion_matrix((1, 1, 0, 0))
    object_to_robot = tf.transformations.quaternion_matrix([pos.orientation.x, pos.orientation.y, pos.orientation.z, pos.orientation.w]) # the order of xyzw
    object_to_robot[:, 3] = [pos.position.x, pos.position.y, pos.position.z, 1]

    toolend_to_object = tf.transformations.quaternion_matrix([0, 0, 0, 1]) # the order of xyzw
    toolend_to_object[:, 3] = [0, 0, 0, 1]

    toolend_to_eelink = tf.transformations.quaternion_matrix([0, 0, 0, 1]) # the order of xyzw
    toolend_to_eelink[:, 3] = [0.0, 0.0, 0.22, 1]

    eelink_to_robot = numpy.dot( numpy.dot(object_to_robot, toolend_to_object), numpy.linalg.inv(toolend_to_eelink))

    realEnd.position.x = eelink_to_robot[0, 3]
    realEnd.position.y = eelink_to_robot[1, 3]
    realEnd.position.z = eelink_to_robot[2, 3]

    q = tf.transformations.quaternion_from_matrix(eelink_to_robot)
    realEnd.orientation.x = q[0]
    realEnd.orientation.y = q[1]
    realEnd.orientation.z = q[2]
    realEnd.orientation.w = q[3]

    return realEnd

    # print("toolend_to_object:", toolend_to_object)
    # print("object_to_robot:", object_to_robot)
    # print("inv:", numpy.linalg.inv(toolend_to_eelink))
    # print("realEnd:", realEnd)

# straightPath ##################################################################################################################

def straightPath (waypoints):
    
    # 初始化move_group的API
    moveit_commander.roscpp_initialize(sys.argv)
                    
    # 初始化需要使用move group控制的机械臂中的arm group
    arm = MoveGroupCommander('manipulator')
    
    # 当运动规划失败后，允许重新规划
    arm.allow_replanning(True)
    
    # 设置目标位置所使用的参考坐标系
    arm.set_pose_reference_frame('base_link')
            
    # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
    arm.set_goal_position_tolerance(0.001)
    arm.set_goal_orientation_tolerance(0.1)
    
    # 设置允许的最大速度和加速度
    arm.set_max_acceleration_scaling_factor(0.5)
    arm.set_max_velocity_scaling_factor(0.5)
    
    '''  
    # 获取终端link的名称
    end_effector_link = arm.get_end_effector_link()

    # 控制机械臂先回到初始化位置
    arm.set_named_target('home')
    arm.go()
    rospy.sleep(1)
                                            
    # 获取当前位姿数据最为机械臂运动的起始位姿
    start_pose = arm.get_current_pose(end_effector_link).pose
    
    # 初始化路点列表
    waypoints = []
            
    # 将初始位姿加入路点列表
    waypoints.append(start_pose)
        
    # 设置路点数据，并加入路点列表
    wpose = deepcopy(start_pose)
    wpose.position.z -= 0.2
    waypoints.append(deepcopy(wpose))

    wpose.position.x += 0.1
    waypoints.append(deepcopy(wpose))
    
    wpose.position.y += 0.1
    waypoints.append(deepcopy(wpose))
    '''    

    fraction = 0.0   #路径规划覆盖率
    maxtries = 100   #最大尝试规划次数
    attempts = 0     #已经尝试规划次数
    
    # 设置机器臂当前的状态作为运动初始状态
    arm.set_start_state_to_current_state()

    # 尝试规划一条笛卡尔空间下的路径，依次通过所有路点
    while fraction < 1.0 and attempts < maxtries:
        (plan, fraction) = arm.compute_cartesian_path (
                                waypoints,   # waypoint poses，路点列表
                                0.01,        # eef_step，终端步进值
                                0.0,         # jump_threshold，跳跃阈值
                                True)        # avoid_collisions，避障规划
        
        # 尝试次数累加
        attempts += 1
        
        # 打印运动规划进程
        if attempts % 10 == 0:
            rospy.loginfo("Still trying after " + str(attempts) + " attempts...")
                    
    # 如果路径规划成功（覆盖率100%）,则开始控制机械臂运动
    if fraction == 1.0:
        rospy.loginfo("Path computed successfully. Moving the arm.")
        arm.execute(plan)
        rospy.loginfo("Path execution complete.")
    # 如果路径规划失败，则打印失败信息
    else:
        rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")  

    rospy.sleep(1)

    # 关闭并退出moveit
    # moveit_commander.roscpp_shutdown()
    # moveit_commander.os._exit(0)

# goHome ##################################################################################################################

def goHome():
    # 初始化需要使用move group控制的机械臂中的arm group
    arm = MoveGroupCommander('manipulator')

    # 获取终端link的名称
    end_effector_link = arm.get_end_effector_link()
                                            
    # 获取当前位姿数据最为机械臂运动的起始位姿
    start_pose = arm.get_current_pose(end_effector_link).pose

    print ("link_name:", end_effector_link) # answer is "tool0"
    print ("end_effector_link:", start_pose)
    
    # 初始化路点列表
    waypoints = []
            
    # 将初始位姿加入路点列表
    waypoints.append(start_pose)

    # 设置路点数据，并加入路点列表
    wpose = deepcopy(start_pose)
    wpose.position.x = -0.1
    wpose.position.y =  0.49
    wpose.position.z =  0.30 # 0.45

    waypoints.append(deepcopy(wpose))

    straightPath(waypoints)

# go_graspHome ##################################################################################################################

def go_graspHome():

    # 初始化需要使用move group控制的机械臂中的arm group
    arm = MoveGroupCommander('manipulator')
    
    # 设置目标位置所使用的参考坐标系
    arm.set_pose_reference_frame('base_link')
    
    # 获取终端link的名称
    end_effector_link = arm.get_end_effector_link()

    # print("end_effector_link name:", end_effector_link)
                                            
    # 获取当前位姿数据最为机械臂运动的起始位姿
    start_pose = arm.get_current_pose(end_effector_link).pose
    # start_pose = arm.get_current_pose().pose

    # print("graspHome:", start_pose)
    
    # 初始化路点列表
    waypoints = []
            
    # 将初始位姿加入路点列表
    waypoints.append(start_pose)

    # 设置路点数据，并加入路点列表
    wpose = deepcopy(start_pose)
    wpose.position.x =  0.48
    wpose.position.y =  0.10
    wpose.position.z =  0.30 # 0.45
    
    wpose.orientation.x = -0.599635
    wpose.orientation.y =  0.800275
    wpose.orientation.z =  0.001545
    wpose.orientation.w =  0.001895

    # wpose.orientation.x = +0.97640
    # wpose.orientation.y = -0.21594
    # wpose.orientation.z = -0.00035
    # wpose.orientation.w = -0.00242

    waypoints.append(deepcopy(wpose))

    straightPath(waypoints)

# go_visionHome ##################################################################################################################

def go_visionHome():

    # 初始化需要使用move group控制的机械臂中的arm group
    arm = MoveGroupCommander('manipulator')
    
    # 设置目标位置所使用的参考坐标系
    arm.set_pose_reference_frame('base_link')
    
    # 获取终端link的名称
    end_effector_link = arm.get_end_effector_link()

    # print("end_effector_link name:", end_effector_link)
                                            
    # 获取当前位姿数据最为机械臂运动的起始位姿
    start_pose = arm.get_current_pose(end_effector_link).pose
    # start_pose = arm.get_current_pose().pose

    # print("graspHome:", start_pose)
    
    # 初始化路点列表
    waypoints = []
            
    # 将初始位姿加入路点列表
    waypoints.append(start_pose)

    # 设置路点数据，并加入路点列表
    wpose = deepcopy(start_pose)
    wpose.position.x =  0.23682
    wpose.position.y =  0.39791
    wpose.position.z =  0.30000 # 0.45000
    
    wpose.orientation.x = -0.599635
    wpose.orientation.y =  0.800275
    wpose.orientation.z =  0.001545
    wpose.orientation.w =  0.001895

    # wpose.orientation.x = +0.97640
    # wpose.orientation.y = -0.21594
    # wpose.orientation.z = -0.00035
    # wpose.orientation.w = -0.00242

    waypoints.append(deepcopy(wpose))

    straightPath(waypoints)
