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

import robot_utils

# 初始化全局发布器
gripper_pub = None

def init_gripper_publisher():
    global gripper_pub
    if gripper_pub is None:
        gripper_pub = rospy.Publisher('/gripper_control', String, queue_size=10)
        rospy.sleep(0.1)  # 确保发布器注册完成

# attainPosition ##################################################################################################################

def attainPosition(x, y, z):
    # 初始化需要使用move group控制的机械臂中的arm group
    arm = MoveGroupCommander('manipulator')

    # 获取终端link的名称
    end_effector_link = arm.get_end_effector_link()
                                            
    # 获取当前位姿数据最为机械臂运动的起始位姿
    start_pose = arm.get_current_pose(end_effector_link).pose
    
    # 初始化路点列表
    waypoints = []
            
    # 将初始位姿加入路点列表
    waypoints.append(start_pose)

    target_pose = deepcopy(start_pose)
    # Starting Postion before picking
    target_pose.position.x = x
    target_pose.position.y = y
    target_pose.position.z = z

    # 将初始位姿加入路点列表
    waypoints.append(target_pose)

    robot_utils.straightPath(waypoints)

    # grasp (general_graspPose2)

# attainObject ##################################################################################################################

def attainObject(general_graspPose2):
    global gripper_pub
    
    # 确保发布器已初始化
    if gripper_pub is None:
        init_gripper_publisher()

    # 初始化需要使用move group控制的机械臂中的arm group
    arm = MoveGroupCommander('manipulator')

    # 获取终端link的名称
    end_effector_link = arm.get_end_effector_link()
                                            
    # 获取当前位姿数据最为机械臂运动的起始位姿
    start_pose = arm.get_current_pose(end_effector_link).pose
    
    # 初始化路点列表
    waypoints = []
            
    # 将初始位姿加入路点列表
    waypoints.append(start_pose)

    # 设置路点数据，并加入路点列表
    wpose = (robot_utils.transform_end(robot_utils.transformCoor(general_graspPose2)))
    wpose.position.z = wpose.position.z + 0.100

    # print ("wpose:", wpose)

    waypoints.append((wpose))
    robot_utils.straightPath(waypoints)

    rospy.loginfo("Open gripper")
    # gripper_control = "open %s"
    gripper_pub.publish("open")
    rospy.sleep(1) # 1.5
    # gripper_control = "open_relax %s"
    gripper_pub.publish("relax_open")
    rospy.sleep(1)

    # grasp (general_graspPose2)

# attainObject ##################################################################################################################

def attainRuler(general_graspPose2):
    global gripper_pub
    
    # 确保发布器已初始化
    # if gripper_pub is None:
    #     init_gripper_publisher()

    # 初始化需要使用move group控制的机械臂中的arm group
    arm = MoveGroupCommander('manipulator')

    # 获取终端link的名称
    end_effector_link = arm.get_end_effector_link()
                                            
    # 获取当前位姿数据最为机械臂运动的起始位姿
    start_pose = arm.get_current_pose(end_effector_link).pose
    
    # 初始化路点列表
    waypoints = []
            
    # 将初始位姿加入路点列表
    waypoints.append(start_pose)

    # 设置路点数据，并加入路点列表
    wpose = (robot_utils.transform_end(robot_utils.transformCoor(general_graspPose2)))
    wpose.position.z = wpose.position.z + 0.020

    # print ("wpose:", wpose)

    waypoints.append((wpose))
    robot_utils.straightPath(waypoints)

    # rospy.loginfo("Open gripper")
    # # gripper_control = "open %s"
    # gripper_pub.publish("open")
    # rospy.sleep(1) # 1.5
    # # gripper_control = "open_relax %s"
    # gripper_pub.publish("relax_open")
    # rospy.sleep(1)

# contactObject ##################################################################################################################

# def contactObject(general_graspPose2):
#     # 初始化需要使用move group控制的机械臂中的arm group
#     arm = MoveGroupCommander('manipulator')

#     # 获取终端link的名称
#     end_effector_link = arm.get_end_effector_link()
                                            
#     # 获取当前位姿数据最为机械臂运动的起始位姿
#     start_pose = arm.get_current_pose(end_effector_link).pose
    
#     # 初始化路点列表
#     waypoints = []
            
#     # 将初始位姿加入路点列表
#     waypoints.append(start_pose)

#     # 设置路点数据，并加入路点列表
#     wpose = (robot_utils.transform_end(robot_utils.transformCoor(general_graspPose2)))
#     wpose.position.z = wpose.position.z

#     # print ("wpose:", wpose)

#     waypoints.append((wpose))
#     robot_utils.straightPath(waypoints)

#     # grasp (general_graspPose2)

# grasp ##################################################################################################################

def grasp(general_graspPose1):
    global gripper_pub
    
    # 确保发布器已初始化
    if gripper_pub is None:
        init_gripper_publisher()

    # 初始化需要使用move group控制的机械臂中的arm group
    arm = MoveGroupCommander('manipulator')

    # 获取终端link的名称
    end_effector_link = arm.get_end_effector_link()
                                            
    # # 获取当前位姿数据最为机械臂运动的起始位姿
    # start_pose1 = arm.get_current_pose(end_effector_link).pose
    
    # # 初始化路点列表
    # waypoints1 = []
            
    # # 将初始位姿加入路点列表
    # waypoints1.append(start_pose1)

    # # 设置路点数据，并加入路点列表
    # wpose1 = (transform_end(transformCoor(general_graspPose1)))
    # wpose1.position.z = wpose1.position.z + 0.100

    # waypoints1.append((wpose1))
    # straightPath(waypoints1)

    # rospy.loginfo("Open gripper")
    # rospy.sleep(2)

    # 获取当前位姿数据最为机械臂运动的起始位姿
    start_pose2 = arm.get_current_pose(end_effector_link).pose
    
    # 初始化路点列表
    waypoints2 = []
            
    # 将初始位姿加入路点列表
    waypoints2.append(start_pose2)

    # 设置路点数据，并加入路点列表
    wpose2 = (robot_utils.transform_end(robot_utils.transformCoor(general_graspPose1)))
    # wpose2.position.z = wpose2.position.z - 0.015

    waypoints2.append(wpose2)
    robot_utils.straightPath(waypoints2)

    rospy.loginfo("Close gripper")
    # gripper_control = "close %s"
    gripper_pub.publish("close")
    rospy.sleep(1)

# grasp_paper ##################################################################################################################

def grasp_paper(general_graspPose1):
    global gripper_pub
    
    # 确保发布器已初始化
    if gripper_pub is None:
        init_gripper_publisher()

    # 初始化需要使用move group控制的机械臂中的arm group
    arm = MoveGroupCommander('manipulator')

    # 获取终端link的名称
    end_effector_link = arm.get_end_effector_link()
                                            
    # 获取当前位姿数据最为机械臂运动的起始位姿
    start_pose2 = arm.get_current_pose(end_effector_link).pose
    
    # 初始化路点列表
    waypoints2 = []
            
    # 将初始位姿加入路点列表
    waypoints2.append(start_pose2)

    # 设置路点数据，并加入路点列表
    wpose2 = (robot_utils.transform_end(robot_utils.transformCoor(general_graspPose1)))
    # wpose2.position.z = 0.11882# delta = 0 # RVIZ Tool0
    wpose2.position.z -= 0.011 # 0.008 # perpendicular difference between open and close gripper

    waypoints2.append(wpose2)
    robot_utils.straightPath(waypoints2)

    rospy.loginfo("Close gripper")
    # gripper_control = "close %s"
    gripper_pub.publish("close")
    rospy.sleep(1)

# lift ##################################################################################################################
# In some cases, it is equal to attainObject
def lift(delta_z=0.15):

    # 初始化需要使用move group控制的机械臂中的arm group
    arm = MoveGroupCommander('manipulator')
    
    # 获取终端link的名称
    end_effector_link = arm.get_end_effector_link()
                                            
    # 获取当前位姿数据最为机械臂运动的起始位姿
    start_pose = arm.get_current_pose(end_effector_link).pose
    
    # 初始化路点列表
    waypoints = []
            
    # 将初始位姿加入路点列表
    waypoints.append(start_pose)

    # 设置路点数据，并加入路点列表
    wpose = deepcopy(start_pose)
    # 若未传入则使用默认 0.15；若传入则使用实参
    wpose.position.z = wpose.position.z + float(delta_z)

    waypoints.append(wpose)
    robot_utils.straightPath(waypoints)

# place##################################################################################################################

def place_to_penHolder(pose, delta_z=0.000): # delta_z to decrease the place height 
    global gripper_pub
    
    # 确保发布器已初始化
    if gripper_pub is None:
        init_gripper_publisher()

    # 初始化需要使用move group控制的机械臂中的arm group
    arm = MoveGroupCommander('manipulator')

    # 获取终端link的名称
    end_effector_link = arm.get_end_effector_link()
                                            
    # 获取当前位姿数据最为机械臂运动的起始位姿
    start_pose = arm.get_current_pose(end_effector_link).pose
    
    # 初始化路点列表
    waypoints = []
    # 将初始位姿加入路点列表
    waypoints.append(start_pose)
            
    pose1 = Pose()
    pose1.position.x =  0.75707 # tool0 in Rviz
    pose1.position.y =  0.18258
    pose1.position.z =  start_pose.position.z - 0.22 - delta_z #- 0.02 # + 0.05 for pen bias experimentsi
    pose1.orientation = robot_utils.transformCoor(pose).orientation   # pose.pose.orientation

    waypoints.append(robot_utils.transform_end(pose1))

    robot_utils.straightPath(waypoints)

    rospy.loginfo("control gripper")
    # gripper_control = "close %s"
    gripper_pub.publish("open")
    rospy.sleep(0.5)
    gripper_pub.publish("relax_open")
    rospy.sleep(0.1)
    gripper_pub.publish("close")
    rospy.sleep(0.1)
    gripper_pub.publish("relax_close")
    rospy.sleep(0.1)

# place ##################################################################################################################

def place(general_graspPose1):
    global gripper_pub
    
    # 确保发布器已初始化
    if gripper_pub is None:
        init_gripper_publisher()

    # 初始化需要使用move group控制的机械臂中的arm group
    arm = MoveGroupCommander('manipulator')

    # 获取终端link的名称
    end_effector_link = arm.get_end_effector_link()

    # 获取当前位姿数据最为机械臂运动的起始位姿
    start_pose2 = arm.get_current_pose(end_effector_link).pose
    
    # 初始化路点列表
    waypoints2 = []
            
    # 将初始位姿加入路点列表
    waypoints2.append(start_pose2)

    # 设置路点数据，并加入路点列表
    wpose2 = (robot_utils.transform_end(robot_utils.transformCoor(general_graspPose1))) # place pose
    wpose2.position.z += 0.010 # 0.010

    waypoints2.append(wpose2)
    robot_utils.straightPath(waypoints2)

    rospy.loginfo("control gripper")
    # gripper_control = "close %s"
    gripper_pub.publish("open")
    rospy.sleep(1)
    gripper_pub.publish("relax_open")
    rospy.sleep(0.1)
    gripper_pub.publish("close")
    rospy.sleep(0.1)
    gripper_pub.publish("relax_close")
    rospy.sleep(0.1)

# push0: for ruler primitive ##################################################################################################################

def push0(push_pose, grasp_pose0, grasp_pose):
    global gripper_pub
    
    # 确保发布器已初始化
    if gripper_pub is None:
        init_gripper_publisher()

    rospy.loginfo("control gripper")
    # gripper_control = "close %s"
    gripper_pub.publish("open")
    rospy.sleep(1)
    gripper_pub.publish("relax_open")
    rospy.sleep(0.1)

    # 初始化需要使用move group控制的机械臂中的arm group
    arm = MoveGroupCommander('manipulator')

    # 获取终端link的名称
    end_effector_link = arm.get_end_effector_link()
                                            
    # 获取当前位姿数据最为机械臂运动的起始位姿
    start_pose1 = arm.get_current_pose(end_effector_link).pose
    
    # 初始化路点列表
    waypoints0 = []
    waypoints1 = []
    waypoints2 = []
    waypoints3 = []
            
    # 将初始位姿加入路点列表
    waypoints1.append(start_pose1)

    # 设置路点数据，并加入路点列表
    wpose1 = (robot_utils.transform_end(robot_utils.transformCoor(push_pose)))
    # wpose1.position.z = wpose1.position.z - 0.005 #+ 0.05 #05 # - 0.005 

    waypoints1.append((wpose1))
    # robot_utils.straightPath(waypoints1)

    # robot_utils.straightPath(waypoints0)

    middle_pose = deepcopy(push_pose)
    middle_pose.pose.position.x = grasp_pose0.pose.position.x
    middle_pose.pose.position.y = grasp_pose0.pose.position.y

    wpose2 = (robot_utils.transform_end(robot_utils.transformCoor(middle_pose)))
    waypoints1.append((wpose2))

    robot_utils.straightPath(waypoints1)
    # gripper_pub.publish("relax_open")
    # rospy.sleep(0.1)

    wpose3 = (robot_utils.transform_end(robot_utils.transformCoor(grasp_pose0)))
    # wpose3.position.z = wpose3.position.z - 0.010 #+ 0.05 #05 # - 0.01 

    waypoints2.append(wpose3)
    robot_utils.straightPath(waypoints2)

    wpose4 = (robot_utils.transform_end(robot_utils.transformCoor(grasp_pose)))
    # wpose3.position.z = wpose3.position.z - 0.010 #+ 0.05 #05 # - 0.01 

    waypoints3.append(wpose4)
    robot_utils.straightPath(waypoints3)

    rospy.loginfo("Close gripper")
    # gripper_control = "close %s"
    gripper_pub.publish("close")
    rospy.sleep(0.5)

# pry: for book primitive ##################################################################################################################

def pry(pry_pose):
    global gripper_pub
    
    # 确保发布器已初始化
    if gripper_pub is None:
        init_gripper_publisher()

    # 初始化需要使用move group控制的机械臂中的arm group
    arm = MoveGroupCommander('manipulator')
    
    # 获取终端link的名称
    end_effector_link = arm.get_end_effector_link()
    
    # 获取当前位姿数据最为机械臂运动的起始位姿
    start_pose1 = arm.get_current_pose(end_effector_link).pose
    
    # 初始化路点列表
    waypoints1 = []
    
    # 将初始位姿加入路点列表
    waypoints1.append(start_pose1)            

    # 设置路点数据，并加入路点列表
    wpose1 = (robot_utils.transform_end(robot_utils.transformCoor(pry_pose)))
    # wpose.position.z = wpose.position.z - 0.010 #+ 0.05 #05 # - 0.01 

    waypoints1.append(wpose1)
    robot_utils.straightPath(waypoints1)                                                                                                         

    rospy.loginfo("Close gripper")
    # gripper_control = "close %s"
    gripper_pub.publish("close")
    rospy.sleep(1)

    # 获取当前位姿数据作为机械臂运动的起始位姿
    start_pose2 = arm.get_current_pose(end_effector_link).pose

    waypoints2 = []
    
    waypoints2.append(start_pose2)

    rotation_pose = robot_utils.rotaionClose(pry_pose, -9*math.pi/180) #-7.5*math.pi/180

    # 设置路点数据，并加入路点列表
    wpose2 = (robot_utils.transform_end(robot_utils.transformCoor(rotation_pose)))
    # wpose.position.z = wpose.position.z - 0.010 #+ 0.05 #05 # - 0.01 

    waypoints2.append(wpose2)
    robot_utils.straightPath(waypoints2)     

# pull: for book primitive ##################################################################################################################

def pull(pose0, pose1):
    global gripper_pub
    
    # 确保发布器已初始化
    if gripper_pub is None:
        init_gripper_publisher()

    # 初始化需要使用move group控制的机械臂中的arm group
    arm = MoveGroupCommander('manipulator')

    # 获取终端link的名称
    end_effector_link = arm.get_end_effector_link()

    # 获取当前位姿数据最为机械臂运动的起始位姿
    start_pose = arm.get_current_pose(end_effector_link).pose

    # 初始化路点列表
    waypoints = []
    # 将初始位姿加入路点列表
    waypoints.append(start_pose)

    # 设置路点数据，并加入路点列表
    wpose0 = (robot_utils.transform_end(robot_utils.transformCoor(pose0)))               
    waypoints.append(wpose0)

    wpose1 = (robot_utils.transform_end(robot_utils.transformCoor(pose1)))               
    waypoints.append(wpose1)

    robot_utils.straightPath(waypoints)

    rospy.loginfo("Open gripper")
    # gripper_control = "close %s"
    gripper_pub.publish("open")
    rospy.sleep(1)
    gripper_pub.publish("relax_open")
    rospy.sleep(0.1)
    gripper_pub.publish("close")
    rospy.sleep(0.1)
    gripper_pub.publish("relax_close")
    rospy.sleep(0.1)
    
# push ##################################################################################################################

def push(pos, inside, outside):

    # 初始化需要使用move group控制的机械臂中的arm group
    arm = MoveGroupCommander('manipulator')

    # 设置目标位置所使用的参考坐标系
    arm.set_pose_reference_frame('base_link')

    # 获取终端link的名称
    end_effector_link = arm.get_end_effector_link()

    # rospy.loginfo("Go to the position above the table")
    # # goHome()
    # go_graspHome()
                                            
    # 获取当前位姿数据最为机械臂运动的起始位姿
    start_pose = arm.get_current_pose(end_effector_link).pose
    
    # 初始化路点列表
    waypoints = []
    # prePose = Pose()
    # pushPose = Pose()
            
    # 将初始位姿加入路点列表
    waypoints.append(start_pose)

    ratio1 = 1000
    pose0 = tf2_geometry_msgs.PoseStamped()
    pose0.header.frame_id = "camera_color_frame"
    pose0.header.stamp = rospy.Time.now()
    pose0.pose.position.x = inside.x*ratio1/500 + outside.x*(500-ratio1)/500
    pose0.pose.position.y = inside.y*ratio1/500 + outside.y*(500-ratio1)/500
    pose0.pose.position.z = inside.z + 0.02
    pose0.pose.orientation = pos.pose.orientation 

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
    roll = 0
    pitch = 0
    yaw = -math.pi/2
    q = quaternion_from_euler(roll, pitch, yaw)

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

    ratio2 = 250
    pose2 = tf2_geometry_msgs.PoseStamped()
    pose2.header.frame_id = "camera_color_frame"
    pose2.header.stamp = rospy.Time.now()
    pose2.pose.position.x = inside.x*ratio2/500 + outside.x*(500-ratio2)/500
    pose2.pose.position.y = inside.y*ratio2/500 + outside.y*(500-ratio2)/500
    pose2.pose.position.z = inside.z*ratio2/500 + outside.z*(500-ratio2)/500 + 0.01
    pose2.pose.orientation = pose1.pose.orientation 

    ratio3 = -250
    pose3 = tf2_geometry_msgs.PoseStamped()
    pose3.header.frame_id = "camera_color_frame"
    pose3.header.stamp = rospy.Time.now()
    pose3.pose.position.x = inside.x*ratio3/500 + outside.x*(500-ratio3)/500
    pose3.pose.position.y = inside.y*ratio3/500 + outside.y*(500-ratio3)/500
    pose3.pose.position.z = outside.z - 0.05
    pose3.pose.orientation = pose1.pose.orientation 


    # realEnd = tf2_geometry_msgs.PoseStamped()
    # # realEnd.pose = msg.pose
    # realEnd.header.frame_id = "camera_color_frame"
    # realEnd.header.stamp = rospy.Time.now()

    # # realEnd = Pose()
    # # print("tuble or list is ok", pos.orientation, pos.position) # or tset it in visionInterface.py

    # # object_to_robot = tf.transformations.quaternion_matrix((1, 1, 0, 0))
    # object_to_camera = tf.transformations.quaternion_matrix([pos.pose.orientation.x, pos.pose.orientation.y, pos.pose.orientation.z, pos.pose.orientation.w]) # the order of xyzw
    # object_to_camera[:, 3] = [pos.pose.position.x, pos.pose.position.y, pos.pose.position.z, 1]

    # pushpose1_to_object = tf.transformations.quaternion_matrix([0, 0, 0, 1]) # the order of xyzw
    # pushpose1_to_object[:, 3] = [0, 0.15, 0.03, 1]

    # pushpose1_to_camera = numpy.dot(object_to_camera, pushpose1_to_object)

    # realEnd.pose.position.x = pushpose1_to_camera[0, 3]
    # realEnd.pose.position.y = pushpose1_to_camera[1, 3]
    # realEnd.pose.position.z = pushpose1_to_camera[2, 3]

    # q = tf.transformations.quaternion_from_matrix(pushpose1_to_camera)
    # realEnd.pose.orientation.x = q[0]
    # realEnd.pose.orientation.y = q[1]
    # realEnd.pose.orientation.z = q[2]
    # realEnd.pose.orientation.w = q[3]

    poseA = robot_utils.transform_end(robot_utils.transformCoor(pose1)) 
    poseB = robot_utils.transform_end(robot_utils.transformCoor(pose2)) 
    # poseC = transform_end(transformCoor(pose3)) 

    waypoints.append(poseA)
    waypoints.append(poseB)
    # waypoints.append(poseC)

    robot_utils.straightPath(waypoints)

    # rospy.loginfo("Go to the position above the table")
    # # goHome()
    # go_graspHome()

def push_neighbor (obj_pose, neighbor_center):
    """
    推动物体本身以远离其邻近物体
    
    参数:
        obj_center: 当前物体中心点 (geometry_msgs/Point)
        neighbor_center: 邻近物体中心点 (geometry_msgs/Point)
        obj_orientation: pose.pose.orientation
    
    返回:
        bool: 是否成功执行推动操作
    """

    # global gripper_pub
    # # 确保发布器已初始化
    # if gripper_pub is None:
    #     init_gripper_publisher()
    # gripper_pub.publish("close")
    # rospy.sleep(0.1)

    try:

        # 创建起始位姿（当前物体中心）
        start_pose = obj_pose
        start_pose.pose.position.z += 0.006
        # start_pose.header.frame_id = "camera_color_frame"
        # start_pose.header.stamp = rospy.Time.now()
        # start_pose.pose.position.x = obj_center.x
        # start_pose.pose.position.y = obj_center.y
        # start_pose.pose.position.z = obj_center.z
        # start_pose.pose.orientation = push_pose.pose.orientation
        obj_center = start_pose.pose.position

        rospy.loginfo(f"开始执行push_neighbor操作: 物体中心({obj_center.x:.3f}, {obj_center.y:.3f}), 邻近物体中心({neighbor_center.x:.3f}, {neighbor_center.y:.3f})")
        
        # 计算推动方向（从邻近物体指向当前物体的方向）
        push_direction_x = obj_center.x - neighbor_center.x
        push_direction_y = obj_center.y - neighbor_center.y
        
        # 计算推动距离（5cm）
        push_distance = 0.05
        
        # 归一化方向向量
        direction_magnitude = math.sqrt(push_direction_x**2 + push_direction_y**2)
        if direction_magnitude == 0:
            rospy.logwarn("推动方向为零向量，无法执行推动操作")
            return False
            
        normalized_direction_x = push_direction_x / direction_magnitude
        normalized_direction_y = push_direction_y / direction_magnitude
        
        # 计算推动后的目标位置
        target_x = obj_center.x + normalized_direction_x * push_distance
        target_y = obj_center.y + normalized_direction_y * push_distance
        target_z = obj_center.z
        
        # 创建推动位姿
        push_pose = PoseStamped()
        push_pose.header.frame_id = "camera_color_frame"
        push_pose.header.stamp = rospy.Time.now()
        push_pose.pose.position.x = target_x
        push_pose.pose.position.y = target_y
        push_pose.pose.position.z = target_z
        
        # 设置推动姿态（保持物体的原始角度）
        push_pose.pose.orientation = start_pose.pose.orientation
        
        rospy.loginfo(f"推动起始位置: ({start_pose.pose.position.x:.3f}, {start_pose.pose.position.y:.3f})")
        rospy.loginfo(f"推动目标位置: ({push_pose.pose.position.x:.3f}, {push_pose.pose.position.y:.3f})")
        
        # 执行推动运动
        # 1. 移动到起始位置上方
        approach_pose = robot_utils.transform_end(robot_utils.transformCoor(deepcopy(start_pose)))
        approach_pose.position.z += 0.05  # 略高于物体表面
        
        # 2. 移动到起始位置
        # robot_utils.goHome()
        # robot_utils.go_visionHome()
        
        # 3. 执行推动运动
        waypoints = []
        waypoints.append(approach_pose)
        waypoints.append(robot_utils.transform_end(robot_utils.transformCoor(start_pose)))
        waypoints.append(robot_utils.transform_end(robot_utils.transformCoor(push_pose)))
        
        # 使用直线路径执行推动
        robot_utils.straightPath(waypoints)

        lift()

        # gripper_pub.publish("relax_close")
        # rospy.sleep(0.1)

        robot_utils.go_visionHome()
        
        rospy.loginfo("push_neighbor操作完成")
        return True
        
    except Exception as e:
        rospy.logerr(f"push_neighbor执行出错: {str(e)}")
        return False

# reorientation ##################################################################################################################
def reorientation(push_point, push_orientation, rotation_direction, object_yaw_angle, center_3d, width=None, height=None):
    """
    执行可变形物体的圆弧推动重新定向
    
    参数:
        push_point: 推动点 (geometry_msgs/Point)
        push_orientation: 推动姿态角 (度)
        rotation_direction: 转动方向 ("cw" 顺时针 或 "ccw" 逆时针)
        object_yaw_angle: 物体角度 (度)
        center_3d: 圆弧路径的中心点 (geometry_msgs/Point)
        width: 物体宽度 (米)，用于计算圆弧半径
        height: 物体高度 (米)，用于计算圆弧半径
    
    返回:
        bool: 是否成功执行重新定向操作
    """
    try:
        rospy.loginfo(f"开始执行reorientation操作")
        rospy.loginfo(f"推动点: ({push_point.x:.3f}, {push_point.y:.3f}, {push_point.z:.3f})")
        rospy.loginfo(f"推动姿态角: {push_orientation:.1f}°")
        rospy.loginfo(f"物体角度: {object_yaw_angle:.1f}°")
        rospy.loginfo(f"转动方向: {rotation_direction}")
        
        # 初始化机械臂
        arm = MoveGroupCommander('manipulator')
        end_effector_link = arm.get_end_effector_link()
        
        # 获取当前位姿作为起始位姿
        start_pose = arm.get_current_pose(end_effector_link).pose
        
        # 计算物体的对角线长度的一半作为圆弧半径
        if width is not None and height is not None:
            diagonal_length = math.sqrt(width**2 + height**2)
            arc_radius = diagonal_length / 2
            rospy.loginfo(f"物体尺寸: {width:.3f} x {height:.3f}, 对角线长度: {diagonal_length:.3f}, 圆弧半径: {arc_radius:.3f}")
        else:
            arc_radius = 0.10  # 10cm，默认值
            rospy.logwarn("未提供物体尺寸，使用默认圆弧半径: 0.1m")
        
        # 根据转动方向确定转动角度
        if rotation_direction == "ccw":
            # 逆时针转动，角度为负值
            rotation_angle = -abs(object_yaw_angle*1.5)
            rospy.loginfo(f"逆时针转动角度: {rotation_angle:.1f}°")
            additional_angle = math.pi/2-math.atan(height/width) #? math.atan(height/width) # 
            rospy.loginfo(f"对角线弧度: {additional_angle:.2f}") # 对角线角度
            coordinate_angle  = math.pi/2 # 坐标系转换角度
        else:
            # 顺时针转动，角度为正值
            rotation_angle = abs(object_yaw_angle*1.5)
            rospy.loginfo(f"顺时针转动角度: {rotation_angle:.1f}°")
            additional_angle = -(math.pi/2-math.atan(width/height)) #? -math.atan(width/height) # 
            rospy.loginfo(f"对角线弧度: {additional_angle:.2f}")
            coordinate_angle  = -math.pi/2
        
        # 将角度转换为弧度
        rotation_angle_rad = math.radians(rotation_angle)
        push_orientation_rad = math.radians(push_orientation)
        
        # 使用传入的圆弧路径中心点
        arc_center_x = center_3d[0]
        arc_center_y = center_3d[1]
        arc_center_z = center_3d[2]
        
        # 计算圆弧路径上的多个点
        num_waypoints = 10  # 圆弧路径上的路点数量
        waypoints = []
        
        # 起始点（推动点上方）
        # start_push_pose = Pose()
        start_push_pose = PoseStamped()
        start_push_pose.header.frame_id = "camera_color_frame"
        start_push_pose.header.stamp = rospy.Time.now()
        start_push_pose.pose.position.x = push_point.x
        start_push_pose.pose.position.y = push_point.y
        start_push_pose.pose.position.z = push_point.z - 0.02  # 略高于物体表面
        
        # 设置起始推动姿态
        q_start = tf.transformations.quaternion_from_euler(0, 0, push_orientation_rad) #- math.pi/2)
        start_push_pose.pose.orientation = Quaternion(*q_start)
        
        waypoints.append(start_push_pose)
        
        # 计算圆弧路径上的中间点
        for i in range(1, num_waypoints + 1):
            # 当前角度
            current_angle = math.radians(object_yaw_angle) + (rotation_angle_rad * i / num_waypoints) + coordinate_angle + additional_angle
            rospy.loginfo(f"current_angle弧度: {current_angle:.2f}")
            
            # 圆弧上的点
            arc_x = arc_center_x + arc_radius * math.cos(current_angle)
            arc_y = arc_center_y + arc_radius * math.sin(current_angle)
            arc_z = arc_center_z +0.006 # 0.002  0.006
            
            # 创建路点
            waypoint = PoseStamped()
            waypoint.header.frame_id = "camera_color_frame"
            waypoint.header.stamp = rospy.Time.now()
            waypoint.pose.position.x = arc_x
            waypoint.pose.position.y = arc_y
            waypoint.pose.position.z = arc_z
            
            # 夹爪姿态和书籍保持相对静止，角度也随着书籍角度的变化而变化
            gripper_angle = push_orientation_rad + (rotation_angle_rad * i / num_waypoints) #- math.pi/2
            q_waypoint = tf.transformations.quaternion_from_euler(0, 0, gripper_angle)
            waypoint.pose.orientation = Quaternion(*q_waypoint)
            
            waypoints.append(waypoint)
        
        # 最终点（推动完成后的位置）
        # final_push_pose = Pose()
        final_push_pose = PoseStamped()
        final_push_pose.header.frame_id = "camera_color_frame"
        final_push_pose.header.stamp = rospy.Time.now()
        final_angle = math.radians(object_yaw_angle) + rotation_angle_rad + coordinate_angle + additional_angle
        final_x = arc_center_x + arc_radius * math.cos(final_angle)
        final_y = arc_center_y + arc_radius * math.sin(final_angle)
        final_z = arc_center_z - 0.02
        
        final_push_pose.pose.position.x = final_x
        final_push_pose.pose.position.y = final_y
        final_push_pose.pose.position.z = final_z
        
        final_gripper_angle = push_orientation_rad + rotation_angle_rad #- math.pi/2
        q_final = tf.transformations.quaternion_from_euler(0, 0, final_gripper_angle)
        final_push_pose.pose.orientation = Quaternion(*q_final)
        
        waypoints.append(final_push_pose)
        
        rospy.loginfo(f"圆弧路径计算完成，共 {len(waypoints)} 个路点")
        rospy.loginfo(f"圆弧中心: ({arc_center_x:.3f}, {arc_center_y:.3f}, {arc_center_z:.3f})")
        rospy.loginfo(f"圆弧半径: {arc_radius:.3f}m")
        
        # 执行圆弧路径运动
        # 1. 移动到起始推动位置上方
        # approach_pose = deepcopy(start_push_pose)
        # approach_pose.pose.position.z += 0.05  # 更高一些，避免碰撞
        
        # 2. 移动到起始推动位置
        # attainObject(approach_pose)
        
        # 3. 执行圆弧推动路径
        for i, waypoint in enumerate(waypoints):
            rospy.loginfo(f"执行路点 {i+1}/{len(waypoints)}: ({waypoint.pose.position.x:.3f}, {waypoint.pose.position.y:.3f})")
            
            # 转换坐标系并执行运动
            target_pose = robot_utils.transform_end(robot_utils.transformCoor(waypoint))
            arm.set_pose_target(target_pose)
            arm.go(wait=True)
            
            # 短暂等待，确保运动完成
            rospy.sleep(0.1)

        # 4. 抬起夹爪
        # lift_pose = deepcopy(final_push_pose)
        # lift_pose.pose.position.z += 0.05
        # attainObject(lift_pose)
        
        # 5. 返回初始位置
        robot_utils.go_visionHome()
        
        rospy.loginfo("reorientation操作完成")
        return True
        
    except Exception as e:
        rospy.logerr(f"reorientation执行出错: {str(e)}")
        return False
