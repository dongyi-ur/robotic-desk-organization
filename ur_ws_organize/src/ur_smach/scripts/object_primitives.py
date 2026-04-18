#!/usr/bin/python

import numpy as np
import rospy, sys
# import smach
# import smach_ros
# from smach import CBState, State
# from smach import State

# from turtlesim.msg import Pose
# from geometry_msgs import Pose
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, PointStamped
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

from object_vision_manager1 import ObjectVisionManager
import robot_utils, geometry_utils
import motion_primitives

from geometry_msgs.msg import Pose, Quaternion
import tf.transformations

# class EraserPrimitive: Line55
# class PenPrimitive: Line172
# class RulerPrimitive: Line265
# class PaperPrimitive: Line494
# class BookPrimitive: Line773

# class PrimitiveFactory:
#     @staticmethod
#     def get_primitive(obj_type: str):
#         """工厂方法返回对应物体的操作原语"""
#         primitives = {
#             "pen": PenPrimitive(),
#             "book": BookPrimitive(),
#             "ruler": RulerPrimitive()
#         }
#         return primitives.get(obj_type, DefaultPrimitive())

# manage_neighborhood ###################################################################################################  
def manage_neighborhood(obj_class, pose):
    """
    管理邻近物体，清除4cm圆形区域内的紧邻物体干扰
    
    参数:
        obj_class: 物体类别名
        obj_info: 视觉层的 ObjectInfo
        pose: 该物体的位姿（Pose 或 PoseStamped），obj_center 取 pose.pose.position
    
    返回:
        bool: 是否成功处理邻近物体
    """
    try:
        # 计算二维点到线段/多边形边界的最短距离（仅使用x,y）
        def _point_to_segment_distance_2d(px, py, ax, ay, bx, by):
            abx = bx - ax
            aby = by - ay
            apx = px - ax
            apy = py - ay
            ab_len_sq = abx * abx + aby * aby
            if ab_len_sq == 0:
                return math.hypot(apx, apy)
            t = (apx * abx + apy * aby) / ab_len_sq
            if t < 0.0:
                t = 0.0
            elif t > 1.0:
                t = 1.0
            cx = ax + t * abx
            cy = ay + t * aby
            return math.hypot(px - cx, py - cy)

        def _point_to_polygon_boundary_distance_2d(point_xy, polygon_pts):
            if not polygon_pts:
                return float('inf')
            cx = sum(p.x for p in polygon_pts) / float(len(polygon_pts))
            cy = sum(p.y for p in polygon_pts) / float(len(polygon_pts))
            sorted_pts = sorted(polygon_pts, key=lambda p: math.atan2(p.y - cy, p.x - cx))
            px, py = point_xy
            min_d = float('inf')
            n = len(sorted_pts)
            for i in range(n):
                a = sorted_pts[i]
                b = sorted_pts[(i + 1) % n]
                d = _point_to_segment_distance_2d(px, py, a.x, a.y, b.x, b.y)
                if d < min_d:
                    min_d = d
            return min_d

        # 获取所有物体的视觉信息
        vision_manager = ObjectVisionManager()
        
        # 添加调试信息
        rospy.loginfo(f"=== 调试信息 ===")
        rospy.loginfo(f"正在创建 ObjectVisionManager 实例...")
        # 等待一段时间让订阅者建立连接
        rospy.sleep(0.5) # very important
        # 检查话题状态
        try:
            topics = rospy.get_published_topics()
            rospy.loginfo(f"可用的物体检测话题:")
            for topic_name, topic_type in topics:
                if 'object' in topic_name.lower() or 'detection' in topic_name.lower():
                    rospy.loginfo(f"  - {topic_name} ({topic_type})")
        except Exception as e:
            rospy.logwarn(f"无法获取话题列表: {e}")
        
        all_objects = vision_manager.get_all_objects_3d()
        
        # 打印 all_objects 的详细信息
        # rospy.loginfo(f"=== all_objects 信息 ===")
        # rospy.loginfo(f"检测到的物体总数: {len(all_objects)}")
        # if len(all_objects) == 0:
        #     rospy.logwarn("警告：未检测到任何物体！可能的原因：")
        #     rospy.logwarn("1. 相机话题未发布数据")
        #     rospy.logwarn("2. 物体检测话题未发布数据")
        #     rospy.logwarn("3. 数据同步失败")
        #     rospy.logwarn("4. 相机内参或深度图像未准备好")
        # else:
        #     for i, (obj, pts) in enumerate(all_objects):
        #         rospy.loginfo(f"物体 {i+1}:")
        #         rospy.loginfo(f"  - 类别: {obj.class_name}")
        #         rospy.loginfo(f"  - 是否为多边形: {obj.is_polygon}")
        #         rospy.loginfo(f"  - 点云数量: {len(pts)}")
        #         if len(pts) > 0:
        #             rospy.loginfo(f"  - 第一个点位置: ({pts[0].x:.3f}, {pts[0].y:.3f}, {pts[0].z:.3f})")
        # rospy.loginfo(f"=== all_objects 信息结束 ===")
        
        # 直接使用传入位姿的平移作为当前物体中心
        obj_center = pose.pose.position
        
        # 判断该物体是否在书上
        is_on_book = False
        if obj_class not in ["paper", "book"]:
            # 获取所有书本的角点信息
            book_corners_list = []
            for obj, pts in all_objects:
                if obj.class_name == "book" and obj.is_polygon and len(pts) >= 4:
                    book_corners_list.append(pts)
            
            # 检查是否在书上
            for book_corners in book_corners_list:
                if geometry_utils.is_ruler_on_book(book_corners, obj_center):
                    is_on_book = True
                    break
        
        # 定义需要排除的物体类别
        if is_on_book:
            # 在书上时，排除纸张和书
            exclude_classes = {"paper", "book"}
        else:
            # 不在书上时，只排除纸张
            exclude_classes = {"paper"}
        
        # 查找4cm圆形区域内的紧邻物体
        nearest_neighbor = None
        min_distance = float('inf')
        neighbor_center = None
        
        for other_obj, other_pts in all_objects:
            # 跳过自身和需要排除的类别
            if (other_obj.class_name == obj_class or 
                other_obj.class_name in exclude_classes):
                continue
            
            # 计算其他物体的中心
            if other_obj.is_polygon and len(other_pts) >= 3:
                other_center_xyz, _, _, _ = geometry_utils.calculate_min_area_rect(other_pts)
                other_center = Point(x=other_center_xyz[0], y=other_center_xyz[1], z=other_center_xyz[2])
            elif len(other_pts) >= 1:
                other_center = other_pts[0]
            else:
                continue
            # 输出 other_center 的信息
            # rospy.loginfo(f"other_center 信息 - 类别: {other_obj.class_name}, 位置: ({other_center.x:.3f}, {other_center.y:.3f}, {other_center.z:.3f})")
            
            # 计算距离
            if other_obj.is_polygon and len(other_pts) >= 3:
                # 多边形：使用到其边界的最近二维距离
                distance = _point_to_polygon_boundary_distance_2d(
                    (obj_center.x, obj_center.y), other_pts
                )
            else:
                # 非多边形：使用到中心的XY平面距离
                distance = math.sqrt(
                    (obj_center.x - other_center.x) ** 2 +
                    (obj_center.y - other_center.y) ** 2
                )
            # rospy.loginfo(f"与 {other_obj.class_name} 的距离: {distance:.3f}m")
            
            # 检查是否在4cm圆形区域内
            if distance <= 0.04 and distance < min_distance:
                min_distance = distance
                nearest_neighbor = other_obj
                neighbor_center = other_center
        
        # 如果找到紧邻物体，执行推动操作
        if nearest_neighbor is not None:
            rospy.loginfo(f"检测到物体 {obj_class} 与 {nearest_neighbor.class_name} 紧邻，距离: {min_distance:.3f}m")
            
            # 调用push_neighbor函数
            success = motion_primitives.push_neighbor(
                pose,  # 当前物体中心
                neighbor_center,  # 邻近物体中心
                # pose.pose.orientation
                # obj_info.rect_angle if hasattr(obj_info, 'rect_angle') else 0  # 当前物体姿态
            )
            
            if success:
                rospy.loginfo(f"成功推动物体 {obj_class} 远离邻近物体")
                return True
            else:
                rospy.logwarn(f"推动物体 {obj_class} 失败")
                return False
        
        # 未找到紧邻物体
        rospy.loginfo("该物体4cm圆形区域内无紧邻物体")
        return True
        
    except Exception as e:
        rospy.logerr(f"manage_neighborhood 执行出错: {str(e)}")
        return False

# EraserPrimitive ###################################################################################################  
class EraserPrimitive:
    def __init__(self):
        """
        初始化橡皮擦操作原语
        
        参数:
            motion_primitives: 运动原语对象
            gripper_pub: 夹爪控制发布器
        """

        self.object_vision_manager = ObjectVisionManager()

        self.motion_primitives = motion_primitives
        self.motion_primitives.init_gripper_publisher()

        # self.gripper_pub = gripper_pub
        self.eraser_pose = None
        self.eraser_position = None
        self.eraser_width = None
        self.eraser_height = None
        self.eraser_angle = None

        # lead_case 属性
        self.lead_case_pose = None
        self.lead_case_position = None
        self.lead_case_width = None
        self.lead_case_height = None
        self.lead_case_angle = None

        # self.eraser_pose = None
        # self.penHolder_position = None
        # self.eraser_angle = None
        
    def _get_eraser_data(self):
        """获取橡皮擦的视觉数据"""
        try:
            eraser_data = self.object_vision_manager.get_objects_by_class_3d('eraser')
            if not eraser_data:
                rospy.logerr("未检测到橡皮擦物体!")
                return False
            
            # 检查数据结构的完整性
            if len(eraser_data[0]) < 2:
                rospy.logerr("橡皮擦数据结构不完整!")
                return False
                
            obj_info, points_3d = eraser_data[0]
            
            # 检查3D点数据是否存在
            if not points_3d or len(points_3d) == 0:
                rospy.logerr("橡皮擦3D点数据为空!")
                return False
                
            self.eraser_position = points_3d[0]  # 第一个橡皮擦物体的第一个3D点坐标
            #self.eraser_position.x += 0.006
            #self.eraser_position.y += 0.009
            
            self.eraser_width = obj_info.rect_width  # 第一个橡皮擦物体的width
            self.eraser_height = obj_info.rect_height  # 第一个橡皮擦物体的height
            
            # 根据宽高比调整角度赋值
            raw_angle = obj_info.rect_angle  # 原始角度值
            if self.eraser_width < self.eraser_height:
                self.eraser_angle = raw_angle
            else:
                self.eraser_angle = raw_angle + 90  
            
            self.eraser_pose = robot_utils.pose3d(self.eraser_position, self.eraser_angle-90)
            return True
        except Exception as e:
            rospy.logerr(f"获取橡皮擦数据时出错: {str(e)}")
            return False

    def _get_lead_case_data(self):
        """获取铅盒(lead_case)的视觉数据"""
        try:
            lead_case_data = self.object_vision_manager.get_objects_by_class_3d('lead_case')
            if not lead_case_data:
                rospy.logerr("未检测到lead_case物体!")
                return False

            # 检查数据结构的完整性
            if len(lead_case_data[0]) < 2:
                rospy.logerr("lead_case数据结构不完整!")
                return False
                
            obj_info, points_3d = lead_case_data[0]
            
            # 检查3D点数据是否存在
            if not points_3d or len(points_3d) == 0:
                rospy.logerr("lead_case 3D点数据为空!")
                return False

            self.lead_case_position = points_3d[0]
            #self.lead_case_position.x += 0.003
            #self.lead_case_position.y += 0.021

            self.lead_case_width = obj_info.rect_width
            self.lead_case_height = obj_info.rect_height

            raw_angle = obj_info.rect_angle
            if self.lead_case_width < self.lead_case_height:
                self.lead_case_angle = raw_angle
            else:
                self.lead_case_angle = raw_angle + 90

            self.lead_case_pose = robot_utils.pose3d(self.lead_case_position, self.lead_case_angle-90)
            return True
        except Exception as e:
            rospy.logerr(f"获取lead_case数据时出错: {str(e)}")
            return False
        
    # def _get_penHolder_data(self):
    #     """获取橡皮擦的视觉数据"""
    #     try:
    #         penHolder_data = self.object_vision_manager.get_objects_by_class_3d('pen holder')
    #         if not penHolder_data:
    #             rospy.logerr("未检测到笔筒物体!")
    #             return False
                
    #         self.penHolder_pose = self.eraser_pose
    #         self.penHolder_pose.pose.position = penHolder_data[0][1][0]  # 第一个橡皮擦物体的第一个3D点坐标
    #         return True
    #     except Exception as e:
    #         rospy.logerr(f"获取笔筒数据时出错: {str(e)}")
    #         return False
    
    # def _control_gripper(self, command, duration=1.0):
    #     """控制夹爪状态"""
    #     self.gripper_pub.publish(command)
    #     rospy.sleep(duration)
    
    def execute(self):

        """执行橡皮擦/lead_case 操作序列"""
        has_eraser = self._get_eraser_data()
        if not has_eraser:
            has_lead_case = self._get_lead_case_data()
            if not has_lead_case:
                return False

        target_class = 'eraser' if has_eraser else 'lead_case'
        target_pose = self.eraser_pose if has_eraser else self.lead_case_pose

        # 管理邻近物体，清除干扰
        detected_list = self.object_vision_manager.get_objects_by_class_3d(target_class)
        if detected_list:
            manage_neighborhood(target_class, target_pose)
            # 邻近物体处理可能改变目标物体位姿，重新获取数据与位姿
            rospy.sleep(0.3)
            if target_class == 'eraser':
                self._get_eraser_data()
                target_pose = self.eraser_pose
            else:
                self._get_lead_case_data()
                target_pose = self.lead_case_pose
            
        try:
            # 确保发布器已初始化
            if self.motion_primitives.gripper_pub is None:
                self.motion_primitives.init_gripper_publisher()

            rospy.loginfo("move the arm...")
            # robot_utils.goHome()
            self.motion_primitives.gripper_pub.publish("close")
            self.motion_primitives.gripper_pub.publish("relax_close")

            rospy.loginfo("移动到桌子上方位置...")
            robot_utils.go_visionHome()

            rospy.loginfo("接近物体...")
            self.motion_primitives.attainObject(target_pose)

            adjusted_pose = deepcopy(target_pose)
            adjusted_pose.pose.position.z = target_pose.pose.position.z + 0.011 # necessary 0.006 

            rospy.loginfo("尝试抓取物体...")
            self.motion_primitives.grasp(adjusted_pose)

            rospy.loginfo("抬起物体...")
            self.motion_primitives.lift()

            rospy.loginfo("place pbject...")
            self.motion_primitives.place_to_penHolder(adjusted_pose)

            rospy.loginfo("返回初始位置...")
            #robot_utils.go_graspHome()
            robot_utils.go_visionHome()
            
            return True
        except Exception as e:
            rospy.logerr(f"执行橡皮擦/lead_case 操作时出错: {str(e)}")
            return False

# PenPrimitive ###################################################################################################  
class PenPrimitive:
    def __init__(self):
        """
        初始化Pen操作原语
        
        参数:
            motion_primitives: 运动原语对象
            gripper_pub: 夹爪控制发布器
        """

        self.object_vision_manager = ObjectVisionManager()

        self.motion_primitives = motion_primitives
        self.motion_primitives.init_gripper_publisher()

        # self.gripper_pub = gripper_pub
        self.pen_pose = None
        self.pen_position = None
        self.pen_width = None
        self.pen_height = None
        self.pen_angle = None
        
    def _get_pen_data(self):
        """获取Pen的视觉数据"""
        try:
            pen_data = self.object_vision_manager.get_objects_by_class_3d('pen')
            if not pen_data:
                rospy.logerr("未检测到pen!")
                return False
            
            # 检查数据结构的完整性
            if len(pen_data[0]) < 2:
                rospy.logerr("pen数据结构不完整!")
                return False
                
            obj_info, points_3d = pen_data[0]
            
            # 检查3D点数据是否存在
            if not points_3d or len(points_3d) == 0:
                rospy.logerr("pen 3D点数据为空!")
                return False
                
            self.pen_position = points_3d[0]  # 第一个物体的第一个3D点坐标 
            #self.pen_position.x += 0.003
            #self.pen_position.y += 0.051

            self.pen_width = obj_info.rect_width  # 第一个物体的width 
            self.pen_height = obj_info.rect_height  # 第一个物体的height 
            
            # 根据宽高比调整角度赋值
            raw_angle = obj_info.rect_angle  # 原始角度值
            if self.pen_width < self.pen_height:
                self.pen_angle = raw_angle
            else:
                self.pen_angle = raw_angle + 90  # 当高度大于宽度时旋转90度
            
            self.pen_pose = robot_utils.pose3d(self.pen_position, self.pen_angle-90)
            return True
        except Exception as e:
            rospy.logerr(f"获取pen数据时出错: {str(e)}")
            return False
    
    def execute(self):

        """执行pen操作序列"""
        if not self._get_pen_data():
            return False
        # if not self._get_penHolder_data(): # change the pen data ??? deepcopy???
        #     return False
        
        # 管理邻近物体，清除干扰
        pen_data = self.object_vision_manager.get_objects_by_class_3d('pen')
        if pen_data:
            manage_neighborhood('pen', self.pen_pose)
            # 邻近物体处理可能改变位姿，重新获取pen数据与位姿
            rospy.sleep(0.3)
            self._get_pen_data()
            
        try:
            # 确保发布器已初始化
            if self.motion_primitives.gripper_pub is None:
                self.motion_primitives.init_gripper_publisher()

            rospy.loginfo("move the arm...")
            # robot_utils.goHome()
            self.motion_primitives.gripper_pub.publish("close")
            self.motion_primitives.gripper_pub.publish("relax_close")

            rospy.loginfo("移动到桌子上方位置...")
            robot_utils.go_visionHome()

            rospy.loginfo("接近物体...")
            self.motion_primitives.attainObject(self.pen_pose)

            grasp_pose = deepcopy(self.pen_pose)
            grasp_pose.pose.position.z = self.pen_pose.pose.position.z + 0.009 # 0.015ii
            rospy.loginfo("尝试抓取物体...")
            self.motion_primitives.grasp(grasp_pose)

            rospy.loginfo("抬起物体...")
            self.motion_primitives.lift(0.20)

            place_pose = robot_utils.setTargetRotation(self.pen_pose) # the other option is to define a class subfunction for placePose
            rospy.loginfo("place pbject...")
            self.motion_primitives.place_to_penHolder(place_pose)

            rospy.loginfo("返回初始位置...")
            # robot_utils.go_graspHome()
            robot_utils.go_visionHome()
            
            return True
        except Exception as e:
            rospy.logerr(f"执行pen操作时出错: {str(e)}")
            return False

# RulerPrimitive ###################################################################################################  
class RulerPrimitive: # motion primitive does not been defined
    def __init__(self):
        """
        初始化Ruler/Triangle操作原语
        
        参数:
            motion_primitives: 运动原语对象
        """
        self.object_vision_manager = ObjectVisionManager()

        self.motion_primitives = motion_primitives
        self.motion_primitives.init_gripper_publisher()
        
        # Ruler 属性
        self.ruler_position = None
        # self.ruler_position2D =None # for is_ruler_on_book
        self.ruler_width = None
        self.ruler_height = None
        self.ruler_angle = None
        self.ruler_yaw_angle = None

        # Triangle 属性
        self.triangle_position = None
        self.triangle_width = None
        self.triangle_height = None
        self.triangle_angle = None
        self.triangle_yaw_angle = None

        self.book_corners = None
        # self.book_corners2D = None # is_ruler_on_book
        self.book_position = None
        self.book_width = None
        self.book_height = None
        self.book_angle = None
        self.book_yaw_angle = None

        self.desktop_corners = None
        # self.book_corners2D = None # is_ruler_on_book
        self.desktop_position = None
        self.desktop_width = None
        self.desktop_height = None
        self.desktop_angle = None
        self.desktop_yaw_angle = None

        self.rotationOpen_direction = None # control the opposite rotation angle bet. pushPose and graspPose

    def _get_ruler_data(self):
        """获取尺子的视觉数据"""
        try:
            ruler_data = self.object_vision_manager.get_objects_by_class_3d('ruler')
            if not ruler_data:
                rospy.logerr("未检测到尺子!")
                return False
                
            # 获取尺子的角点信息
            ruler_obj = ruler_data[0][0]
            if ruler_obj.is_polygon and len(ruler_data[0][1]) >= 4: # and len(ruler_data[0][1]) >= 4
                # corners2D = ruler_obj.polygon_corners
                # 计算最小外接矩形
                # self.ruler_position2D = geometry_utils.calculate_min_area_rect2D(corners2D)[0]
                # center_xy = geometry_utils.calculate_min_area_rect2D(corners2D)[0]
                # self.ruler_position2D = Point(x=center_xy[0], y=center_xy[1], z=0.0)

                corners = ruler_data[0][1]
                # rospy.loginfo(f"Ruler角点信息: {corners}")
                
                # 计算最小外接矩形
                center, width, height, angle = geometry_utils.calculate_min_area_rect(corners)
                
                # 创建3D点对象 (假设使用类似geometry_msgs/Point的结构)
                self.ruler_position = Point(x=center[0], y=center[1], z=center[2])
                self.ruler_width = width
                self.ruler_height = height
                self.ruler_angle = angle

                # 计算基础角度
                if self.ruler_width > self.ruler_height:
                    self.ruler_yaw_angle = self.ruler_angle + 90
                else:
                    self.ruler_yaw_angle = self.ruler_angle
                
                rospy.loginfo(f"检测到尺子: 位置({center[0]:.3f}, {center[1]:.3f}), "
                            f"ruler_angle: {angle:.1f}°, ruler_yaw_angle: {self.ruler_yaw_angle:.1f}°, 尺寸: {width:.2f}x{height:.2f}")
                return True
            else:
                rospy.logwarn("尺子信息不完整，无法获取角点")
                return False
        except Exception as e:
            rospy.logerr(f"获取尺子数据时出错: {str(e)}")
            return False

    def _get_triangle_data(self):
        """获取三角形的视觉数据"""
        try:
            triangle_data = self.object_vision_manager.get_objects_by_class_3d('triangle')
            if not triangle_data:
                rospy.logerr("未检测到三角形!")
                return False
                
            # 获取三角形的角点信息
            triangle_obj = triangle_data[0][0]
            if triangle_obj.is_polygon and len(triangle_data[0][1]) >= 3: # 三角形至少需要3个角点
                corners = triangle_data[0][1]
                
                # 计算最小外接矩形
                center, width, height, angle = geometry_utils.calculate_min_area_rect(corners)
                
                # 创建3D点对象
                self.triangle_position = Point(x=center[0], y=center[1], z=center[2])
                self.triangle_width = width
                self.triangle_height = height
                self.triangle_angle = angle

                # 计算基础角度
                if self.triangle_width > self.triangle_height:
                    self.triangle_yaw_angle = self.triangle_angle + 90
                else:
                    self.triangle_yaw_angle = self.triangle_angle
                
                rospy.loginfo(f"检测到三角形: 位置({center[0]:.3f}, {center[1]:.3f}), "
                            f"triangle_angle: {angle:.1f}°, triangle_yaw_angle: {self.triangle_yaw_angle:.1f}°, 尺寸: {width:.2f}x{height:.2f}")
                return True
            else:
                rospy.logwarn("三角形信息不完整，无法获取角点")
                return False
        except Exception as e:
            rospy.logerr(f"获取三角形数据时出错: {str(e)}")
            return False
    
    def _get_book_data(self):
        """获取书本的视觉数据"""
        try:
            book_data = self.object_vision_manager.get_objects_by_class_3d('book')
            if not book_data:
                rospy.logwarn("未检测到书本!")
                self.book_corners = None
                return False
                
            # 获取书本的角点信息
            book_obj = book_data[0][0]
            if book_obj.is_polygon and len(book_data[0][1]) >= 4:
                self.book_corners = book_data[0][1] 
                # self.book_corners2D = book_obj.polygon_corners

                # 计算最小外接矩形
                center, width, height, angle = geometry_utils.calculate_min_area_rect(self.book_corners)
                
                # 创建3D点对象 (假设使用类似geometry_msgs/Point的结构)
                self.book_position = Point(x=center[0], y=center[1], z=center[2])
                self.book_width = width
                self.book_height = height
                self.book_angle = angle

                # 计算基础角度
                if self.book_width > self.book_height:
                    self.book_yaw_angle = self.book_angle + 90
                else:
                    self.book_yaw_angle = self.book_angle

                rospy.loginfo(f"检测到书本: 位置({center[0]:.3f}, {center[1]:.3f}), "
                            f"book_angle: {angle:.1f}°, book_yaw_angle: {self.book_yaw_angle:.1f}°, 尺寸: {width:.2f}x{height:.2f}")
                # rospy.loginfo(f"书本角点信息: {self.book_corners}")
                return True
            else:
                rospy.logwarn("书本信息不完整，无法获取角点")
                self.book_corners = None
                return False
        except Exception as e:
            rospy.logerr(f"获取书本数据时出错: {str(e)}")
            self.book_corners = None
            return False
        
    def _get_desktop_data(self):
        """获取desktop的视觉数据"""
        try:
            desktop_data = self.object_vision_manager.get_objects_by_class_3d('desktop')
            if not desktop_data:
                rospy.logwarn("未检测到desktop!")
                self.desktop_corners = None
                return False
                
            # 获取desktop的角点信息
            desktop_obj = desktop_data[0][0]
            if desktop_obj.is_polygon and len(desktop_data[0][1]) >= 4:
                self.desktop_corners = desktop_data[0][1] 
                # self.book_corners2D = book_obj.polygon_corners

                # 计算最小外接矩形
                center, width, height, angle = geometry_utils.calculate_min_area_rect(self.desktop_corners)
                
                # 创建3D点对象 (假设使用类似geometry_msgs/Point的结构)
                self.desktop_position = Point(x=center[0], y=center[1], z=center[2])
                self.desktop_width = width
                self.desktop_height = height
                self.desktop_angle = angle

                # 计算基础角度
                if self.desktop_width > self.desktop_height:
                    self.desktop_yaw_angle = self.desktop_angle + 90
                else:
                    self.desktop_yaw_angle = self.desktop_angle

                rospy.loginfo(f"检测到desktop: 位置({center[0]:.3f}, {center[1]:.3f}), "
                            f"desktop_angle: {angle:.1f}°, desktop_yaw_angle: {self.desktop_yaw_angle:.1f}°, 尺寸: {width:.2f}x{height:.2f}")
                # rospy.loginfo(f"desktop角点信息: {self.book_corners}")
                return True
            else:
                rospy.logwarn("desktop信息不完整，无法获取角点")
                self.desktop_corners = None
                return False
        except Exception as e:
            rospy.logerr(f"获取desktop数据时出错: {str(e)}")
            self.desktop_corners = None
            return False
        
    def get_push_pose(self, direction, on_book, target_class, target_position, target_yaw_angle, target_angle):
        """
        根据push方向生成push位姿
        
        参数:
            direction: 'longEdge' 或 'shortEdge'
            on_book: 是否在书本上
            target_class: 目标物体类别 ('ruler' 或 'triangle')
            target_position: 目标物体位置
            target_yaw_angle: 目标物体偏航角
            target_angle: 目标物体角度
        返回:
            位姿 (geometry_msgs/Pose) 或 None (失败时)
        """
        try:
            # 验证输入方向
            if direction not in ['longEdge', 'shortEdge']:
                rospy.logerr(f"无效的推方向: {direction}，应为'longEdge'或'shortEdge'")
                return None
                
            # 检查目标物体位置
            if not target_position:
                rospy.logerr(f"{target_class}位置未知，无法生成push位姿")
                return None
            
            # 2. 确定push方向 
            push_direction = direction
            
            # 4. 计算垂足作为抓取位置
            if on_book:
                grasp_position_2d = geometry_utils.calculate_foot_of_perpendicular(
                    self.book_corners,
                    push_direction, 
                    target_position
                )
            else:
                grasp_position_2d = geometry_utils.calculate_foot_of_perpendicular(
                    self.desktop_corners,
                    push_direction, 
                    target_position
                )

            # 目标物体参数
            target_center = np.array([target_position.x, target_position.y])
            target_angle_rad = np.radians(target_angle)
            
            # 根据目标物体类型获取尺寸
            if target_class == 'ruler':
                target_width = self.ruler_width
                target_height = self.ruler_height
            else:  # triangle
                target_width = self.triangle_width
                target_height = self.triangle_height
            
            # 计算目标物体两个长边的中心点
            edge1, edge2, is_width_longer = geometry_utils.calculate_edge_centers(
                center=target_center,
                width=target_width,
                height=target_height,
                angle=target_angle_rad
            )
            
            # # 选择距离grasp_position close的边
            # dist1 = np.linalg.norm(edge1 - grasp_position_2d)
            # dist2 = np.linalg.norm(edge2 -grasp_position_2d)
            # push_point_2d = edge1 if dist1 > dist2 else edge2

            # 计算边的连线方向向量
            edge_vector = edge2 - edge1
            edge_direction = edge_vector / np.linalg.norm(edge_vector)  # 单位方向向量
            # 计算边的长度
            edge_length = np.linalg.norm(edge_vector)
            
            # 选择距离grasp_position较远的边，并向外延伸
            dist1 = np.linalg.norm(edge1 - grasp_position_2d)
            dist2 = np.linalg.norm(edge2 - grasp_position_2d)
            
            if dist1 > dist2:
                # 从edge1向外延伸边长的1/10
                push_point_2d = edge1 - edge_direction * (edge_length * 0.1)
            else:
                # 从edge2向外延伸边长的1/10
                push_point_2d = edge2 + edge_direction * (edge_length * 0.1)

            # 计算抓取姿态的偏航角source devel/setup.bash
            # yaw_angle = paper_angle_rad # if is_width_longer else paper_angle_rad + np.pi/2
            
            # 创建抓取位姿
            # grasp_pose = Pose()
            # if on_book:
            #     push_position = Point(
            #     x=push_point_2d[0], 
            #     y=push_point_2d[1], 
            #     z=self.book_position.z # or use is_ruler_on_book, and dektop_position, book_position
            #     )
            # else:
            #     push_position = Point(
            #     x=push_point_2d[0], 
            #     y=push_point_2d[1], 
            #     z=self.desktop_position.z # or use is_ruler_on_book, and dektop_position, book_position
            #     )

            push_position = Point(x=push_point_2d[0], y=push_point_2d[1], z=target_position.z)

            push_pose0 = robot_utils.pose3d(push_position, target_yaw_angle-90)
            self.rotationOpen_direction = geometry_utils.rotationOpen_direction(target_yaw_angle, push_point_2d, grasp_position_2d)
            push_pose = robot_utils.rotaionOpen(push_pose0, 7.5*math.pi/180, self.rotationOpen_direction) # control the fingertip
            # push_pose = robot_utils.rotaionOpen(push_pose0, 5*math.pi/180) # control the fingertip

            # 记录调试信息
            pos = push_pose0.pose.position
            rospy.loginfo(f"生成{target_class}推位姿: 方向={direction}, 位置=({pos.x:.3f}, {pos.y:.3f}), yaw_angle={target_yaw_angle:.1f}°")
            return push_pose0, push_pose
            
        except Exception as e:
            rospy.logerr(f"生成{target_class}推位姿时出错: {str(e)}")
            import traceback
            rospy.logerr(traceback.format_exc())  # 输出完整堆栈跟踪
            return None
    
    def get_grasp_pose(self, direction, on_book, target_class, target_position, target_yaw_angle, target_angle):
        """
        获取抓取位姿
        
        参数:
            direction: 'longEdge' 或 'shortEdge'
            on_book: 是否在书本上
            target_class: 目标物体类别 ('ruler' 或 'triangle')
            target_position: 目标物体位置
            target_yaw_angle: 目标物体偏航角
            target_angle: 目标物体角度
        返回: 
            geometry_msgs/Pose 对象
        """
        try:
            # # 1. 获取书本和尺子数据
            # if not self._get_book_data() or not self._get_ruler_data():
            #     rospy.logerr("无法获取书本或尺子数据")
            #     return None

            # 验证输入方向
            if direction not in ['longEdge', 'shortEdge']:
                rospy.logerr(f"无效的推方向: {direction}，应为'longEdge'或'shortEdge'")
                return None
            
            # 2. 确定push方向 
            push_direction = direction
            yaw_angle = 0
            
            # 4. 计算垂足作为抓取位置
            if on_book:
                grasp_position_2d = geometry_utils.calculate_foot_of_perpendicular(
                    self.book_corners,
                    push_direction, 
                    target_position
                )

                grasp_position = Point()
                grasp_position.x = grasp_position_2d[0]
                grasp_position.y = grasp_position_2d[1]
                grasp_position.z = target_position.z

                yaw_angle = target_yaw_angle + geometry_utils.calculate_min_rotation(target_yaw_angle, self.book_yaw_angle)

            else:
                grasp_position_2d = geometry_utils.calculate_foot_of_perpendicular(
                    self.desktop_corners,
                    push_direction, 
                    target_position
                )

                grasp_position = Point()
                grasp_position.x = grasp_position_2d[0]
                grasp_position.y = grasp_position_2d[1]
                grasp_position.z = target_position.z

                yaw_angle = target_yaw_angle + geometry_utils.calculate_min_rotation(target_yaw_angle, self.desktop_yaw_angle)

                # if push_direction == 'longEdge':  
                #     yaw_angle = self.desktop_yaw_angle-90 # desktop_yaw_angle

                # else:  
                #     yaw_angle = self.desktop_yaw_angle

            # if push_direction == 'longEdge':  
            #     yaw_angle -= 90 # desktop_yaw_angle

            # else:  
            #     yaw_angle = yaw_angle
            
            grasp_pose0 = robot_utils.pose3d(grasp_position, yaw_angle-90)
            # grasp_pose = robot_utils.rotaionOpenGrasp2(grasp_pose0, 7.5*math.pi/180, self.rotationOpen_direction) 
            
            rospy.loginfo(f"生成{target_class}抓取位姿: 方向={direction}, 位置=({grasp_position.x:.3f}, {grasp_position.y:.3f}), yaw_angle={yaw_angle:.1f}°")
            return grasp_pose0
            
        except Exception as e:
            rospy.logerr(f"计算{target_class}抓取位姿时出错: {str(e)}")
            return None
    
    def execute(self):
        """执行尺子/三角形抓取操作序列"""
        # 步骤1: 获取视觉信息，优先处理ruler，如果没有ruler就处理triangle
        has_ruler = self._get_ruler_data()
        if not has_ruler:
            has_triangle = self._get_triangle_data()
            if not has_triangle:
                rospy.logerr("无法获取尺子或三角形数据，操作终止")
                return False
        
        # 确定目标物体类型和属性
        if has_ruler:
            target_class = 'ruler'
            target_position = self.ruler_position
            target_yaw_angle = self.ruler_yaw_angle
            target_angle = self.ruler_angle
            rospy.loginfo("检测到尺子，将执行尺子操作")
        else:
            target_class = 'triangle'
            target_position = self.triangle_position
            target_yaw_angle = self.triangle_yaw_angle
            target_angle = self.triangle_angle
            rospy.loginfo("检测到三角形，将执行三角形操作")
        
        # 步骤2: 获取书本信息
        self._get_book_data()
        self._get_desktop_data()
        
        # 步骤3: 检查目标物体是否在书本上
        on_book = self.book_corners is not None and len(self.book_corners) > 0 and geometry_utils.is_ruler_on_book(self.book_corners, target_position)
        rospy.loginfo(f"{target_class}是否在书本上: {on_book}")
        
        # 步骤4: 确定push方向
        if on_book:
            push_direction = geometry_utils.determine_push_direction(target_yaw_angle, self.book_yaw_angle)
        else:
            push_direction = geometry_utils.determine_push_direction(target_yaw_angle, self.desktop_yaw_angle)
        
        # 管理邻近物体，清除干扰
        target_data = self.object_vision_manager.get_objects_by_class_3d(target_class)
        if target_data:
            # 临时计算位姿用于管理邻近物体
            temp_push_pose0, _ = self.get_push_pose(push_direction, on_book, target_class, target_position, target_yaw_angle, target_angle)
            manage_neighborhood(target_class, temp_push_pose0)
            # 邻近物体处理可能改变场景，重新获取并更新决策
            rospy.sleep(0.3)

            # 重新获取目标物体数据
            if target_class == 'ruler':
                if not self._get_ruler_data():
                    rospy.logerr("manage_neighborhood 后无法获取尺子数据，操作终止")
                    return False
                target_position = self.ruler_position
                target_yaw_angle = self.ruler_yaw_angle
                target_angle = self.ruler_angle
            else:
                if not self._get_triangle_data():
                    rospy.logerr("manage_neighborhood 后无法获取三角形数据，操作终止")
                    return False
                target_position = self.triangle_position
                target_yaw_angle = self.triangle_yaw_angle
                target_angle = self.triangle_angle
            
            # 更新书本与桌面信息（用于on_book与方向决策）
            self._get_book_data()
            self._get_desktop_data()

            on_book = self.book_corners is not None and len(self.book_corners) > 0 and geometry_utils.is_ruler_on_book(self.book_corners, target_position)
            
            if on_book:
                push_direction = geometry_utils.determine_push_direction(target_yaw_angle, self.book_yaw_angle)
            else:
                push_direction = geometry_utils.determine_push_direction(target_yaw_angle, self.desktop_yaw_angle)
        
        # 步骤5: 生成push位姿
        push_pose0, push_pose = self.get_push_pose(push_direction, on_book, target_class, target_position, target_yaw_angle, target_angle)
        if not push_pose:
            rospy.logerr("无法生成push位姿，操作终止")
            return False
        push_pose.pose.position.z += 0.0035
        
        # 步骤7:生成抓取pose
        grasp_pose0 = self.get_grasp_pose(push_direction, on_book, target_class, target_position, target_yaw_angle, target_angle)
        if not grasp_pose0:
            rospy.logerr("无法生成抓取位姿，操作终止")
            return False
        grasp_pose0.pose.position.z += 0.010 # including difference bet. open and nature state (8mm+)
        #grasp_pose.pose.position.z += 0.0035 # 0.010
        grasp_pose = robot_utils.rotaionOpenGrasp1(grasp_pose0, 10*math.pi/180, self.rotationOpen_direction) 
        
        rospy.loginfo("移动到桌子上方位置...")
        robot_utils.go_visionHome()
        
        # 步骤6: 执行push操作
        rospy.loginfo("attainObject...")
        self.motion_primitives.attainRuler(push_pose0)

        rospy.loginfo(f"执行{target_class} push抓取操作...")
        self.motion_primitives.push0(push_pose, grasp_pose0, grasp_pose)
        
        # 步骤8: 抬起物体
        rospy.loginfo(f"抬起{target_class}...")
        self.motion_primitives.lift(0.25) #0.25

        place_pose = robot_utils.setTargetRotation(grasp_pose0)
        rospy.loginfo("place pbject...")
        self.motion_primitives.place_to_penHolder(place_pose, 0.03)
        
        # 步骤9: 移动到安全位置
        rospy.loginfo("移动到安全位置...")
        # robot_utils.go_graspHome()
        robot_utils.go_visionHome()
        
        return True

# paperPrimitive ###################################################################################################  
class PaperPrimitive: # motion primitive has not been defined 
    def __init__(self):
        """
        初始化Paper操作原语
        
        参数:
            motion_primitives: 运动原语对象
        """
        self.object_vision_manager = ObjectVisionManager()
        self.motion_primitives = motion_primitives
        self.motion_primitives.init_gripper_publisher()
        
        # 纸张属性
        self.paper_position = None
        self.paper_width = None
        self.paper_height = None
        self.paper_angle = None
        
        # 书本属性
        self.book_corners = None
        self.book_center = None   # 书本中心点
        self.book_width = None    # 书本宽度
        self.book_height = None   # 书本高度
        self.book_angle = None    # 书本角度
        self.book_avg_z = None    # 书本平均高度
    
    def _get_paper_data(self):
        """获取Paper的视觉数据"""
        try:
            paper_data = self.object_vision_manager.get_objects_by_class_3d('paper')
            if not paper_data:
                rospy.logerr("未检测到paper!")
                return False

            paper_obj = paper_data[0][0]
            if paper_obj.is_polygon and len(paper_data[0][1]) >= 4:
                corners = paper_data[0][1]   
                
                # 计算最小外接矩形
                center, width, height, angle = geometry_utils.calculate_min_area_rect(corners) # meter
                
                # 创建3D点对象
                self.paper_position = Point(x=center[0], y=center[1], z=center[2])
                self.paper_width = width
                self.paper_height = height
                self.paper_angle = angle
                
                rospy.loginfo(f"检测到Paper: 位置({center[0]:.3f}, {center[1]:.3f}), "
                            f"角度: {angle:.1f}°, 尺寸: {width:.2f}x{height:.2f}")
                return True
            else:
                rospy.logwarn("Paper信息不完整，无法获取角点")
                return False
        except Exception as e:
            rospy.logerr(f"获取Paper数据时出错: {str(e)}")
            return False
    
    def _get_book_data(self):
        """获取书本的视觉数据并计算最小外接矩形"""
        try:
            book_data = self.object_vision_manager.get_objects_by_class_3d('book')
            if not book_data:
                rospy.logwarn("未检测到书本!")
                self.book_corners = None
                return False

            # 获取书本的角点信息
            book_obj = book_data[0][0]
            if book_obj.is_polygon and len(book_data[0][1]) >= 4:
                self.book_corners = book_data[0][1]   
            # book_obj = book_data[0][0]
            # if book_obj.is_polygon and len(book_obj.polygon_corners) >= 4:
            #     self.book_corners = [point for point in book_obj.polygon_corners]
                
                # # 计算书本最小外接矩形
                # corners_2d = np.array([[p.x, p.y] for p in self.book_corners])
                # center_2d, width, height, angle = geometry_utils.calculate_min_area_rect(corners_2d)
                
                # # 计算书本的平均高度
                # z_avg = np.mean([p.z for p in self.book_corners])

                # 计算最小外接矩形
                center_3d, width, height, angle = geometry_utils.calculate_min_area_rect(self.book_corners) # meter
                
                # 存储书本属性
                self.book_center = Point(x=center_3d[0], y=center_3d[1], z=center_3d[2])
                self.book_width = width
                self.book_height = height
                self.book_angle = angle
                self.book_avg_z = center_3d[2]
                
                rospy.loginfo(f"书本: 位置({center_3d[0]:.3f}, {center_3d[1]:.3f}), "
                            f"角度: {angle:.1f}°, 尺寸: {width:.2f}x{height:.2f}, 高度: {center_3d[2]:.3f}")
                return True
            else:
                rospy.logwarn("书本信息不完整，无法获取角点")
                self.book_corners = None
                return False
        except Exception as e:
            rospy.logerr(f"获取书本数据时出错: {str(e)}")
            self.book_corners = None
            return False
        
    def get_paperGraspPose(self):
        """
        计算抓取纸张的位姿
        抓取位置: 纸张上距离书本较远的长边中心点
        抓取姿态: 基于纸张角度确定，使夹爪平行于纸张长边
        """
        # if not self._get_paper_data():
        #     rospy.logerr("无法获取纸张数据")
        #     return None
            
        # if not self._get_book_data():
        #     rospy.logwarn("书本数据缺失，使用默认抓取方向")
        #     book_center_2d = np.array([0, 0])  # 默认坐标系中心
        # else:
        #     book_center_2d = np.array([self.book_center.x, self.book_center.y])

        book_center_2d = np.array([self.book_center.x, self.book_center.y])

        # 纸张参数
        paper_center = np.array([self.paper_position.x, self.paper_position.y])
        paper_angle_rad = np.radians(self.paper_angle)
        
        # 计算纸张两个长边的中心点
        edge1, edge2, is_width_longer = geometry_utils.calculate_edge_centers(
            center=paper_center,
            width=self.paper_width,
            height=self.paper_height,
            angle=paper_angle_rad # need raw data from the minAreaRect 
        )
        
        # 选择距离书本中心较远的边 # 修改为较近的点为起点
        dist1 = np.linalg.norm(edge1 - book_center_2d)
        dist2 = np.linalg.norm(edge2 - book_center_2d)
        grasp_point_2d = edge1 if dist1 < dist2 else edge2
        
        # 计算抓取姿态的偏航角
        # yaw_angle = paper_angle_rad # if is_width_longer else paper_angle_rad + np.pi/2
        
        # 创建抓取位姿
        # grasp_pose = Pose()
        grasp__position = Point(
            x=grasp_point_2d[0], 
            y=grasp_point_2d[1], 
            z=self.paper_position.z
        )
        
        # q = tf.transformations.quaternion_from_euler(0, 0, yaw_angle)
        # grasp_pose.orientation = Quaternion(*q)

        # 计算姿态的偏航角
        if is_width_longer:
            yaw_angle = self.paper_angle + 90 # if is_width_longer else book_angle_rad + np.pi/2
        else:
            yaw_angle = self.paper_angle

        grasp_pose = robot_utils.pose3d(grasp__position, yaw_angle-90)
        
        rospy.loginfo(f"抓取位姿计算完成: 位置({grasp__position.x:.3f}, {grasp__position.y:.3f}), 角度: {yaw_angle:.1f}°")
        return grasp_pose

    def get_placePose(self):
        """
        计算放置纸张的位姿
        放置位置: 书本上edge1和edge2连线上的十等分位置处
        放置姿态: 基于书本角度确定
        """
        # if not self._get_book_data():
        #     rospy.logerr("无法获取书本数据")
        #     return None
            
        # # 获取原始纸张中心位置
        # if not self._get_paper_data():
        #     rospy.logwarn("纸张数据缺失，使用默认位置")
        #     paper_center = np.array([0.5, 0.5])  # 默认位置
        # else:
        #     paper_center = np.array([self.paper_position.x, self.paper_position.y])

        paper_center = np.array([self.paper_position.x, self.paper_position.y])
        
        # 书本参数
        book_center_2d = np.array([self.book_center.x, self.book_center.y])
        book_angle_rad = np.radians(self.book_angle)
        
        # 计算书本两个长边的中心点
        edge1, edge2, is_width_longer = geometry_utils.calculate_edge_centers(
            center=book_center_2d,
            width=self.book_width,
            height=self.book_height,
            angle=book_angle_rad # need raw data from the minAreaRect
        )
        
        # 计算在edge1和edge2连线上的十等分位置
        # 选择距离原始纸张位置较近的边作为起点 # 当纸张在右侧时，修改为较远的点为起点
        dist1 = np.linalg.norm(edge1 - paper_center)
        dist2 = np.linalg.norm(edge2 - paper_center)
        
        if dist1 > dist2:
            start_edge = edge1
            end_edge = edge2
        else:
            start_edge = edge2
            end_edge = edge1
        
        # 计算十等分位置（选择第5个等分点，即中间位置）
        t = 0.1  # 0.0 是 start_edge, 1.0 是 end_edge, 0.5 是中间
        place_point_2d = start_edge + t * (end_edge - start_edge)
        
        # 创建放置位姿
        # place_pose = Pose()
        place_position = Point(
            x=place_point_2d[0], 
            y=place_point_2d[1], 
            z=self.book_avg_z  
        )

        # 计算放置姿态的偏航角
        if is_width_longer:
            yaw_angle = self.book_angle + 90 # if is_width_longer else book_angle_rad + np.pi/2
        else:
            yaw_angle = self.book_angle
        
        # q = tf.transformations.quaternion_from_euler(0, 0, yaw_angle)
        # place_pose.orientation = Quaternion(*q)

        place_pose = robot_utils.pose3d(place_position, yaw_angle-90)
        
        rospy.loginfo(f"放置位姿计算完成: 位置({place_position.x:.3f}, {place_position.y:.3f}), 角度: {yaw_angle:.1f}°")
        # rospy.loginfo(f"十等分位置: t={t:.1f}, 在edge1({edge1[0]:.3f}, {edge1[1]:.3f})和edge2({edge2[0]:.3f}, {edge2[1]:.3f})连线上")
        return place_pose

    def execute(self):

        """执行纸张操作序列"""
        if not self._get_paper_data():
            return False
        if not self._get_book_data(): # change the eraser data ??? deepcopy???
            return False
        
        # 管理邻近物体，清除干扰
        paper_data = self.object_vision_manager.get_objects_by_class_3d('paper')
        if paper_data:
            # 临时计算位姿用于管理邻近物体
            temp_paper_graspPose = self.get_paperGraspPose()
            manage_neighborhood('paper', temp_paper_graspPose)
            # 邻近物体处理可能改变位姿，重新获取paper与book数据
            rospy.sleep(0.3)
            if not self._get_paper_data():
                rospy.logerr("manage_neighborhood 后无法获取纸张数据，操作终止")
                return False
            if not self._get_book_data():
                rospy.logerr("manage_neighborhood 后无法获取书本数据，操作终止")
                return False
            
        try:
            # 确保发布器已初始化
            if self.motion_primitives.gripper_pub is None:
                self.motion_primitives.init_gripper_publisher()

            rospy.loginfo("move the arm...")
            # robot_utils.goHome()
            self.motion_primitives.gripper_pub.publish("close")
            self.motion_primitives.gripper_pub.publish("relax_close")

            rospy.loginfo("移动到桌子上方位置...")
            robot_utils.go_visionHome()

            paper_graspPose = self.get_paperGraspPose()
            rospy.loginfo("接近物体...")
            self.motion_primitives.attainObject(paper_graspPose)

            # # adjusted_pose = deepcopy(self.eraser_pose)
            # # adjusted_pose.pose.position.z = self.eraser_pose.pose.position.z + 0.02

            rospy.loginfo("尝试抓取物体...")
            self.motion_primitives.grasp_paper(paper_graspPose)

            rospy.loginfo("抬起物体...")
            self.motion_primitives.lift()

            rospy.loginfo("place object...")
            # robot_utils.go_visionHome()

            placePose = self.get_placePose()
            rospy.loginfo("place pbject...")
            self.motion_primitives.place(placePose)
            self.motion_primitives.lift()

            rospy.loginfo("返回初始位置...")
            # robot_utils.go_graspHome()
            robot_utils.go_visionHome()
            
            return True
        except Exception as e:
            rospy.logerr(f"执行橡皮擦操作时出错: {str(e)}")
            return False

# BookPrimitive ###################################################################################################      
  
class BookPrimitive: # # motion primitive does not been defined
    def __init__(self):
        """
        初始化Book操作原语
        """
        self.object_vision_manager = ObjectVisionManager()
        self.motion_primitives = motion_primitives  # 假设已定义
        self.motion_primitives.init_gripper_publisher()
        
        # 左侧书本属性
        self.left_book_center = None
        self.left_book_width = None
        self.left_book_height = None
        self.left_book_angle = None
        self.left_book_avg_z = None
        self.left_book_thickness = 0.015 #0.0025 #0.0065 #0.009 #0.012 #0.025
        self.left_book_pryAngle = math.asin(self.left_book_thickness/0.095) # width of the book is 95mm
        
        # 右侧书本属性
        self.right_book_center = None
        self.right_book_width = None
        self.right_book_height = None
        self.right_book_angle = None
        self.right_book_avg_z = None
    
    def _get_book_data(self):
        """获取两本书的视觉数据并计算最小外接矩形"""
        try:
            book_data = self.object_vision_manager.get_objects_by_class_3d('book')
            if not book_data or len(book_data) < 2:
                rospy.logwarn("未检测到足够的书本! 需要至少两本书")
                return False
                
            # 按3D相机坐标系的x均值排序（左到右）
            # b 结构: (ObjectInfo, points_3d_list)
            # 使用每本书3D点列表的x均值作为排序键；若3D点缺失，则置为无穷大，排在后面
            sorted_books = sorted(
                book_data,
                key=lambda b: (np.mean([p.x for p in b[1]]) if b[1] else float('inf'))
            )
            
            # 处理左侧书
            left_book = sorted_books[0][0]
            if left_book.is_polygon and len(left_book.polygon_corners) >= 4:
                # corners_2d = np.array([[p.x, p.y] for p in left_book.polygon_corners])
                center_3d, width, height, angle = geometry_utils.calculate_min_area_rect(sorted_books[0][1])
                
                self.left_book_center = Point(x=center_3d[0], y=center_3d[1], z=center_3d[2])
                self.left_book_width = width
                self.left_book_height = height
                self.left_book_angle = angle
                self.left_book_avg_z = center_3d[2]
                
                rospy.loginfo(f"左侧书: 位置({center_3d[0]:.3f}, {center_3d[1]:.3f}), "
                            f"角度: {angle:.1f}°, 尺寸: {width:.2f}x{height:.2f}")
            else:
                rospy.logwarn("左侧书信息不完整")
                return False
                
            # 处理右侧书
            right_book = sorted_books[1][0]
            if right_book.is_polygon and len(right_book.polygon_corners) >= 4:
                # corners_2d = np.array([[p.x, p.y] for p in right_book.polygon_corners])
                center_3d, width, height, angle = geometry_utils.calculate_min_area_rect(sorted_books[1][1])
                
                self.right_book_center = Point(x=center_3d[0], y=center_3d[1], z=center_3d[2])
                self.right_book_width = width
                self.right_book_height = height
                self.right_book_angle = angle
                self.right_book_avg_z = center_3d[2]
                
                rospy.loginfo(f"右侧书: 位置({center_3d[0]:.3f}, {center_3d[1]:.3f}), "
                            f"角度: {angle:.1f}°, 尺寸: {width:.2f}x{height:.2f}")
            else:
                rospy.logwarn("右侧书信息不完整")
                return False
                
            return True
        except Exception as e:
            rospy.logerr(f"获取书本数据时出错: {str(e)}")
            return False
        
    def get_pryPose(self):
        """
        计算撬动位姿（左侧书靠近右侧书的边中心点）
        返回: geometry_msgs/Pose 对象
        """
        # if not self._get_book_data():
        #     rospy.logerr("无法获取书本数据")
        #     return None
            
        # 左侧书参数
        left_center = np.array([self.left_book_center.x, self.left_book_center.y])
        left_angle_rad = np.radians(self.left_book_angle)
        
        # 计算左侧书两个长边的中心点
        edge1, edge2, is_width_longer = geometry_utils.calculate_edge_centers(
            center=left_center,
            width=self.left_book_width,
            height=self.left_book_height,
            angle=left_angle_rad
        )
        
        # 右侧书中心点作为参考
        right_center = np.array([self.right_book_center.x, self.right_book_center.y])
        
        # 选择距离右侧书较近的边
        dist1 = np.linalg.norm(edge1 - right_center)
        dist2 = np.linalg.norm(edge2 - right_center)
        closer_edge = edge1 if dist1 < dist2 else edge2
        
        # 计算edge1和edge2连线上超出closer_edge十等分点
        # 计算从edge1到edge2的向量
        edge_vector = edge2 - edge1
        edge_length = np.linalg.norm(edge_vector)
        
        # 计算偏移距离
        tenth_distance = edge_length / 50
        
        # 确定延伸方向：从closer_edge向远离中心的方向延伸
        # 如果closer_edge是edge1，则向edge2方向延伸；如果closer_edge是edge2，则向edge1方向延伸
        if np.array_equal(closer_edge, edge1):
            # closer_edge是edge1，向edge2方向延伸
            direction = -edge_vector / edge_length
            pry_point_2d = closer_edge + direction * tenth_distance
        else:
            # closer_edge是edge2，向edge1方向延伸
            direction = +edge_vector / edge_length
            pry_point_2d = closer_edge + direction * tenth_distance
        
        # 计算撬动姿态的偏航角
        # yaw_angle = left_angle_rad + np.pi/2 if is_width_longer else left_angle_rad 
        
        # 创建撬动位姿
        # pry_pose = Pose()
        pry_position = Point(
            x=pry_point_2d[0], 
            y=pry_point_2d[1], 
            z=self.left_book_avg_z + self.left_book_thickness # 书本厚度
        )
        
        # q = tf.transformations.quaternion_from_euler(0, 0, yaw_angle)
        # pry_pose.orientation = Quaternion(*q)

        # 计算姿态的偏航角
        if is_width_longer:
            yaw_angle = self.left_book_angle + 90 # if is_width_longer else book_angle_rad + np.pi/2
        else:
            yaw_angle = self.left_book_angle

        left_book_pose = robot_utils.pose3d(pry_position, yaw_angle-90)
        pry_pose = robot_utils.rotaionOpen(left_book_pose, self.left_book_pryAngle, True) # 7.5*math.pi/180
        
        # rospy.loginfo(f"撬动位姿计算完成: 位置({pry_position.x:.3f}, {pry_position.y:.3f}), 角度: {yaw_angle:.1f}°")
        rospy.loginfo(f"撬动位姿计算完成: 位置({pry_position.x:.3f}, {pry_position.y:.3f}), "
              f"角度: {yaw_angle:.1f}°, 撬动角: {math.degrees(self.left_book_pryAngle):.1f}°")
        return left_book_pose, pry_pose

    def get_placePose(self):
        """
        计算放置位姿（右侧书两条长边中心点）
        - place_pose2: 距离左侧书较近的那条边（过渡位姿）
        - place_pose: 距离左侧书较远的那条边（最终位姿）
        返回: (place_pose2, place_pose)
        """
        # if not self._get_book_data():
        #     rospy.logerr("无法获取书本数据")
        #     return None
            
        # 右侧书参数
        right_center = np.array([self.right_book_center.x, self.right_book_center.y])
        right_angle_rad = np.radians(self.right_book_angle)
        
        # 计算右侧书两个长边的中心点
        edge1, edge2, is_width_longer = geometry_utils.calculate_edge_centers(
            center=right_center,
            width=self.right_book_width,
            height=self.right_book_height,
            angle=right_angle_rad
        )
        
        # 左侧书中心点作为参考
        left_center = np.array([self.left_book_center.x, self.left_book_center.y])
        
        # 计算与左侧书的距离
        dist1 = np.linalg.norm(edge1 - left_center)
        dist2 = np.linalg.norm(edge2 - left_center)

        # 选择两种边：
        # - place_point_2d2 为距离左侧书更近的边（过渡位姿）
        # - place_point_2d 为距离左侧书更远的边（最终位姿）
        place_point_2d2 = edge1 if dist1 < dist2 else edge2
        place_point_2d = edge1 if dist1 > dist2 else edge2
        
        # 计算放置姿态的偏航角
        # yaw_angle = right_angle_rad if is_width_longer else right_angle_rad + np.pi/2

        # 创建放置位姿 - 过渡位姿
        place_position0 = Point(
            x=place_point_2d2[0],
            y=place_point_2d2[1],
            z=self.right_book_avg_z - 0.010 # 0.010 # 0.020 # 略高于书本表面 pull-over the book
        )
        
        # 创建放置位姿 - 最终位姿
        place_position1 = Point(
            x=place_point_2d[0], 
            y=place_point_2d[1], 
            z=self.right_book_avg_z - 0.020 # 0.020 # 0.030 # 略高于书本表面 press-pull
        )

        # q = tf.transformations.quaternion_from_euler(0, 0, yaw_angle)
        # place_pose.orientation = Quaternion(*q)

        # 计算姿态的偏航角
        if is_width_longer:
            yaw_angle = self.right_book_angle + 90 # if is_width_longer else book_angle_rad + np.pi/2
        else:
            yaw_angle = self.right_book_angle

        place_pose0 = robot_utils.pose3d(place_position0, yaw_angle-90)
        place_pose1 = robot_utils.pose3d(place_position1, yaw_angle-90)

        rospy.loginfo(f"放置位姿计算完成: 过渡位姿({place_position0.x:.3f}, {place_position0.y:.3f}), 最终位姿({place_position1.x:.3f}, {place_position1.y:.3f}), 角度: {yaw_angle:.1f}°")
        return place_pose0, place_pose1

    def execute(self):
        """执行书本堆叠操作序列"""
        if not self._get_book_data():
            rospy.logerr("无法获取书本数据")
            return False
        
        # 管理邻近物体，清除干扰
        book_data = self.object_vision_manager.get_objects_by_class_3d('book')
        if book_data:
            # 临时计算位姿用于管理邻近物体
            temp_book1_pose, _ = self.get_pryPose()
            manage_neighborhood('book', temp_book1_pose)

            if not self._get_book_data():
                rospy.logerr("manage_neighborhood 后无法获取书本数据，操作终止")
                return False
                
        try:
            # 确保发布器已初始化
            if self.motion_primitives.gripper_pub is None:
                self.motion_primitives.init_gripper_publisher()

            rospy.loginfo("移动机械臂到初始位置...")
            # robot_utils.goHome()
            self.motion_primitives.gripper_pub.publish("close")
            self.motion_primitives.gripper_pub.publish("relax_close")

            rospy.loginfo("移动到桌子上方位置...")
            robot_utils.go_visionHome()

            # 重新计算所需的位姿（因为manage_neighborhood可能改变了物体位置）
            book1_pose, pryPose = self.get_pryPose()
            placePose0, placePose1 = self.get_placePose()

            rospy.loginfo("接近左侧书本...")
            self.motion_primitives.attainObject(book1_pose)

            rospy.loginfo("尝试撬动书本...") 
            self.motion_primitives.pry(pryPose)

            rospy.loginfo("微微抬起物体...")
            # self.motion_primitives.lift(0.005)

            rospy.loginfo("拖动书本到右侧书本过渡位置...,继续拖动书本到最终位置...")
            self.motion_primitives.pull(placePose0, placePose1)

            # rospy.loginfo("继续拖动书本到最终位置...")
            # self.motion_primitives.pull(placePose)

            rospy.loginfo("返回初始位置...")
            # robot_utils.go_graspHome()
            robot_utils.go_visionHome()
            
            return True
        except Exception as e:
            rospy.logerr(f"执行书本堆叠操作时出错: {str(e)}")
            return False

# ReorientDeformablePrimitive ###################################################################################################
class ReorientDeformablePrimitive:
    def __init__(self):
        """
        初始化可变形物体重新定向操作原语
        """
        self.object_vision_manager = ObjectVisionManager()
        self.motion_primitives = motion_primitives
        self.motion_primitives.init_gripper_publisher()
        
    def execute(self, obj_data):
        """
        执行可变形物体的重新定向操作
        
        参数:
            obj_data: 包含(obj_info, pts3d)的元组
        """
        try:
            obj_info, pts3d = obj_data
            
            if not pts3d or len(pts3d) < 3:
                rospy.logerr("物体3D点数据不足，无法执行重新定向")
                return False
                
            rospy.loginfo(f"开始执行可变形物体重新定向: {obj_info.class_name}")
            
            # 计算物体的最小外接矩形
            center_3d, width, height, angle = geometry_utils.calculate_min_area_rect(pts3d)
            
            # 定义连续的object_yaw_angle
            # object_yaw_angle = angle
            # 计算姿态角度
            if width > height:
                object_yaw_angle = angle + 90
            else:
                object_yaw_angle = angle
            
            rospy.loginfo(f"物体中心: ({center_3d[0]:.3f}, {center_3d[1]:.3f}, {center_3d[2]:.3f})")
            rospy.loginfo(f"物体尺寸: {width:.3f} x {height:.3f}")
            rospy.loginfo(f"物体角度: {angle:.1f}°")
            rospy.loginfo(f"object_yaw_angle: {object_yaw_angle:.1f}°")
            
            # 根据object_yaw_angle的符号确定推动策略
            if object_yaw_angle > 0:
                # 逆时针转动：推动点为左长边的下角点
                rospy.loginfo("执行逆时针转动策略")
                push_point, push_orientation = self._calculate_push_pose_ccw(
                    center_3d, width, height, object_yaw_angle
                )
                rotation_direction = "ccw"  # 逆时针
            else:
                # 顺时针转动：推动点为左长边的上角点
                rospy.loginfo("执行顺时针转动策略")
                push_point, push_orientation = self._calculate_push_pose_cw(
                    center_3d, width, height, object_yaw_angle
                )
                rotation_direction = "cw"  # 顺时针
            
            # 调用motion_primitives中的reorientation函数
            success = self.motion_primitives.reorientation(
                push_point, push_orientation, rotation_direction, object_yaw_angle, center_3d, width, height
            )
            
            if success:
                rospy.loginfo("可变形物体重新定向完成")
            else:
                rospy.logwarn("可变形物体重新定向失败")
                
            return success
            
        except Exception as e:
            rospy.logerr(f"执行可变形物体重新定向时出错: {str(e)}")
            return False
    
    def _calculate_push_pose_ccw(self, center_3d, width, height, object_yaw_angle):
        """
        计算逆时针转动的推动位姿（左长边的下角点）
        
        参数:
            center_3d: 物体中心点 (x, y, z)
            width: 物体宽度
            height: 物体高度
            object_yaw_angle: 物体角度（度）
            
        返回:
            push_point: 推动点
            push_orientation: 推动姿态角（度）
        """
        # 将角度转换为弧度
        angle_rad = np.radians(object_yaw_angle) # 逆时针转动，角度为负值
        
        # 计算左长边的下角点
        # 假设width > height，左长边为width边
        if width > height:
            # 左长边的下角点
            left_edge_x = center_3d[0] - width/2 * np.cos(angle_rad)
            left_edge_y = center_3d[1] - width/2 * np.sin(angle_rad)
            # 下角点（向下的偏移）
            push_x = left_edge_x - height/2 * np.sin(angle_rad)
            push_y = left_edge_y + height/2 * np.cos(angle_rad)
        else:
            # 左长边的下角点
            left_edge_x = center_3d[0] - height/2 * np.cos(angle_rad)
            left_edge_y = center_3d[1] - height/2 * np.sin(angle_rad)
            # 下角点（向下的偏移）
            push_x = left_edge_x - width/2 * np.sin(angle_rad)
            push_y = left_edge_y + width/2 * np.cos(angle_rad)
        
        push_point = Point(x=push_x, y=push_y, z=center_3d[2])
        
        # 推动姿态角垂直于物体姿态角
        push_orientation = object_yaw_angle
        
        rospy.loginfo(f"逆时针推动点: ({push_x:.3f}, {push_y:.3f}, {center_3d[2]:.3f})")
        rospy.loginfo(f"推动姿态角: {push_orientation:.1f}°")
        
        return push_point, push_orientation
    
    def _calculate_push_pose_cw(self, center_3d, width, height, object_yaw_angle):
        """
        计算顺时针转动的推动位姿（左长边的上角点）
        
        参数:
            center_3d: 物体中心点 (x, y, z)
            width: 物体宽度
            height: 物体高度
            object_yaw_angle: 物体角度（度）
            
        返回:
            push_point: 推动点
            push_orientation: 推动姿态角（度）
        """
        # 将角度转换为弧度
        angle_rad = np.radians(object_yaw_angle) # 顺时针转动，角度为正值
        
        # 计算左长边的上角点
        # 假设width > height，左长边为width边
        if width > height:
            # 左长边的上角点
            left_edge_x = center_3d[0] - width/2 * np.cos(angle_rad)
            left_edge_y = center_3d[1] - width/2 * np.sin(angle_rad)
            # 上角点（向上的偏移）
            push_x = left_edge_x + height/2 * np.sin(angle_rad)
            push_y = left_edge_y - height/2 * np.cos(angle_rad)
        else:
            # 左长边的上角点
            left_edge_x = center_3d[0] - height/2 * np.cos(angle_rad)
            left_edge_y = center_3d[1] - height/2 * np.sin(angle_rad)
            # 上角点（向上的偏移）
            push_x = left_edge_x + width/2 * np.sin(angle_rad)
            push_y = left_edge_y - width/2 * np.cos(angle_rad)
        
        push_point = Point(x=push_x, y=push_y, z=center_3d[2])
        
        # 推动姿态角垂直于物体姿态角
        push_orientation = object_yaw_angle
        
        rospy.loginfo(f"顺时针推动点: ({push_x:.3f}, {push_y:.3f}, {center_3d[2]:.3f})")
        rospy.loginfo(f"推动姿态角: {push_orientation:.1f}°")
        
        return push_point, push_orientation

# 创建reorient_deformable实例，供其他文件导入
# reorient_deformable = ReorientDeformablePrimitive()
