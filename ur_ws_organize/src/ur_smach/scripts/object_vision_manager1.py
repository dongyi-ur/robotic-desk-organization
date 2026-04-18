#!/usr/bin/python

import rospy
import threading
import numpy as np
import message_filters
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from object_keypoint_msgs.msg import ObjectInfo, ObjectInfoArray
from geometry_msgs.msg import PointStamped, Point

class ObjectVisionManager:
    def __init__(self):
        # 定义所有可能的物体类别
        self.CLASS_NAMES = ["eraser", "pen", "paper", "box", "book", 
                           "ruler", "lead_case", "triangle", "desktop"] # "pen holder",
        
        # 初始化物体存储字典
        self.object_dict = {cls: [] for cls in self.CLASS_NAMES}
        self.object_3d_dict = {cls: [] for cls in self.CLASS_NAMES}  # 存储三维坐标
        
        # 线程安全的锁，防止同时访问字典
        self.lock = threading.RLock()
        self.cv_bridge = CvBridge()
        
        # 相机参数
        self.camera_info = None
        self.depth_image = None
        self.last_desktop_msg = None
        
        # 订阅相机信息
        rospy.Subscriber("/camera/aligned_depth_to_color/camera_info", CameraInfo, self.camera_info_callback)
        self.depth_sub = message_filters.Subscriber("/camera/aligned_depth_to_color/image_raw", Image)
        self.object_sub = message_filters.Subscriber("/object_detection/info", ObjectInfoArray)
        
        # 使用消息过滤器同步深度图像和物体检测信息
        ts = message_filters.ApproximateTimeSynchronizer(
            [self.depth_sub, self.object_sub], queue_size=10, slop=0.1)
        ts.registerCallback(self.synced_callback)
        
        # 订阅笔筒位置
        # rospy.Subscriber("/pen_holder/position", PointStamped, self.pen_holder_callback)
        rospy.Subscriber("/desktop/detection", ObjectInfoArray, self.desktop_callback)
        
        # self.last_pen_holder_msg = None
        # self.desktop_msg = None
        rospy.loginfo("物体视觉管理器已启动，等待数据...")

    def camera_info_callback(self, msg):
        """存储相机内参"""
        with self.lock:
            self.camera_info = msg
            rospy.loginfo_once("已接收相机内参信息")

    def synced_callback(self, depth_msg, object_msg):
        """处理同步的深度图像和物体检测信息"""
        with self.lock:
            # 转换深度图像
            try:
                self.depth_image = self.cv_bridge.imgmsg_to_cv2(depth_msg, "passthrough")
            except Exception as e:
                rospy.logerr(f"深度图像转换错误: {e}")
                return
            
            # 处理物体检测信息
            self.process_object_info(object_msg)

    def process_object_info(self, msg):
        """处理物体信息并进行坐标转换"""
        # 清空物体信息
        self.object_dict = {cls: [] for cls in self.CLASS_NAMES}
        self.object_3d_dict = {cls: [] for cls in self.CLASS_NAMES}
        
        # 添加新的物体信息(不包括笔筒)
        for obj in msg.objects:
            if obj.class_name in self.CLASS_NAMES and obj.class_name != "desktop":
                self.object_dict[obj.class_name].append(obj)
                # 转换坐标
                self.convert_object_coordinates(obj)
        
        # 处理笔筒
        # if self.last_pen_holder_msg:
        #     pen_holder = ObjectInfo()
        #     pen_holder.class_name = "pen holder"
        #     pen_holder.is_polygon = False
        #     pen_holder.rect_center = self.last_pen_holder_msg.point
        #     self.object_dict["pen holder"] = [pen_holder]
        #     self.convert_pen_holder_coordinates(pen_holder)

        # 处理桌面
        if self.last_desktop_msg:
            desktop = ObjectInfo()
            desktop = self.last_desktop_msg.objects[0]

            self.object_dict["desktop"] = [desktop]
            self.convert_desktop_coordinates(desktop)

            # 假设 self.object_3d_dict 已经填充了数据
            # desktop_points = self.object_3d_dict["desktop"]

            # 打印所有三维点
            # for frame_points in desktop_points:
            #     for point in frame_points:
            #         print(f"Point: x={point.x:.3f}, y={point.y:.3f}, z={point.z:.3f}")
        
        # rospy.loginfo(f"更新了{len(msg.objects)}个物体的视觉信息")
        # self.print_objects_info()

    def validate_depth(self, depth_value, u, v, is_polygon=False, polygon_depths=None):
        """验证深度值是否有效"""
        if np.isnan(depth_value) or np.isinf(depth_value):
            rospy.logdebug(f"无效深度值(NaN/Inf)在像素({u},{v})")
            return False
        
        if depth_value <= 0.1 or depth_value > 10.0:  # 典型RGB-D相机有效范围
            rospy.logdebug(f"深度值超出范围({depth_value:.3f}m)在像素({u},{v})")
            return False
            
        if is_polygon and polygon_depths:
            # 检查多边形点之间的深度一致性
            avg_depth = np.mean(polygon_depths)
            if abs(depth_value - avg_depth) > 0.5:  # 50cm差异阈值
                rospy.logdebug(f"深度不一致({depth_value:.3f}m vs avg {avg_depth:.3f}m)在像素({u},{v})")
                return False
                
        return True

    def convert_object_coordinates(self, obj):
        """转换物体坐标到相机坐标系"""
        if not self.camera_info or self.depth_image is None:
            rospy.logwarn("相机信息或深度图像未准备好")
            return
        
        # 获取相机内参
        K = np.array(self.camera_info.K).reshape(3, 3)
        fx, fy = K[0, 0], K[1, 1]
        cx, cy = K[0, 2], K[1, 2]
        
        points_3d = []
        polygon_depths = [] if obj.is_polygon else None
        
        if obj.is_polygon:
            # 计算多边形所有角点的像素中心
            center_u = int(np.mean([corner.x for corner in obj.polygon_corners]))
            center_v = int(np.mean([corner.y for corner in obj.polygon_corners]))
            
            # 获取中心点对应的深度值
            center_depth = None
            if 0 <= center_u < self.depth_image.shape[1] and 0 <= center_v < self.depth_image.shape[0]:
                center_depth = self.depth_image[center_v, center_u] / 1000.0  # 转换为米
            
            # 如果中心点深度无效，尝试从其他角点获取有效深度
            if center_depth is None or center_depth == 0:
                for corner in obj.polygon_corners:
                    u, v = int(corner.x), int(corner.y)
                    if 0 <= u < self.depth_image.shape[1] and 0 <= v < self.depth_image.shape[0]:
                        temp_depth = self.depth_image[v, u] / 1000.0
                        if temp_depth > 0:
                            center_depth = temp_depth
                            break
            
            # 处理多边形角点，所有角点使用相同的深度值
            if center_depth and center_depth > 0:
                for corner in obj.polygon_corners:
                    u, v = int(corner.x), int(corner.y)
                    if 0 <= u < self.depth_image.shape[1] and 0 <= v < self.depth_image.shape[0]:
                        x = (u - cx) * center_depth / fx
                        y = (v - cy) * center_depth / fy
                        z = center_depth
                        points_3d.append(Point(x, y, z))
                        polygon_depths.append(center_depth)
        else:
            # 处理矩形中心点
            u, v = int(obj.rect_center.x), int(obj.rect_center.y)
            if 0 <= u < self.depth_image.shape[1] and 0 <= v < self.depth_image.shape[0]:
                depth = self.depth_image[v, u] / 1000.0  # 转换为米
                if self.validate_depth(depth, u, v):
                    x = (u - cx) * depth / fx
                    y = (v - cy) * depth / fy
                    z = depth
                    points_3d.append(Point(x, y, z))
        
        # 存储转换后的3D点
        self.object_3d_dict[obj.class_name].append(points_3d)

    def convert_desktop_coordinates(self, obj):
        """转换物体坐标到相机坐标系"""
        if not self.camera_info or self.depth_image is None:
            rospy.logwarn("相机信息或深度图像未准备好")
            return

        # 获取相机内参
        K = np.array(self.camera_info.K).reshape(3, 3)
        fx, fy = K[0, 0], K[1, 1]
        cx, cy = K[0, 2], K[1, 2]

        # # ---------- 新增：计算 ruler 的平均 z ----------
        # ruler_avg_z = None
        # if 'ruler' in self.object_3d_dict and self.object_3d_dict['ruler']:
        #     first_ruler_points = self.object_3d_dict['ruler'][0]
        #     if first_ruler_points:
        #         ruler_avg_z = sum(p.z for p in first_ruler_points) / len(first_ruler_points)

        # if ruler_avg_z is None:
        #     # rospy.logwarn("未找到 ruler，无法确定 desktop 深度")
        #     return
        # # ---------------------------------------------

        # ---------- 修改：计算 ruler 或 triangle 的平均 z ----------
        ruler_avg_z = None
        triangle_avg_z = None

        # 先尝试获取 ruler 的平均深度
        if 'ruler' in self.object_3d_dict and self.object_3d_dict['ruler']:
            first_ruler_points = self.object_3d_dict['ruler'][0]
            if first_ruler_points:
                ruler_avg_z = sum(p.z for p in first_ruler_points) / len(first_ruler_points)

        # 如果 ruler 不存在，尝试获取 triangle 的平均深度
        if ruler_avg_z is None:
            if 'triangle' in self.object_3d_dict and self.object_3d_dict['triangle']:
                first_triangle_points = self.object_3d_dict['triangle'][0]
                if first_triangle_points:
                    triangle_avg_z = sum(p.z for p in first_triangle_points) / len(first_triangle_points)

        # 确定最终使用的深度值
        depth_to_use = ruler_avg_z if ruler_avg_z is not None else triangle_avg_z

        if depth_to_use is None:
            #rospy.logwarn("未找到 ruler和triangle，无法确定 desktop 深度")
            return
        # ---------------------------------------------

        points_3d = []
        polygon_depths = []

        for corner in obj.polygon_corners:
            u, v = int(corner.x), int(corner.y)
            if 0 <= u < self.depth_image.shape[1] and 0 <= v < self.depth_image.shape[0]:
                depth = depth_to_use          # 直接采用 ruler 的平均 z
                if self.validate_depth(depth, u, v, True, polygon_depths):
                    x = (u - cx) * depth / fx
                    y = (v - cy) * depth / fy
                    z = depth
                    points_3d.append(Point(x, y, z))
                    polygon_depths.append(depth)

        # 存储转换后的3D点
        self.object_3d_dict[obj.class_name].append(points_3d)

    # def convert_desktop_coordinates(self, obj):
    #     """转换笔筒坐标到相机坐标系"""
    #     if not self.camera_info or self.depth_image is None or not self.last_pen_holder_msg:
    #         rospy.logwarn("相机信息或深度图像未准备好")
    #         return
        
    #     # 获取相机内参
    #     K = np.array(self.camera_info.K).reshape(3, 3)
    #     fx, fy = K[0, 0], K[1, 1]
    #     cx, cy = K[0, 2], K[1, 2]
        
    #     points_3d = []
    #     pt = self.last_pen_holder_msg.point
    #     u, v = int(pt.x), int(pt.y)
        
    #     if 0 <= u < self.depth_image.shape[1] and 0 <= v < self.depth_image.shape[0]:
    #         depth = self.depth_image[v, u] / 1000.0  # 转换为米
    #         if self.validate_depth(depth, u, v):
    #             x = (u - cx) * depth / fx
    #             y = (v - cy) * depth / fy
    #             z = depth
    #             points_3d.append(Point(x, y, z))
        
    #     self.object_3d_dict["pen holder"].append(points_3d)

    def print_objects_info(self, event=None):
        """打印所有物体信息(包含3D坐标)"""
        with self.lock:
            rospy.loginfo("\n===== 当前检测到的物体 =====")
            for class_name, objects in self.object_dict.items():
                if objects:
                    rospy.loginfo(f"{class_name} (数量: {len(objects)}):")
                    for i, obj in enumerate(objects, 1):
                        # 获取对应的3D坐标
                        obj_3d = self.object_3d_dict[class_name][i-1] if i-1 < len(self.object_3d_dict[class_name]) else []
                        
                        if obj.is_polygon:
                            corners = [(p.x, p.y) for p in obj.polygon_corners]
                            rospy.loginfo(f"  物体{i}: 多边形, 角点: {corners}")
                            if obj_3d:
                                rospy.loginfo(f"    3D角点坐标 (camera_color_frame):")
                                for j, pt in enumerate(obj_3d):
                                    rospy.loginfo(f"      角点{j+1}: ({pt.x:.3f}, {pt.y:.3f}, {pt.z:.3f}) m")
                        else:
                            center = (obj.rect_center.x, obj.rect_center.y)
                            rospy.loginfo(f"  物体{i}: 矩形, 中心: ({center[0]:.1f}, {center[1]:.1f}), "
                                        f"尺寸: {obj.rect_width:.1f}x{obj.rect_height:.1f}, 角度: {obj.rect_angle:.1f}°")
                            if obj_3d:
                                pt = obj_3d[0]
                                rospy.loginfo(f"    3D中心坐标 (camera_color_frame): ({pt.x:.3f}, {pt.y:.3f}, {pt.z:.3f}) m")
            rospy.loginfo("=========================\n")
    
    # def pen_holder_callback(self, msg):
    #     """处理笔筒位置信息"""
    #     with self.lock:
    #         self.last_pen_holder_msg = msg
    #         rospy.logdebug("更新了笔筒位置信息")

    def desktop_callback(self, msg):
        """处理笔筒位置信息"""
        with self.lock:
            self.last_desktop_msg = msg
            rospy.logdebug("更新了desktop位置信息")
    
    def get_objects_by_class_3d(self, class_name):
        """获取指定类别的物体信息(包含3D坐标)"""
        with self.lock:
            if class_name in self.CLASS_NAMES:
                # 返回元组列表：(原始ObjectInfo, 3D点列表)
                return list(zip(self.object_dict[class_name], self.object_3d_dict[class_name]))
            return []
    
    def get_all_objects_3d(self):
        """获取所有物体的信息(包含3D坐标)"""
        with self.lock:
            all_objects = []
            for class_name in self.CLASS_NAMES:
                # 为每个类别添加(原始ObjectInfo, 3D点列表)元组
                all_objects.extend(zip(self.object_dict[class_name], self.object_3d_dict[class_name]))
            return all_objects

if __name__ == '__main__':
    rospy.init_node('object_vision_manager')  # 需要初始化节点来使用消息过滤器
    vision_manager = ObjectVisionManager()
    rospy.spin()
