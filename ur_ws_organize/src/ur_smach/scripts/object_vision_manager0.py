#!/usr/bin/python

import rospy
import threading  # 添加这行导入
from object_keypoint_msgs.msg import ObjectInfo, ObjectInfoArray
from geometry_msgs.msg import PointStamped

class ObjectVisionManager:
    def __init__(self):
        rospy.init_node('object_vision_manager') # if use TaskPlanner, do not use this line
        
        # 定义所有可能的物体类别
        self.CLASS_NAMES = ["eraser", "pen", "paper", "box", "book", 
                           "ruler", "lead_case", "triangle", "pen holder"]
        
        # 初始化物体存储字典
        self.object_dict = {cls: [] for cls in self.CLASS_NAMES}
        
        # 线程安全的锁，防止同时访问字典
        self.lock = threading.RLock()

        # ...其他初始化代码不变...
        self.last_pen_holder_msg = None  # 新增：保存最后一次笔筒位置消息
        
        # 订阅两个节点的话题
        rospy.Subscriber("/object_detection/info", ObjectInfoArray, self.object_info_callback)
        rospy.Subscriber("/pen_holder/position", PointStamped, self.pen_holder_callback)

        # 添加定时器，每2秒输出一次物体信息
        # rospy.Timer(rospy.Duration(2), self.print_objects_info)
        
        rospy.loginfo("物体视觉管理器已启动，等待数据...")

    def print_objects_info(self, event=None):
        """打印所有物体信息"""
        with self.lock:
            rospy.loginfo("\n===== 当前检测到的物体 =====")
            for class_name, objects in self.object_dict.items():
                if objects:  # 只输出有物体的类别
                    rospy.loginfo(f"{class_name} (数量: {len(objects)}):")
                    for i, obj in enumerate(objects, 1):
                        if obj.is_polygon:
                            corners = [(p.x, p.y) for p in obj.polygon_corners]
                            rospy.loginfo(f"  物体{i}: 多边形, 角点: {corners}")
                        else:
                            rospy.loginfo(f"  物体{i}: 矩形, 中心: ({obj.rect_center.x:.1f}, {obj.rect_center.y:.1f}), "
                                        f"尺寸: {obj.rect_width:.1f}x{obj.rect_height:.1f}, 角度: {obj.rect_angle:.1f}°")
            rospy.loginfo("=========================\n")
    
    def object_info_callback(self, msg):
        """处理来自节点1的物体信息"""
        with self.lock:
            # 清空所有物体信息(包括笔筒)
            self.object_dict = {cls: [] for cls in self.CLASS_NAMES}
            
            # 添加新的物体信息(不包括笔筒)
            for obj in msg.objects:
                if obj.class_name in self.CLASS_NAMES and obj.class_name != "pen holder":
                    self.object_dict[obj.class_name].append(obj)
            
            # 如果有保存的笔筒位置，添加到物体信息
            if self.last_pen_holder_msg:
                pen_holder = ObjectInfo()
                pen_holder.class_name = "pen holder"
                pen_holder.is_polygon = False
                pen_holder.rect_center = self.last_pen_holder_msg.point
                # 设置默认尺寸(根据实际情况调整)
                pen_holder.rect_width = 0.0  
                pen_holder.rect_height = 0.0
                pen_holder.rect_angle = 0.0
                self.object_dict["pen holder"] = [pen_holder]
            
            rospy.logdebug(f"更新了{len(msg.objects)}个物体的视觉信息")
            self.print_objects_info()
    
    def pen_holder_callback(self, msg):
        """处理来自节点2的笔筒位置信息"""
        with self.lock:
            # 只保存位置信息，不直接修改object_dict
            self.last_pen_holder_msg = msg
            rospy.logdebug("更新了笔筒位置信息")
    
    def get_objects_by_class(self, class_name):
        """获取指定类别的物体信息"""
        with self.lock:
            if class_name in self.object_dict:
                return self.object_dict[class_name]
            return []
    
    def get_all_objects(self):
        """获取所有物体的信息"""
        with self.lock:
            all_objects = []
            for objects in self.object_dict.values():
                all_objects.extend(objects)
            return all_objects

if __name__ == '__main__':
    # 初始化视觉管理器
    vision_manager = ObjectVisionManager()
    
    # 示例：等待规划器使用这些信息
    rospy.spin()