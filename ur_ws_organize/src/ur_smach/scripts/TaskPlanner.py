#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Point
from object_primitives import ReorientDeformablePrimitive,EraserPrimitive, PenPrimitive, RulerPrimitive, PaperPrimitive, BookPrimitive
from object_vision_manager1 import ObjectVisionManager
import geometry_utils
# import motion_primitives  # 假设这是您的运动控制模块

class TaskPlanner:
    def __init__(self):
        """初始化任务规划器"""
        rospy.init_node('task_planner', anonymous=True)
        
        # # 创建橡皮擦操作原语
        self.eraser_primitive = EraserPrimitive()
        self.pen_primitive = PenPrimitive()
        self.ruler_primitive = RulerPrimitive()
        self.paper_primitive = PaperPrimitive()
        self.book_primitive = BookPrimitive()
        self.reorient_deformable = ReorientDeformablePrimitive()
        
        # 视觉管理器
        self.vision = ObjectVisionManager()
        
        # 设置话题订阅者（新增总任务入口）
        self.command_sub = rospy.Subscriber('/tidy_task_command', String, self.tidy_command_cb)
        self.result_pub = rospy.Publisher('/tidy_task_result', String, queue_size=10)
        
        rospy.loginfo("TaskPlanner 初始化完成，等待命令...")

        # 定义物体类别集合
        self.small_objects = {"eraser", "lead_case", "pen"}
        self.rigid_2d_objects = {"ruler", "triangle"}
        self.deformable_2d_objects = {"paper", "book"}

    def tidy_command_cb(self, msg):
        """总整理任务的命令回调: data=='start' 触发一次完整整理流程"""
        try:
            command = msg.data
            rospy.loginfo(f"收到整理任务命令: {command}")
            if command == "start":
                success = self.plan_and_execute()
                self.result_pub.publish("success" if success else "failure")
        except Exception as e:
            rospy.logerr(f"整理任务执行出错: {str(e)}")
            self.result_pub.publish(f"error: {str(e)}")

    def _group_objects(self):
        """从视觉管理器一次性读取所有视觉信息并按类别分组。排除多余物体如box
        返回 dict: class_name -> list[(ObjectInfo, points3d)]"""
        grouped = {cls: [] for cls in self.small_objects.union(self.rigid_2d_objects).union(self.deformable_2d_objects).union({"desktop"})}
        all_objs = self.vision.get_all_objects_3d()
        for obj_info, pts3d in all_objs:
            cls = getattr(obj_info, 'class_name', None)
            if cls in grouped:
                grouped[cls].append((obj_info, pts3d))
        return grouped

    def _any_rigid_on_book(self, grouped):
        """判断是否存在任一 2D 刚体物体（ruler/triangle）在任一本书上。
        使用 geometry_utils.is_ruler_on_book(book_corners, rigid_center) 判断。
        """
        books = grouped.get("book", [])
        if not books:
            return False

        # 收集每本书的四边形角点列表
        book_corners_list = []
        for book_obj, book_pts in books:
            if book_obj.is_polygon and len(book_pts) >= 4:
                book_corners_list.append(book_pts)

        if not book_corners_list:
            return False

        # 遍历所有刚体（ruler/triangle），计算其中心作为 ruler_position
        rigid_instances = []
        for cls in self.rigid_2d_objects:
            rigid_instances.extend(grouped.get(cls, []))

        for rigid_obj, rigid_pts in rigid_instances:
            # 若是多边形，依据角点算最小外接矩形中心；否则使用第一个3D点作为中心
            if rigid_obj.is_polygon and len(rigid_pts) >= 3:
                center_xyz, _, _, _ = geometry_utils.calculate_min_area_rect(rigid_pts)
                rigid_center = Point(x=center_xyz[0], y=center_xyz[1], z=center_xyz[2])
            elif len(rigid_pts) >= 1:
                rigid_center = rigid_pts[0]
            else:
                continue

            # 任一本书包含该中心即视为在书上
            for book_corners in book_corners_list:
                if geometry_utils.is_ruler_on_book(book_corners, rigid_center):
                    return True

        return False

    def plan_and_execute(self):
        """按照用户指定逻辑执行一次完整的桌面整理流程。"""
        try:
            grouped = self._group_objects()

            # 1) 先整理 small_objects：eraser/lead_case -> EraserPrimitive；pen -> PenPrimitive
            # 对每个实例执行操作（邻近物体管理已在原语内部处理）
            for cls in ["eraser", "lead_case"]:
                for obj_info, pts3d in grouped.get(cls, []):
                    rospy.loginfo(f"执行 {cls} -> EraserPrimitive")
                    self.eraser_primitive.execute()

            for obj_info, pts3d in grouped.get("pen", []):
                rospy.loginfo("执行 pen -> PenPrimitive")
                self.pen_primitive.execute()

            # 2) 判断 2D 刚体是否在书上
            rigid_on_book = self._any_rigid_on_book(grouped)
            rospy.loginfo(f"2D 刚体是否在书上: {rigid_on_book}")

            def handle_rigid():
                # ruler/triangle 统一执行 RulerPrimitive（邻近物体管理已在原语内部处理）
                for cls in ["ruler", "triangle"]:
                    for obj_info, pts3d in grouped.get(cls, []):
                        rospy.loginfo(f"执行 {cls} -> RulerPrimitive")
                        self.ruler_primitive.execute()

            def handle_deformable():
                """处理可变形物体（paper和book）的逻辑
                一般最多出现2个deformable objects
                """
                papers = grouped.get("paper", [])
                books = grouped.get("book", [])
                
                # 统计可变形物体总数
                total_deformable = len(papers) + len(books)
                rospy.loginfo(f"检测到 {len(papers)} 张纸, {len(books)} 本书, 总计 {total_deformable} 个可变形物体")
                
                if total_deformable > 2:
                    rospy.logwarn(f"可变形物体数量超过预期: {total_deformable} > 2")
                
                # 情况1: 单纸或单书 - 调用reorient_deformable原语
                if total_deformable == 1:
                    if papers:
                        obj_info, pts3d = papers[0]
                        rospy.loginfo("执行 单纸 -> reorient_deformable")
                        # TODO: 调用reorient_deformable原语，传入obj_info和pts3d
                        self.reorient_deformable.execute(papers[0])
                    elif books:
                        obj_info, pts3d = books[0]
                        rospy.loginfo("执行 单书 -> reorient_deformable")
                        # TODO: 调用reorient_deformable原语，传入obj_info和pts3d
                        self.reorient_deformable.execute(books[0])
                
                # 情况2: 纸+书 或 两张纸 - 调用PaperPrimitive原语
                elif total_deformable == 2:
                    if len(papers) == 2:
                        rospy.loginfo("执行 两张纸 -> PaperPrimitive")
                        self.paper_primitive.execute()
                    elif len(papers) == 1 and len(books) == 1:
                        rospy.loginfo("执行 纸+书 -> PaperPrimitive")
                        self.paper_primitive.execute()
                    elif len(books) == 2:
                        rospy.loginfo("执行 两本书 -> BookPrimitive")
                        self.book_primitive.execute()
                
                # 情况3: 超过2个可变形物体时的处理（异常情况）
                elif total_deformable > 2:
                    rospy.logwarn("可变形物体数量异常，尝试按优先级处理")
                    # 优先处理paper
                    if papers:
                        rospy.loginfo("执行 多张纸 -> PaperPrimitive")
                        self.paper_primitive.execute()
                    # 然后处理book
                    if books:
                        rospy.loginfo("执行 多本书 -> BookPrimitive")
                        self.book_primitive.execute()

            # 3) 顺序：在书上 -> 先刚体后可变形；不在书上 -> 先可变形后刚体
            if rigid_on_book:
                handle_rigid()
                handle_deformable()
            else:
                handle_deformable()
                handle_rigid()

            rospy.loginfo("整理流程完成")
            return True
        except Exception as e:
            rospy.logerr(f"整理流程失败: {str(e)}")
            return False

    def run(self):
        """运行任务规划器"""
        rospy.spin()

if __name__ == '__main__':
    try:
        planner = TaskPlanner()
        planner.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"TaskPlanner 发生错误: {str(e)}")

# 启动节点:
# rosrun your_package task_planner.py

# 发送任务指令:
# rostopic pub /tidy_task_command std_msgs/String "start"  # 开始任务
# rostopic pub /tidy_task_command std_msgs/String "stop"   # 停止任务

# 监控任务状态:
# rostopic echo /tidy_task/status  # 查看状态信息
# rostopic echo /tidy_task_result  # 查看最终结果