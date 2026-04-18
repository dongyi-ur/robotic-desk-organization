#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from object_primitives import EraserPrimitive, PenPrimitive, RulerPrimitive, PaperPrimitive, BookPrimitive
# import motion_primitives  # 假设这是您的运动控制模块

class TaskPlanner:
    def __init__(self):
        """初始化任务规划器"""
        rospy.init_node('task_planner', anonymous=True)
        
        # 初始化运动控制和夹爪控制
        # self.motion_prim = motion_primitives
        # self.motion_prim.init_gripper_publisher() # rospy.Publisher('/gripper_control', String, queue_size=10) 
        # self.gripper_pub = rospy.Publisher('/gripper_control', String, queue_size=10)
        
        # # 创建橡皮擦操作原语
        self.eraser_primitive = EraserPrimitive()
        self.pen_primitive = PenPrimitive()
        self.ruler_primitive = RulerPrimitive()
        self.paper_primitive = PaperPrimitive()
        self.book_primitive = BookPrimitive()
        
        # 设置话题订阅者
        self.command_sub = rospy.Subscriber('/eraser_task_command', String, self.book_command_cb)
        self.result_pub = rospy.Publisher('/eraser_task_result', String, queue_size=10)
        
        rospy.loginfo("TaskPlanner 初始化完成，等待命令...")

    def eraser_command_cb(self, msg):
        """橡皮擦任务命令的话题回调"""
        try:
            command = msg.data
            rospy.loginfo(f"收到橡皮擦任务命令: {command}")
            
            if command == "start":
                success = self.eraser_primitive.execute()
                result_msg = "success" if success else "failure"
                self.result_pub.publish(result_msg) 
                
                if success:
                    rospy.loginfo("橡皮擦任务执行成功!")
                else:
                    rospy.logwarn("橡皮擦任务执行失败!")
                    
        except Exception as e:
            rospy.logerr(f"执行橡皮擦任务时出错: {str(e)}")
            self.result_pub.publish(f"error: {str(e)}")

    def pen_command_cb(self, msg):
        """Pen任务命令的话题回调"""
        try:
            command = msg.data
            rospy.loginfo(f"收到Pen任务命令: {command}")
            
            if command == "start":
                success = self.pen_primitive.execute()
                result_msg = "success" if success else "failure"
                self.result_pub.publish(result_msg) 
                
                if success:
                    rospy.loginfo("Pen任务执行成功!")
                else:
                    rospy.logwarn("Pen任务执行失败!")
                    
        except Exception as e:
            rospy.logerr(f"执行Pen任务时出错: {str(e)}")
            self.result_pub.publish(f"error: {str(e)}")

    def ruler_command_cb(self, msg):
        """ruler任务命令的话题回调"""
        try:
            command = msg.data
            rospy.loginfo(f"收到Ruler任务命令: {command}")
            
            if command == "start":
                success = self.ruler_primitive.execute()
                result_msg = "success" if success else "failure"
                self.result_pub.publish(result_msg) 
                
                if success:
                    rospy.loginfo("Ruler任务执行成功!")
                else:
                    rospy.logwarn("Ruler任务执行失败!")
                    
        except Exception as e:
            rospy.logerr(f"执行Ruler任务时出错: {str(e)}")
            self.result_pub.publish(f"error: {str(e)}")

    def paper_command_cb(self, msg):
        """paper任务命令的话题回调"""
        try:
            command = msg.data
            rospy.loginfo(f"收到Paper任务命令: {command}")
            
            if command == "start":
                success = self.paper_primitive.execute()
                result_msg = "success" if success else "failure"
                self.result_pub.publish(result_msg) 
                
                if success:
                    rospy.loginfo("Paper任务执行成功!")
                else:
                    rospy.logwarn("Paper任务执行失败!")
                    
        except Exception as e:
            rospy.logerr(f"执行Paper任务时出错: {str(e)}")
            self.result_pub.publish(f"error: {str(e)}")

    def book_command_cb(self, msg):
        """book任务命令的话题回调"""
        try:
            command = msg.data
            rospy.loginfo(f"收到Book任务命令: {command}")
            
            if command == "start":
                success = self.book_primitive.execute()
                result_msg = "success" if success else "failure"
                self.result_pub.publish(result_msg) 
                
                if success:
                    rospy.loginfo("Book任务执行成功!")
                else:
                    rospy.logwarn("Book任务执行失败!")
                    
        except Exception as e:
            rospy.logerr(f"执行Book任务时出错: {str(e)}")
            self.result_pub.publish(f"error: {str(e)}")

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
# rostopic pub /eraser_task_command std_msgs/String "start"  # 开始任务
# rostopic pub /eraser_task/command std_msgs/String "stop"   # 停止任务

# 监控任务状态:
# rostopic echo /eraser_task/status  # 查看状态信息
# rostopic echo /eraser_task/result  # 查看最终结果