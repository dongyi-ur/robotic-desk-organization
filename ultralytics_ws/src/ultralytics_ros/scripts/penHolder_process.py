#!/home/dongyi/anaconda3/envs/yolo_ros/bin/python

import rospy
import ros_numpy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, PointStamped
from ultralytics import YOLO, SAM
from std_msgs.msg import Header

class PenHolderDetector:
    def __init__(self):
        rospy.init_node('pen_holder_detector', anonymous=True)
        
        # 加载模型
        self.model_det = YOLO("/home/dongyi/Ultralytics_test/models/best.pt")
        self.model_seg = SAM("/home/dongyi/Ultralytics_test/models/sam2.1_b.pt")
        
        # 定义类别
        self.CLASS_NAMES = ["eraser", "pen", "paper", "box", "book", 
                            "ruler", "lead_case", "triangle", "pen holder"]
        self.PEN_HOLDER_ID = self.CLASS_NAMES.index("pen holder")
        
        # 创建发布者 (只保留位置发布)
        self.position_pub = rospy.Publisher("/pen_holder/position", PointStamped, queue_size=10)
        
        # 订阅相机话题
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        
        # 创建OpenCV窗口
        cv2.namedWindow("Pen Holder Detection", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Pen Holder Detection", 800, 600)
        
        rospy.loginfo("笔筒检测节点已启动，等待图像数据...")
    
    def find_pen_holder_center(self, color_img, mask, bbox):
        """寻找笔筒端面中心点并返回椭圆参数"""
        try:
            # 1. 提取ROI区域
            x1, y1, x2, y2 = map(int, bbox)
            roi = color_img[y1:y2, x1:x2]
            
            # 2. 应用SAM掩码
            mask_data = mask.data.cpu().numpy().squeeze()
            mask_uint8 = (mask_data * 255).astype(np.uint8)
            roi_mask = mask_uint8[y1:y2, x1:x2]
            
            # 3. 转换为灰度图并提取上半部分
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            top_region = gray[:gray.shape[0]//2, :]
            
            # 4. 边缘检测
            edges = cv2.Canny(top_region, 50, 180)
            edges = cv2.dilate(edges, np.ones((3, 3), np.uint8), iterations=1)
            
            # 5. 寻找轮廓
            contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not contours:
                return None, None
            
            # 6. 选择最大轮廓
            largest_contour = max(contours, key=cv2.contourArea)
            
            # 7. 椭圆拟合
            if len(largest_contour) >= 5:
                ellipse = cv2.fitEllipse(largest_contour)
                (cx, cy), (ma, MA), angle = ellipse
                
                # 转换到全局坐标
                cx_global = cx + x1
                cy_global = cy + y1
                
                # 返回全局中心点和椭圆参数（局部坐标）
                return (cx_global, cy_global), ellipse
            return None, None
            
        except Exception as e:
            rospy.logwarn(f"笔筒中心检测错误: {str(e)}")
            return None, None
    
    def image_callback(self, msg):
        """处理传入的图像"""
        try:
            # 转换ROS图像消息为OpenCV格式
            cv_image = ros_numpy.numpify(msg)
            bgr_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
            
            # 运行目标检测
            results_det = self.model_det(bgr_image)
            boxes = results_det[0].boxes
            if len(boxes) == 0:
                # 显示原始图像
                cv2.imshow("Pen Holder Detection", bgr_image)
                cv2.waitKey(1)
                return
            
            # 查找笔筒检测框
            pen_holder_detected = False
            debug_image = bgr_image.copy()
            position_msg = None
            
            # 查找所有笔筒检测
            pen_holder_boxes = []
            for i in range(len(boxes)):
                cls_idx = int(boxes.cls[i])
                if cls_idx == self.PEN_HOLDER_ID:
                    confidence = boxes.conf[i].item()
                    bbox = boxes.xyxy[i].cpu().numpy()
                    pen_holder_boxes.append((confidence, bbox))
            
            # 如果有检测到笔筒
            if pen_holder_boxes:
                # 选择置信度最高的笔筒
                best_box = max(pen_holder_boxes, key=lambda x: x[0])[1]
                
                # 运行SAM分割
                results_seg = self.model_seg(bgr_image, bboxes=[best_box])
                if results_seg and len(results_seg[0].masks) > 0:
                    mask = results_seg[0].masks[0]
                    
                    # 寻找笔筒中心
                    center, ellipse_params = self.find_pen_holder_center(bgr_image, mask, best_box)
                    
                    if center and ellipse_params:
                        cx, cy = center
                        
                        # 创建位置消息
                        position_msg = PointStamped()
                        position_msg.header = Header(stamp=rospy.Time.now(), frame_id="camera_color_optical_frame")
                        position_msg.point = Point(x=cx, y=cy, z=0)
                        
                        # 在调试图像上标记
                        # 1. 绘制检测框
                        cv2.rectangle(debug_image, 
                                    (int(best_box[0]), int(best_box[1])),
                                    (int(best_box[2]), int(best_box[3])),
                                    (0, 255, 0), 2)
                        
                        # 2. 绘制椭圆轮廓
                        # 提取ROI坐标
                        x1, y1, x2, y2 = map(int, best_box)
                        # 调整椭圆参数到全局坐标系
                        (cx_local, cy_local), (ma, MA), angle = ellipse_params
                        # 创建全局坐标的椭圆参数
                        ellipse_global = ((cx, cy), (ma, MA), angle)
                        # 绘制椭圆
                        cv2.ellipse(debug_image, ellipse_global, (0, 255, 255), 2)
                        
                        # 3. 绘制中心点
                        cv2.circle(debug_image, (int(cx), int(cy)), 10, (0, 0, 255), -1)
                        
                        # 4. 添加文本信息
                        cv2.putText(debug_image, f"Pen Holder ({cx:.1f}, {cy:.1f})", 
                                (int(best_box[0]), int(best_box[1]) - 10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                        
                        pen_holder_detected = True
            
            # 发布位置
            if position_msg:
                self.position_pub.publish(position_msg)
            
            # 直接显示调试图像
            cv2.imshow("Pen Holder Detection", debug_image)
            cv2.waitKey(1)

        except Exception as e:
            rospy.logerr(f"图像处理错误: {str(e)}")

    def __del__(self):
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        detector = PenHolderDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass