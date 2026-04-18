#!/home/dongyi/anaconda3/envs/yolo_ros/bin/python

import cv2
import numpy as np
import rospy
import ros_numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from ultralytics import YOLO, SAM
import time
from object_keypoint_msgs.msg import ObjectInfo, ObjectInfoArray

# 定义类别和形状类（与原始程序相同）
CLASS_NAMES = ["eraser", "pen", "paper", "box", "book", "ruler", "lead_case", "triangle", "pen holder"]
SHAPE_CLASSES = ["ruler", "triangle", "book", "paper"]

def process_image(image):
    """处理图像并返回物体信息"""
    # 将ROS图像消息转换为OpenCV格式
    cv_image = ros_numpy.numpify(image)
    bgr_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
    
    # 运行目标检测
    results_det = model_det(bgr_image)
    boxes = results_det[0].boxes
    if len(boxes) == 0:
        return []
    
    # 提取边界框和类别
    bboxes = boxes.xyxy.tolist()
    cls_indices = boxes.cls.tolist()
    
    # 运行SAM分割
    results_seg = model_seg(bgr_image, bboxes=bboxes)
    if len(results_seg[0].masks) != len(bboxes):
        rospy.logwarn("检测框与分割结果数量不匹配")
        return []
    
    # 处理每个分割结果
    objects_info = []
    for i, mask in enumerate(results_seg[0].masks):
        cls_idx = int(cls_indices[i])
        class_name = CLASS_NAMES[cls_idx]
        contour = mask.xy[0].astype(np.int32)
        
        # 创建物体信息消息
        obj_info = ObjectInfo()
        obj_info.class_name = class_name
        
        if class_name in SHAPE_CLASSES:
            # 多边形近似处理
            epsilon = 0.02 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            
            # 根据类别调整顶点数
            if class_name in ["ruler", "book", "paper"]:
                desired_vertices = 4
            elif class_name == "triangle":
                desired_vertices = 3
            else:
                desired_vertices = None
            
            # 顶点优化
            if desired_vertices and len(approx) != desired_vertices:
                hull = cv2.convexHull(contour)
                epsilon = 0.02 * cv2.arcLength(hull, True)
                approx = cv2.approxPolyDP(hull, epsilon, True)
                
                max_iterations = 5
                iteration = 0
                while len(approx) > desired_vertices and iteration < max_iterations:
                    epsilon *= 1.5
                    approx = cv2.approxPolyDP(hull, epsilon, True)
                    iteration += 1
                
                if len(approx) > desired_vertices:
                    approx = approx[:desired_vertices]
            
            # 添加角点
            for pt in approx.squeeze():
                point = Point()
                point.x = float(pt[0])
                point.y = float(pt[1])
                obj_info.polygon_corners.append(point)
            obj_info.is_polygon = True
            
        else:
            # 计算位姿信息
            rect = cv2.minAreaRect(contour)
            center = rect[0]
            width, height = rect[1]
            angle = rect[2]
            
            obj_info.rect_center.x = float(center[0])
            obj_info.rect_center.y = float(center[1])
            obj_info.rect_width = float(width)
            obj_info.rect_height = float(height)
            obj_info.rect_angle = float(angle)
            obj_info.is_polygon = False
        
        objects_info.append(obj_info)
    
    return objects_info

def draw_detection_results(image, objects_info):
    img_draw = image.copy()
    colors = [(0, 255, 0), (255, 0, 0), (0, 0, 255), (255, 255, 0), 
              (0, 255, 255), (255, 0, 255), (128, 128, 0), (0, 128, 128), (128, 0, 128)]
    font = cv2.FONT_HERSHEY_SIMPLEX
    
    for obj_info in objects_info:
        # 跳过无效对象
        if obj_info.is_polygon and not obj_info.polygon_corners:
            continue
        if not obj_info.is_polygon and not hasattr(obj_info, 'rect_center'):
            continue
            
        color = colors[CLASS_NAMES.index(obj_info.class_name) % len(colors)]
        
        if obj_info.is_polygon and obj_info.polygon_corners:
            # 多边形绘制 - 确保正确提取坐标
            points = []
            for pt in obj_info.polygon_corners:
                # 直接提取 Point 对象的 x 和 y 属性
                points.append([pt.x, pt.y])
            
            pts = np.array(points, dtype=np.int32)
            pts = pts.reshape((-1, 1, 2))
            cv2.polylines(img_draw, [pts], True, color, 2)
            
            # 绘制角点
            for pt in points:
                cv2.circle(img_draw, (int(pt[0]), int(pt[1])), 5, (0, 0, 255), -1)
            
            # 计算标签位置
            center_x = int(np.mean([pt[0] for pt in points]))
            center_y = int(np.mean([pt[1] for pt in points]))
        else:
            # 矩形绘制
            try:
                # 确保 rect_center 是 Point 对象
                center = (obj_info.rect_center.x, obj_info.rect_center.y)
                size = (obj_info.rect_width, obj_info.rect_height)
                angle = obj_info.rect_angle
                
                box = cv2.boxPoints((center, size, angle))
                box = np.int0(box)
                cv2.drawContours(img_draw, [box], 0, color, 2)
                center_x = int(center[0])
                center_y = int(center[1])
            except AttributeError:
                continue
        
        # 绘制标签
        cv2.putText(img_draw, obj_info.class_name, (center_x, center_y), 
                   font, 0.5, color, 2, cv2.LINE_AA)
    
    return img_draw

def callback(data):
    """处理传入的图像并发布结果"""
    try:
        # 将ROS图像消息转换为OpenCV格式
        cv_image = ros_numpy.numpify(data)
        bgr_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)

        # 处理图像并获取物体信息
        objects_info = process_image(data)
        
        # 创建并发布物体信息数组
        info_array = ObjectInfoArray()
        info_array.header.stamp = rospy.Time.now()
        info_array.objects = objects_info
        object_info_pub.publish(info_array)

        # 打印检测结果到控制台
        print("\n" + "="*50)
        print(f"检测时间: {rospy.Time.now().to_sec():.3f}")
        print(f"检测到 {len(objects_info)} 个物体:")
        
        for i, obj in enumerate(objects_info):
            print(f"\n物体 {i+1}:")
            print(f"  类别: {obj.class_name}")
            
            if obj.is_polygon:
                print(f"  类型: 多边形 (角点数: {len(obj.polygon_corners)})")
                for j, corner in enumerate(obj.polygon_corners):
                    print(f"    角点 {j+1}: X={corner.x:.1f}, Y={corner.y:.1f}")
            else:
                print("  类型: 矩形")
                print(f"  中心点: X={obj.rect_center.x:.1f}, Y={obj.rect_center.y:.1f}")
                print(f"  尺寸: {obj.rect_width:.1f} x {obj.rect_height:.1f}")
                print(f"  旋转角度: {obj.rect_angle:.1f}°")
        
        print("="*50 + "\n")

        # 绘制结果并显示
        result_image = draw_detection_results(bgr_image, objects_info)
        # 显示结果
        cv2.imshow("Object Detection Results", result_image)
        cv2.waitKey(1)  # 必要的，用于刷新显示
        
        # rospy.loginfo(f"检测到 {len(objects_info)} 个物体")
        
    except Exception as e:
        rospy.logerr(f"处理错误: {str(e)}")

if __name__ == '__main__':
    rospy.init_node("object_detection_segmentation")
    
    # 加载模型
    model_det = YOLO("/home/dongyi/Ultralytics_test/models/bestV2.pt")
    model_seg = SAM("/home/dongyi/Ultralytics_test/models/sam2.1_b.pt")
    
    # 创建发布者
    object_info_pub = rospy.Publisher("/object_detection/info", ObjectInfoArray, queue_size=10)
    
    # 订阅相机话题
    rospy.Subscriber("/camera/color/image_raw", Image, callback, queue_size=1)
    
    rospy.loginfo("物体检测与分割节点已启动，等待图像数据...")

    try:
        rospy.spin()
    except KeyboardInterrupt:
        cv2.destroyAllWindows()
        rospy.loginfo("关闭节点...")