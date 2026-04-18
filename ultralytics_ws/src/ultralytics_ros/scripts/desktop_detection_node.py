#! /home/dongyi/anaconda3/envs/yolo_ros/bin/python

import rospy
import numpy as np
import cv2
import open3d as o3d
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridge
from scipy.spatial import ConvexHull
from geometry_msgs.msg import Point
from object_keypoint_msgs.msg import ObjectInfo, ObjectInfoArray
from std_msgs.msg import Header

class DesktopDetector:
    def __init__(self):
        rospy.init_node('desktop_detector', anonymous=True)
        
        # 参数设置
        self.voxel_size = rospy.get_param('~voxel_size', 0.005)
        self.distance_threshold = rospy.get_param('~distance_threshold', 0.01)
        self.min_depth = rospy.get_param('~min_depth', 0.4)
        self.max_depth = rospy.get_param('~max_depth', 1.2)
        
        # 订阅Realsense话题
        rospy.Subscriber("/camera/depth/color/points", PointCloud2, self.pointcloud_callback) # pointcloud is aligned automatively
        rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        rospy.Subscriber("/camera/aligned_depth_to_color/camera_info", CameraInfo, self.camera_info_callback)
        
        # 发布自定义消息
        self.detection_pub = rospy.Publisher("/desktop/detection", ObjectInfoArray, queue_size=10)
        
        # 初始化变量
        self.bridge = CvBridge()
        self.latest_pointcloud = None
        self.latest_image = None
        self.camera_intrinsics = None
        
        # 获取相机内参
        # self.get_camera_intrinsics()
        
        rospy.loginfo("Desktop detector initialized")

    def camera_info_callback(self, camera_info):
        """从camera_info话题获取相机内参"""
        self.camera_intrinsics = {
            'fx': camera_info.K[0],
            'fy': camera_info.K[4],
            'cx': camera_info.K[2],
            'cy': camera_info.K[5]
            }
        rospy.loginfo("Loaded camera intrinsics from topic")

    def pointcloud_callback(self, msg):
        """点云回调函数"""
        self.latest_pointcloud = msg

    def image_callback(self, msg):
        """图像回调函数"""
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"Image conversion error: {str(e)}")

    def process_data(self):
        """处理点云和图像数据"""
        if self.latest_pointcloud is None or self.latest_image is None:
            return
            
        # 转换点云为Open3D格式
        points = pc2.read_points(self.latest_pointcloud, field_names=("x", "y", "z"), skip_nans=True)
        points_list = list(points)
        points_array = np.array(points_list)
        
        if len(points_array) == 0:
            rospy.logwarn("Empty point cloud received")
            return
            
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points_array)
        
        # 检测桌面平面
        desktop_cloud, plane_model = self.detect_desktop_plane(pcd)
        
        if desktop_cloud is None:
            rospy.logwarn("No desktop plane detected")
            return
            
        # 提取桌面多边形
        desktop_polygon = self.extract_desktop_polygon(desktop_cloud)
        
        if desktop_polygon is None or len(desktop_polygon) < 3:
            rospy.logwarn("Invalid desktop polygon detected")
            return
            
        # 拟合最小外接矩形
        rect_info = self.fit_min_area_rect(desktop_polygon)
        
        # 准备发布消息
        detection_msg = self.prepare_detection_msg(rect_info, self.latest_pointcloud.header)
        
        # 发布消息
        self.detection_pub.publish(detection_msg)
        
        # 可视化结果
        self.visualize_results(desktop_polygon, rect_info) # rect_info exceeds the bounding of image???
        
    def detect_desktop_plane(self, pcd):
        """检测桌面平面"""
        # 深度过滤
        pcd_points = np.asarray(pcd.points)
        z_coords = pcd_points[:, 2]
        depth_mask = (z_coords >= self.min_depth) & (z_coords <= self.max_depth)
        filtered_pcd = pcd.select_by_index(np.where(depth_mask)[0])
        
        if len(filtered_pcd.points) == 0:
            return None, None
        
        # 下采样
        downsampled_pcd = filtered_pcd.voxel_down_sample(voxel_size=self.voxel_size)
        
        # 离群点去除
        cl, ind = downsampled_pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
        filtered_down_pcd = downsampled_pcd.select_by_index(ind)
        
        # 平面分割
        try:
            plane_model, inliers = filtered_down_pcd.segment_plane(
                distance_threshold=self.distance_threshold,
                ransac_n=3,
                num_iterations=1000
            )
            desktop_cloud = filtered_down_pcd.select_by_index(inliers)
            return desktop_cloud, plane_model
        except Exception as e:
            rospy.logerr(f"Plane segmentation failed: {str(e)}")
            return None, None

    def extract_desktop_polygon(self, desktop_cloud):
        """提取桌面多边形"""
        desktop_points = np.asarray(desktop_cloud.points)
        
        # 检查点云是否有效
        if len(desktop_points) < 3:
            return None
            
        # 计算XY平面的凸包
        xy_points = desktop_points[:, :2]
        try:
            hull = ConvexHull(xy_points)
            boundary_points = desktop_points[hull.vertices]
        except:
            rospy.logwarn("Convex hull computation failed")
            return None
            
        # 投影到图像平面
        image_points = []
        for point in boundary_points:
            u = int((point[0] * self.camera_intrinsics['fx'] / point[2]) + self.camera_intrinsics['cx'])
            v = int((point[1] * self.camera_intrinsics['fy'] / point[2]) + self.camera_intrinsics['cy'])
            image_points.append([u, v])
            
        return np.array(image_points, dtype=np.int32)

    def fit_min_area_rect(self, desktop_polygon):
        """拟合最小外接矩形"""
        # 拟合旋转矩形
        rect = cv2.minAreaRect(desktop_polygon)
        (center_x, center_y), (width, height), angle = rect
        
        # 获取矩形四个角点
        box = cv2.boxPoints(rect)
        box = np.int0(box)

        # 边界检查：确保坐标在图像范围内
        for point in box:
            # 确保x坐标在[0, 1279]范围内
            if point[0] < 0:
                point[0] = 0
            elif point[0] > 1279:
                point[0] = 1279
            
            # 确保y坐标在[0, 719]范围内
            if point[1] < 0:
                point[1] = 0
            elif point[1] > 719:
                point[1] = 719
        
        return {
            'center': (center_x, center_y),
            'size': (width, height),
            'angle': angle,
            'corners': box
        }

    def prepare_detection_msg(self, rect_info, header):
        """准备检测结果消息"""
        msg = ObjectInfoArray()
        msg.header = Header(stamp=rospy.Time.now(), frame_id=header.frame_id)
        
        obj_info = ObjectInfo()
        obj_info.class_name = "desktop"
        obj_info.is_polygon = True
        
        # 添加矩形角点
        for corner in rect_info['corners']:
            point = Point()
            point.x = corner[0]
            point.y = corner[1]
            point.z = 0
            obj_info.polygon_corners.append(point)
        
        # 添加矩形信息
        center = Point()
        center.x = rect_info['center'][0]
        center.y = rect_info['center'][1]
        center.z = 0
        obj_info.rect_center = center
        
        obj_info.rect_width = rect_info['size'][0]
        obj_info.rect_height = rect_info['size'][1]
        obj_info.rect_angle = rect_info['angle']
        
        msg.objects.append(obj_info)
        return msg

    def visualize_results(self, desktop_polygon, rect_info):
        """可视化结果"""
        if self.latest_image is None:
            return
            
        # 创建副本以保留原始图像
        vis_image = self.latest_image.copy()
        
        # 绘制桌面多边形
        if desktop_polygon is not None and len(desktop_polygon) > 2:
            cv2.polylines(vis_image, [desktop_polygon], True, (0, 255, 0), 2)
            
        # 绘制最小外接矩形
        if rect_info is not None:
            box = rect_info['corners']
            cv2.drawContours(vis_image, [box], 0, (0, 0, 255), 2)
            
            # 绘制中心点
            center = tuple(map(int, rect_info['center']))
            cv2.circle(vis_image, center, 5, (255, 0, 0), -1)
            
            # 添加文本信息
            text = f"Desktop {rect_info['size'][0]:.1f}x{rect_info['size'][1]:.1f}"
            cv2.putText(vis_image, text, (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # 显示结果
        cv2.imshow("Desktop Detection", vis_image)
        cv2.waitKey(1)

    def run(self):
        """主循环"""
        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown():
            try:
                self.process_data()
            except Exception as e:
                rospy.logerr(f"Processing error: {str(e)}")
            rate.sleep()

if __name__ == '__main__':
    detector = DesktopDetector()
    try:
        detector.run()
    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()
