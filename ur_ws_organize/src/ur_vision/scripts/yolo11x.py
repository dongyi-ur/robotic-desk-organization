#!/usr/bin/env python

# import sys
# sys.path.append('/home/dongyi/anaconda3/envs/YOLOnew/lib/python3.12/site-packages')
# sys.path.remove('/usr/lib/python3/dist-packages')
# sys.path.remove('/home/dongyi/realsense_ros_ws/devel/lib/python3/dist-packages')

import time
import ros_numpy
import rospy
from sensor_msgs.msg import Image
from ultralytics import YOLO

def main():
    rospy.init_node('yolo', anonymous=True)
    print("Node initialized.")

    # 加载模型
    print("Loading detection model...")
    detection_model = YOLO("yolo11n.pt")  # 替换为你的模型路径
    print("Detection model loaded.")
    print("Loading segmentation model...")
    segmentation_model = YOLO("yolo11n-seg.pt")
    print("Segmentation model loaded.")

    # 定义发布器
    det_image_pub = rospy.Publisher('/ultralytics/detection/image', Image, queue_size=5)
    seg_image_pub = rospy.Publisher('/ultralytics/segmentation/image', Image, queue_size=5)

    def callback(data):
        try:
            array = ros_numpy.numpify(data)
            # 执行检测
            det_results = detection_model(array)
            det_annotated = det_results[0].plot()
            det_image_pub.publish(ros_numpy.msgify(Image, det_annotated, encoding="rgb8"))
            # 执行分割
            seg_results = segmentation_model(array)
            seg_annotated = seg_results[0].plot()
            seg_image_pub.publish(ros_numpy.msgify(Image, seg_annotated, encoding="rgb8"))
        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

    # 订阅图像话题
    rospy.Subscriber('/camera/color/image_raw', Image, callback)
    print("Ready to process images.")

    try:
        rospy.spin()  # 保持节点运行，直到收到退出信号
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down YOLO node...")

if __name__ == '__main__':
    main()
