#! /home/dongyi/anaconda3/envs/yolo_ros/bin/python

# ROS node for real-time object detection and segmentation using Ultralytics YOLO and SAM models
import time
import ros_numpy  # For converting between ROS and numpy formats
import rospy  # ROS Python client library
import cv2  # 添加OpenCV用于图像转换
from sensor_msgs.msg import Image  # ROS standard image message type
from ultralytics import YOLO, SAM  # Ultralytics detection and segmentation models

# Initialize YOLO and SAM models with pretrained weights
detection_model: YOLO = YOLO("/home/dongyi/Ultralytics_test/models/bestV2.pt")  # Object detection model
segmentation_model: SAM = SAM("/home/dongyi/Ultralytics_test/models/sam2.1_b.pt")  # Segmentation model

rospy.init_node("ultralytics")
time.sleep(1)

# Create ROS publishers for annotated results
det_image_pub: rospy.Publisher = rospy.Publisher(
    "/ultralytics/detection/image",  # Topic for detection results
    Image,  # Message type
    queue_size=5  # Queue size for published messages
)
seg_image_pub: rospy.Publisher = rospy.Publisher(
    "/ultralytics/segmentation/image",  # Topic for segmentation results
    Image,  # Message type
    queue_size=5  # Queue size for published messages
)

def callback(data: Image) -> None:
    """Process incoming image, run detection/segmentation, and publish results.
    
    Args:
        data (sensor_msgs.msg.Image): Input image message from ROS subscriber
        
    Note:
        Only processes images when there are active subscribers to save computation.
        Publishes annotated images with detection/segmentation results.
    """
    try:
        array = ros_numpy.numpify(data)
        rgb_img = cv2.cvtColor(array, cv2.COLOR_BGR2RGB)  # 确保转换为RGB BGR2RGB
        
        # 只在有订阅者时处理
        if det_image_pub.get_num_connections() > 0:
            det_result = detection_model(rgb_img) # rgb_img
            det_annotated = det_result[0].plot()
            det_image_pub.publish(
                ros_numpy.msgify(Image, det_annotated, encoding="rgb8")
            )
        
        if seg_image_pub.get_num_connections() > 0:
            seg_result = segmentation_model(rgb_img)
            seg_annotated = seg_result[0].plot()
            seg_image_pub.publish(
                ros_numpy.msgify(Image, seg_annotated, encoding="rgb8")
            )
    except Exception as e:
        rospy.logerr(f"Processing error: {str(e)}")

# Subscribe to camera input topic
rospy.Subscriber(
    "/camera/color/image_raw",  # Input image topic (assumed RGB camera)
    Image,  # Message type
    callback,  # Processing function
    queue_size=1  # Small queue to handle latest image only
)

# 正确使用spin保证节点可关闭
rospy.spin()  # 移除了while True循环
