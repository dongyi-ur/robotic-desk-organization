#!/home/dongyi/anaconda3/envs/YOLOnew/bin/python

import sys
# # sys.path.append('/home/dongyi/anaconda3/lib/python3.9/site-packages')
sys.path.append('/home/dongyi/anaconda3/envs/YOLOnew/lib/python3.12/site-packages')
print (sys.path)
sys.path.remove('/usr/lib/python3/dist-packages')
sys.path.remove('/home/dongyi/realsense_ros_ws/devel/lib/python3/dist-packages')
print (sys.version)
print (sys.path)

import time

import ros_numpy
import rospy
from sensor_msgs.msg import Image

from ultralytics import YOLO

try:
    rospy.init_node('yolo', anonymous=True)
    print("Node initialized successfully")
except Exception as e:
    print(f"Init node failed: {e}")
    sys.exit(1)
print("GoGoGo0")
time.sleep(1)

print("GoGoGo1")
detection_model = YOLO("yolo11n.pt")
segmentation_model = YOLO("yolo11n-seg.pt")
print("GoGoGo2")

det_image_pub = rospy.Publisher('/ultralytics/detection/image', Image, queue_size=5)
seg_image_pub = rospy.Publisher('/ultralytics/segmentation/image', Image, queue_size=5)


def callback(data):
    """Callback function to process image and publish annotated images."""
    array = ros_numpy.numpify(data)
    print("Received an image for processing.")

    # if det_image_pub.get_num_connections():
    det_result = detection_model(array)
    det_annotated = det_result[0].plot(show=False)
    det_image_pub.publish(ros_numpy.msgify(Image, det_annotated, encoding="rgb8"))
    print("Published detection image.")

    # if seg_image_pub.get_num_connections():
    seg_result = segmentation_model(array)
    seg_annotated = seg_result[0].plot(show=False)
    seg_image_pub.publish(ros_numpy.msgify(Image, seg_annotated, encoding="rgb8"))
    print("Published segmentation image.")


rospy.Subscriber('/camera/color/image_raw', Image, callback)
print("Node initialized and ready to process images.")

while True:
    rospy.spin()

