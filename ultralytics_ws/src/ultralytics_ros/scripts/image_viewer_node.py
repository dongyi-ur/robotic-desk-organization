#! /home/dongyi/anaconda3/envs/yolo_ros/bin/python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ImageViewer:
    def __init__(self):
        self.bridge = CvBridge()
        
        # Create subscribers for detection and segmentation results
        rospy.Subscriber("/ultralytics/detection/image", Image, self.detection_callback)
        # rospy.Subscriber("/ultralytics/segmentation/image", Image, self.segmentation_callback)
        
        # Create windows for display
        cv2.namedWindow("Detection Results", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Segmentation Results", cv2.WINDOW_NORMAL)
        
        rospy.loginfo("Image viewer node initialized. Press 'q' to quit.")

    def detection_callback(self, msg):
        """Callback for detection results"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
            cv2.imshow("Detection Results", cv_image)
            cv2.waitKey(1)
        except Exception as e:
            rospy.logerr(f"Detection display error: {str(e)}")

    def segmentation_callback(self, msg):
        """Callback for segmentation results"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
            cv2.imshow("Segmentation Results", cv_image)
            cv2.waitKey(1)
        except Exception as e:
            rospy.logerr(f"Segmentation display error: {str(e)}")

    def run(self):
        """Main loop"""
        while not rospy.is_shutdown():
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        cv2.destroyAllWindows()

if __name__ == '__main__':
    rospy.init_node('ultralytics_image_viewer')
    viewer = ImageViewer()
    viewer.run()
