/***********************************************************************
Copyright 2021 Nanjing University of Aeronautics and Astronautics.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
***********************************************************************/

#ifndef ur_VISION_MANAGER
#define ur_VISION_MANAGER

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include "tf/transform_datatypes.h" //RPY转换函数头文件
#include <math.h>

#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/ObjectCount.h>

#include <ur_vision/ShoePoint.h>
#include <ur_vision/ShoePoints.h>

using namespace std;

class VisionManager
{
  public:
	
	/**
   * @brief      VisionManager Constructor Function
   */
	VisionManager(ros::NodeHandle n_);

	/**
	 * @brief      Gets the 2d location of object in camera frame
	 *
	 * @param[in]  img   The image
	 * @param      x     x postion of the object
	 * @param      y     y position of the object
	 */
	void get2DLocation(const sensor_msgs::ImageConstPtr &msg, float &x, float &y);

	/**
	 * @brief      imageCb is called when a new image is received from the camera
	 *
	 * @param[in]  msg   Image received as a message
	 */
	void imageCb(const sensor_msgs::ImageConstPtr &msg);

  private:
	
	//number of detected objects//
	void objectNumber(const darknet_ros_msgs::ObjectCount::ConstPtr &t);

	//get boundingbox of detected objects//
	void objectBox(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg);
	
	/**
 	 * @brief      paper grasppoint estimation
 	 *
 	 * @param      pixel_x  postion of the object in x-pixels
 	 * @param      pixel_y  positino of the object in y-pixels
 	 */
	void detectPaper(const sensor_msgs::ImageConstPtr &msg, float &pixel_x, float &pixel_y, cv::Rect &paperBox);

	void surfaceQuality(const sensor_msgs::ImageConstPtr &msg, cv::Rect &paperBox);

	ur_vision::ShoePoint detectShoe(const sensor_msgs::ImageConstPtr &msg, float &pixel_x, float &pixel_y, cv::Rect &shoeBox);

	void detectBox(const sensor_msgs::ImageConstPtr &msg, float &pixel_x, float &pixel_y, cv::Rect &boxBox);

	//get the distance between two points//
	float getDistance (cv::Point2f pointA, cv::Point2f pointB );

	//get the closest point between the pointC and corners (points)//
	cv::Point2f getClosestPoint (cv::Point2f pointC, std::vector < cv::Point2f > corners );

	void cameraInfoCb( const sensor_msgs::CameraInfo &msg );
	void imageDepthCb( const sensor_msgs::ImageConstPtr &msg );

	geometry_msgs::PointStamped coordinateMap(cv::Point2f graspPoint);

	std::vector <cv::Point2f>  contourCorner(std::vector<std::vector<cv::Point>> &contours);

	void getRotatedRect(cv::Mat image, cv::RotatedRect rotatedRect);

    //图像传输指针//
	cv_bridge::CvImagePtr cv_ptr_;

  	//图像句柄//
	image_transport::ImageTransport it_;

	//图像发布与订阅//
	image_transport::Subscriber image_sub_;
	image_transport::Subscriber	image_sub_depth;//接收深度图像
	
	image_transport::Publisher image1_pub_;
	image_transport::Publisher image2_pub_;
	
	//话题订阅与发布//
	ros::Subscriber objectNumber_sub1;
	ros::Subscriber objectBox_sub2;
	ros::Subscriber camera_info_sub_;//接收深度图像对应的相机参数话题

	ros::Publisher paper_graspPose_pub_;
	ros::Publisher shoe_graspPose_pub_;
	ros::Publisher box_graspPose_pub_;

	cv::Mat	colorImage;
	cv::Mat	depthImage = cv::Mat::zeros( 1280, 720, CV_16UC1 );//注意这里要修改为你接收的深度图像尺寸
	sensor_msgs::CameraInfo camera_info;

	//类内区域变量//
	//i为物体数量//
	int i = 0;
	//k为corners的点序号//
	int k = 0;
	//n: number of shoes//
	int n = 0;
	//write image to files
	bool writeImage = false;
	
	//bool surfaceQualityEstimation = true;
	bool surfaceQualityEstimation = false;

	//detected objects' boundingbox//
	cv::Rect paperBox, boxBox;
	std::vector <cv::Rect> shoeBoxes;

	ur_vision::ShoePoints shoe_graspPose;
	geometry_msgs::PoseStamped	paper_graspPose;
	geometry_msgs::PoseArray box_graspPose;
	geometry_msgs::Pose	box_graspPose1, box_graspPose2, box_graspPose3, box_graspPose4;

};

#endif
