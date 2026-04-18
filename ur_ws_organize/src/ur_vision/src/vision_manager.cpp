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

/***********************************************************************
VisionManager
objectNumber
objectBox
imageCb
get2DLocation

detectPaper: 188-455;
surfaceQuality: line 459-892;
detectBox: line 897-1453;

getDistance
getClosestPoint
cameraInfoCb
imageDepthCb
coordinateMap
contourCorner
getRotatedRect

***********************************************************************/

#include "ur_vision/vision_manager.h"

//////////////////////////////////////////////////////////////////////////////////////
//**********************************构造函数*****************************************//
//////////////////////////////////////////////////////////////////////////////////////
//所有的订阅与发布都要放在构造函数中，而不是放在主函数中，因为名称是在类内声明的//
VisionManager::VisionManager(ros::NodeHandle n_) : it_(n_)
{
	objectNumber_sub1 = n_.subscribe("/darknet_ros/found_object", 1, &VisionManager::objectNumber, this);
	objectBox_sub2 = n_.subscribe("/darknet_ros/bounding_boxes", 1, &VisionManager::objectBox, this);

    //aligned depth to color and get the depth data//
	image_sub_depth = it_.subscribe( "/camera/aligned_depth_to_color/image_raw", 1, &VisionManager::imageDepthCb, this );
	camera_info_sub_ = n_.subscribe( "/camera/aligned_depth_to_color/camera_info", 1, &VisionManager::cameraInfoCb, this );

    // Subscribe to input video feed and publish object location//
  	image_sub_  = it_.subscribe("/camera/color/image_raw", 1, &VisionManager::imageCb, this);
    
	//image1_pub_ = it_.advertise("/table_detect", 1);
	image2_pub_ = it_.advertise("/object_detect", 1);

    paper_graspPose_pub_ = n_.advertise<geometry_msgs::PoseStamped>("/paper_graspPose", 10);
    // shoe_graspPose_pub_ = n_.advertise<ur_vision::ShoePoints>("/shoe_graspPose", 1);
    box_graspPose_pub_ = n_.advertise<geometry_msgs::PoseArray>("/box_graspPose", 10);

}

//////////////////////////////////////////////////////////////////////////////////
//***************************obtain boundingbox*********************************//
//////////////////////////////////////////////////////////////////////////////////

//检测到的物体数量i，i为类内区域变量//
void VisionManager::objectNumber(const darknet_ros_msgs::ObjectCount::ConstPtr &t)
{
	i = t->count;
    std::cout<<"ObjectNumber:" << i <<std::endl;
}

//分别提取纸张/鞋子/鞋盒的区域//
//第二只鞋子覆盖了第一只鞋子//
void VisionManager::objectBox(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg)
{
	//在检测到物体之后才能提取区域//
    //检测到的物体数量i，i为类内区域变量//
    n = 0;
	for(int j = 0; j < i; j++)
    {        
        //cv::Rect paperBox0, shoeBox0, boxBox0;
        if(msg->bounding_boxes[j].Class == "paper")
        {
            paperBox.x = msg->bounding_boxes[j].xmin;
            paperBox.y = msg->bounding_boxes[j].ymin;
            paperBox.width = msg->bounding_boxes[j].xmax - msg->bounding_boxes[j].xmin;
            paperBox.height = msg->bounding_boxes[j].ymax - msg->bounding_boxes[j].ymin;
			//扩大矩形，r小写//
	        //paperBox = paperBox0 + cv::Size(100, 100);
            //std::cout<< "paperBox" <<std::endl;
        }
        // else if(msg->bounding_boxes[j].Class == "shoe")
        // {
        //     shoeBoxes[n].x = msg->bounding_boxes[j].xmin;
        //     shoeBoxes[n].y = msg->bounding_boxes[j].ymin;
        //     shoeBoxes[n].width = msg->bounding_boxes[j].xmax - msg->bounding_boxes[j].xmin;
        //     shoeBoxes[n].height = msg->bounding_boxes[j].ymax - msg->bounding_boxes[j].ymin;
        //     n = n+1;
        //     std::cout<< "Shoes number"<<std::endl;
        //     //shoeBox = shoeBox0 + cv::Size(100, 100);
        //     //cout<< shoeBox <<endl;
        // }
        // else
        else if(msg->bounding_boxes[j].Class == "box")
        // if(msg->bounding_boxes[j].Class == "box")
        {
            boxBox.x = msg->bounding_boxes[j].xmin;
            boxBox.y = msg->bounding_boxes[j].ymin;
            boxBox.width = msg->bounding_boxes[j].xmax - msg->bounding_boxes[j].xmin;
            boxBox.height = msg->bounding_boxes[j].ymax - msg->bounding_boxes[j].ymin;
            //boxBox = boxBox0 + cv::Size(100, 100);
            //cout<< paperBox <<endl;
        }
        //std::cout<< "There is detected object"<<std::endl;
    }

    /*
    cout<<"Bouding Boxes (header):" << msg->header <<endl;
    cout<<"Bouding Boxes (image_header):" << msg->image_header <<endl;
    cout<<"Bouding Boxes (Class):" << msg->bounding_boxes[0].Class <<endl;
    cout<<"Bouding Boxes (xmin):" << msg->bounding_boxes[0].xmin <<endl;
    cout<<"Bouding Boxes (xmax):" << msg->bounding_boxes[0].xmax <<endl;
    cout<<"Bouding Boxes (ymin):" << msg->bounding_boxes[0].ymin <<endl;
    cout<<"Bouding Boxes (ymax):" << msg->bounding_boxes[0].ymax <<endl;
    */
}

/////////////////////////////////////////////////////////////////////////////////////
//********************************grasp pose***************************************//
/////////////////////////////////////////////////////////////////////////////////////

void VisionManager::imageCb(const sensor_msgs::ImageConstPtr &msg)
{
    ROS_INFO_STREAM("Processing the Image to locate the Object...");

    //get colorImage
    cv_bridge::CvImagePtr cv_ptr;
    try 
    {
        cv_ptr		= cv_bridge::toCvCopy( msg, sensor_msgs::image_encodings::BGR8 );
        colorImage	= cv_ptr->image;
    } 
    catch ( cv_bridge::Exception &e ) 
    {
        ROS_ERROR( "cv_bridge exception: %s", e.what() );
        return;
    }

    //ROS_INFO("Image Message Received");
    float obj_x, obj_y;
    get2DLocation(msg, obj_x, obj_y);

    // if (paperBox.width > 0 && paperBox.height >0)
    // {
    //     ROS_INFO_STREAM("Go to surfaceQuality function...");
    //     surfaceQuality(msg, paperBox);
    // }

    // Temporary Debugging
    //std::cout<< " X-Co-ordinate in Camera Frame :" << obj_x << std::endl;
    //std::cout<< " Y-Co-ordinate in Camera Frame :" << obj_y << std::endl;
}

void VisionManager::get2DLocation(const sensor_msgs::ImageConstPtr &msg, float &x, float &y)
{
	//cv::Rect tablePos;
	//detectTable(msg, tablePos);

    if (paperBox.width > 0 && paperBox.height >0)
    {
        if (!surfaceQualityEstimation && boxBox.width > 0 && boxBox.height > 0)
        {
            ROS_INFO_STREAM("Go to detectPaper function...");
            detectPaper(msg, x, y, paperBox);
            //detectBox(msg, x, y, boxBox);
        }
        else
        {
            ROS_INFO_STREAM("Go to surfaceQuality function...");
            surfaceQuality(msg, paperBox);
        }
    }

    // for(int j = 0; j < n; j++)
    // {
    //     if (shoeBoxes[j].width > 0 && shoeBoxes[j].height >0)
    //     {
    //         ROS_INFO_STREAM("Go to detectShoe function...");
    //         shoe_graspPose.shoe_points[j] = detectShoe(msg, x, y, shoeBoxes[j]);
    //         shoe_graspPose_pub_.publish(shoe_graspPose);
    //     }
    // }

    // if (boxBox.width > 0 && boxBox.height > 0)
    // {
    //     ROS_INFO_STREAM("Go to detectBox function...");
    //     detectBox(msg, x, y, boxBox);
    // }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//detect paper grasp point//
//Method1 coner detection (grayImage)//
//Method2 threshold boundingRect (binaryImage)//
void VisionManager::detectPaper(const sensor_msgs::ImageConstPtr &msg, float &pixel_x, float &pixel_y, cv::Rect &paperBox)
{

	ROS_INFO_STREAM("Detecting paper now...");
	cv::Mat denoiseImage1, paperHsv, mask;
	float centerx, centery; // center of the image
	std::vector < cv::Point2f > cornersB(4);
	cv::Point2f rectVertex[4], graspPoint, graspPointL, paper_rectCenter, midPoint1, midPoint2, pointMax;
    double r, p, y;
    cv::Rect paperRect; 
    cv::RotatedRect paper_rotatedRect; 
    std::vector< std::vector< cv::Point > > contours;
    double maxarea = 0;  
    int maxAreaIdx = 0;  

    //************************************************************************************

    // obatain the center of the paperRect
    paper_rectCenter.x = paperBox.x + paperBox.width/2;
    paper_rectCenter.y = paperBox.y + paperBox.height/2;

	// median filter
	cv::medianBlur(colorImage, denoiseImage1, 3);
    // denoiseImage2 = denoiseImage1;

	// image segmentation
    cv::floodFill
    (
        denoiseImage1, 
        paper_rectCenter, 
        cv::Scalar(0, 0, 255), 
        &paperRect, 
        cv::Scalar(5, 5, 5),
        cv::Scalar(5, 5, 5)
    );
    if(writeImage)
    {
        cv::imwrite("/home/dongyi/Pictures/paperSegmentation.jpg", denoiseImage1);
        cv::waitKey(500);
    }

    //************************************************************************************

    // change the image space
    cv::cvtColor( denoiseImage1, paperHsv, cv::COLOR_BGR2HSV);

    // obtain a binary image
    //cv::Scalar scalarL = cv::Scalar(80, 43, 46); //desk color
    //cv::Scalar scalarH = cv::Scalar(95, 255, 255);
    cv::Scalar scalarL = cv::Scalar(0, 43, 46); //red color
    cv::Scalar scalarH = cv::Scalar(1, 255, 255);
    //the green background will become to white color//
    cv::inRange(paperHsv, scalarL, scalarH, mask); //mask is a binary image//

    // dilate the image, avoid noise
    dilate(mask, mask, cv::Mat());

    // find the paper contours
    cv::findContours
    (
        mask,
        contours,
        cv::noArray(),
        cv::RETR_EXTERNAL,
        cv::CHAIN_APPROX_SIMPLE
    );

    // black background//
    mask = cv::Scalar::all(0);
    cv::drawContours( mask, contours, -1, cv::Scalar::all(255)); // white contours
    //cv::imshow( "Contours", binaryImage);
    if(writeImage)
    {
        cv::imwrite("/home/dongyi/Pictures/paperContours.jpg", mask);
        cv::waitKey(500);
    }
    
    // find the biggest contour
    for (int index = 0; index < contours.size(); index++)  
    {  
        double tmparea = fabs(contourArea(contours[index]));  
        if (tmparea > maxarea)  
        {  
            maxarea = tmparea;  
            maxAreaIdx = index;//记录最大轮廓的索引号  
        }  
    }

    //********************************************************************************************

    // find the paper rectangle and rectangle center//
    paper_rotatedRect = cv::minAreaRect (contours[maxAreaIdx]);
    
    //get the four corners of the rectangle//
    paper_rotatedRect.points(rectVertex);

    //for (int j = 0; j < 4; j++) 
    //{
	//	
    //    //cv::line(binaryImage, rectVertex[j], rectVertex[(j + 1) % 4], cv::Scalar::all(255), 2, 8);
    //    cv::circle(binaryImage, rectVertex[j], 5, cv::Scalar(0, 255, 0), 2, 8, 0); 
    //    std::cout <<"the paper's rotatedRect cornerSubPix is "<<rectVertex[j]<< '\n' << std::endl;
    //}
    //cv::imshow( "Contours", binaryImage);
    //cv::waitKey(500);
    //std::cout <<"the paperRect cornerSubPix is "<<paperRect.x << paperRect.y<< '\n' << std::endl;
    
    //corner detect//
    //cv::goodFeaturesToTrack
    //(
	//	grayImage,                    // Image to track//
    //    cornersA,                     // Vector of detected corners (output)//
    //    MAX_CORNERS,                  // Keep up to this many corners//
    //    0.01,                         // Quality level (percent of maximum)//
    //    10,                            // Min distance between corners//
    //    cv::noArray(),                // Mask//
    //    3,                            // Block size//
    //    false,                        // true: Harris, false: Shi-Tomasi//
    //    0.04                          // method specific parameter);//
    //);

	//亚像素检测角点//
    //指定亚像素计算迭代标注//
    //cv::TermCriteria criteria = cv::TermCriteria
    //(
    //    cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS,
    //    40,
    //    0.01
    //); 
    //亚像素检测//
    //cornersA here has to be based on the cornersA of function cv::goodFeaturesToTrack//
    //cv::cornerSubPix(grayImage, cornersA, cv::Size(5, 5), cv::Size(-1, -1), criteria); 

    //将检测到的亚像素角点绘制到原图上//
    //cornersB = cornersA;
    //std::cout <<"cornersA.size is "<<cornersA.size()<< std::endl;

    //**********************************************************************************************

    // plot the paper rectangle
    for (int j = 0; j < 4; j++) 
    {
		
        cornersB[j].x = rectVertex[j].x;
		cornersB[j].y = rectVertex[j].y;
        //cv::line(colorImage, cornersB[j], cornersB[(j + 1) % 4], cv::Scalar::all(255), 2, 8);
        cv::circle(colorImage, cornersB[j], 10, cv::Scalar(0, 255, 0), 2, 8, 0); 
        if(writeImage)
        {
            cv::imwrite("/home/dongyi/Pictures/paperCorners.jpg", colorImage);
            cv::waitKey(500);
        }

        //std::cout <<"the paper's cornerSubPix is "<<cornersB[j]<< '\n' << std::endl;
    }

	// find the grasp point//
    // first find the center point//
    centerx = (colorImage.size()).width / 2;
    centery = (colorImage.size()).height / 2;
    //std::cout <<"the center of the image is "<<"("<<centerx<<", "<<centery<<")"<<std::endl;
    cv::Point2f center(centerx, centery);
    //std::cout <<center.x<<std::endl; 

    // second find the first corner point1//
    cv::Point2f point1 = getClosestPoint (center, cornersB);
    //std::cout <<"the paper's point1 is "<<point1<< std::endl;
    //std::cout <<"cornersB.size is "<<cornersB.size()<< std::endl;
    
    // third find the second corner point2//
    // k为类内区域变量，detectPaper函数中用于剔除corners[k]//
    cornersB.erase(remove(cornersB.begin(),cornersB.end(),cornersB[k]),cornersB.end()); 
    //std::cout <<cornersA<< std::endl;
    cv::Point2f point2 = getClosestPoint (point1, cornersB);
    //std::cout <<"the paper's point2 is "<<point2<< std::endl;

    // grasp point
    graspPoint.x = (point1.x + point2.x) / 2;
    graspPoint.y = (point1.y + point2.y) / 2;
    std::cout <<"the paper's grasppoint is "<<graspPoint<< std::endl;
    //cv::circle(colorImage, graspPoint, 5, cv::Scalar(0, 0, 255), 2, 8, 0);
    //cv::imshow("paper_corner_sub", colorImage); 
    //cv::waitKey(1000);

    midPoint1 = graspPoint;
    cornersB.erase(remove(cornersB.begin(),cornersB.end(),cornersB[k]),cornersB.end()); 
    //std::cout <<"cornersC is "<<cornersC<< std::endl;
    midPoint2.x = (cornersB[0].x + cornersB[1].x) / 2;
    midPoint2.y = (cornersB[0].y + cornersB[1].y) / 2;
    //std::cout <<"the box's grasppoint2_temp is "<<grasppoint2_temp<< std::endl;
    //cv::circle(colorImage, grasppoint2_temp, 5, cv::Scalar(0, 0, 255), 2, 8, 0);

    float depthMax = 0.001;
    float aaa;

    // calculate the table height
    for (int i = 480; i <= 520; i++) 
    {
        pointMax.x = midPoint1.x * i/500 + midPoint2.x * (500 - i)/500;
        pointMax.y = midPoint1.y * i/500 + midPoint2.y * (500 - i)/500;
        aaa = coordinateMap(pointMax).point.z;
        
        if (aaa > depthMax)
        {
            depthMax = aaa; //depth is table depth in fact
            //real_grasppoint1 = pointMax; //graspPoint is more accurate position
        }
    }

    float L=148.5;
    //float L=80; 
    graspPointL.x = midPoint1.x * (297 - L)/297 + midPoint2.x * L/297;
    graspPointL.y = midPoint1.y * (297 - L)/297 + midPoint2.y * L/297;

    // for 17g paper (longer)
    // float L=130; 
    // graspPointL.x = midPoint1.x * (303 - L)/303 + midPoint2.x * L/303;
    // graspPointL.y = midPoint1.y * (303 - L)/303 + midPoint2.y * L/303;

	//先查询对齐的深度图像的深度信息，根据读取的camera info内参矩阵求解对应三维坐标
	//float real_z = 0.001 * depthImage.at<u_int16_t>( graspPointL.y, graspPointL.x ); //x y who is the first!!!
    float real_z = depthMax; 
	// float real_x = (graspPoint.x - camera_info.K.at( 2 ) ) / camera_info.K.at( 0 ) * real_z;  //real_z is very important
	// float real_y = (graspPoint.y - camera_info.K.at( 5 ) ) / camera_info.K.at( 4 ) * real_z;
    float real_x = (graspPointL.x - camera_info.K.at( 2 ) ) / camera_info.K.at( 0 ) * real_z;  //strategy1 - grasp centerPoint
	float real_y = (graspPointL.y - camera_info.K.at( 5 ) ) / camera_info.K.at( 4 ) * real_z;

	// //将抓取点坐标打印到图像上
    char tam[100];
	sprintf( tam, "(%0.3f,%0.3f,%0.3f)", real_x, real_y, real_z );
	putText( colorImage, tam, graspPoint, cv::FONT_HERSHEY_SIMPLEX, 0.5, cvScalar(26, 140, 182), 1 );//打印到屏幕上
	cv::circle(colorImage, graspPoint, 10, cv::Scalar(26, 140, 182), 2, 8, 0);
	
    //output_point.header.frame_id = "/camera_depth_optical_frame";
	//output_point.header.stamp = ros::Time::now();
	//output_point.point.x = real_x;
	//output_point.point.y = real_y;
	//output_point.point.z = real_z;
	//arm_point_pub_.publish( output_point );
  
    paper_graspPose.header.frame_id = "camera_color_frame";
    paper_graspPose.header.stamp = ros::Time();
    paper_graspPose.pose.position.x = real_x;
    paper_graspPose.pose.position.y = real_y;
    paper_graspPose.pose.position.z = depthMax; // real_z;

    r = 0;
    p = 0;
    y = - atan ((point2.x - point1.x) / (point2.y - point1.y)) - 3.14/2; // exchange x and y position, = exchange x and y axis
    std::cout <<"the paper's angle is "<< y << std::endl;
    // Z coordinate of the camera's frame down --> -y
    //只通过yaw即绕z的旋转角度计算四元数，用于平面小车。返回四元数
    paper_graspPose.pose.orientation = tf::createQuaternionMsgFromYaw( y );
    //tf::createQuaternionMsgFromRollPitchYaw( - r, p, y);//返回四元数

    paper_graspPose_pub_.publish(paper_graspPose);

    //将抓取点坐标打印到图像上
    // char tam[100];
	// sprintf( tam, "(%0.3f,%0.3f,%0.3f)", paper_graspPose.pose.position.x, paper_graspPose.pose.position.y, paper_graspPose.pose.position.z );
	// putText( colorImage, tam, graspPoint, cv::FONT_HERSHEY_SIMPLEX, 0.6, cvScalar( 0, 0, 255 ), 1 );//打印到屏幕上
	// cv::circle(colorImage, graspPoint, 5, cv::Scalar(0, 0, 255), 2, 8, 0);
    
    if(writeImage)
    {
        cv::imwrite("/home/dongyi/Pictures/paperGraspPose.jpg", colorImage);
        cv::waitKey(500);
    }
    // cv::imshow("paper_corner_sub", colorImage);
    // //paper_depthImage = depthImage( paperBox );
    // //cv::imshow("depthImage", paper_depthImage);
	// cv::waitKey(500);
    
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void VisionManager::surfaceQuality(const sensor_msgs::ImageConstPtr &msg, cv::Rect &paperBox)
{

	ROS_INFO_STREAM("Obtaining surface quality data now...");
	cv::Mat denoiseImage1, paperHsv, mask, paperImage, grayImage, cannyImage;
	float centerx, centery; // center of the image
	std::vector < cv::Point2f > cornersB(4);
	cv::Point2f rectVertex[4], graspPoint, paper_rectCenter, midPoint1, midPoint2, pointMax;
    double r, p, y;
    cv::Rect paperRect; 
    cv::RotatedRect paper_rotatedRect; 
    std::vector< std::vector< cv::Point > > contours0, contours;
    double maxarea = 0;  
    int maxAreaIdx = 0;  
    std::ofstream outfile;

    //************************************************************************************
    
    paperBox.x = paperBox.x*0.97;
    paperBox.y = paperBox.y*0.97;
    paperBox.width = paperBox.width*1.25;
    paperBox.height = paperBox.height*1.25;

    // obatain the center of the paperRect
    paper_rectCenter.x = paperBox.x + paperBox.width/2;
    paper_rectCenter.y = paperBox.y + paperBox.height/2;

	// median filter
	cv::medianBlur(colorImage, denoiseImage1, 3);
    // denoiseImage2 = denoiseImage1;

	// image segmentation
    cv::floodFill
    (
        denoiseImage1, 
        paper_rectCenter, 
        cv::Scalar(0, 0, 255), 
        &paperRect, // combin with paper_rotatedRect line 674
        cv::Scalar(5, 5, 5),
        cv::Scalar(5, 5, 5)
    );
    if(writeImage)
    {
        cv::imwrite("/home/dongyi/Pictures/paperSegmentation.jpg", denoiseImage1);
        cv::waitKey(500);
    }

    //********************************************************************************

    paperImage = colorImage(paperBox); // paperBox is bigger than paperRect
    cv::cvtColor( paperImage, grayImage, cv::COLOR_BGR2GRAY);

    // Canny(grayImage, cannyImage, 50, 100, 3, true); // day
    // Canny(grayImage, cannyImage, 65, 130, 3, true); // 0828good
    Canny(grayImage, cannyImage, 75, 150, 3, true); // 0904good
    // Canny(grayImage, cannyImage, 100, 200, 3, true); // night
    imshow("Canny edge0", cannyImage);
    cv::waitKey(500);

    float edgeLength = 0;
    // int rowNumber = cannyImage.rows;
    // int colNumber = cannyImage.cols;
    for(int i=0; i<cannyImage.rows; i++)
    {
        uchar* data = cannyImage.ptr<uchar>(i);

        for(int j=0; j<cannyImage.cols; j++)
        {
            if (data[j] == 255) {

                edgeLength = edgeLength + (coordinateMap({j,i}).point.z * tan(32.5*M_PI/180) * 2) / 1280;;
                // edgeLength = edgeLength + 1;

                }
        }
    }
    //white contours//
    std::cout << "The edgeLength of creases is " <<edgeLength << std::endl;

    ROS_INFO_STREAM("output edgeLength...");
    outfile.open("edgeLength.dat", ios::app); 
    outfile << edgeLength << " ";
    outfile << "\n";
    outfile.close();

    //**********************************************************************		

    //find the paper contour//
    cv::findContours
    (
        //binaryImage,
        cannyImage,
        contours0,
        cv::noArray(),
        cv::RETR_EXTERNAL,
        cv::CHAIN_APPROX_SIMPLE
    );

    //black background//
    //binaryImage = cv::Scalar::all(0);
    // cannyImage = cv::Scalar::all(0);
    //white contours//
    //cv::drawContours( binaryImage, contours, -1, cv::Scalar::all(255));
    // cv::drawContours( cannyImage, contours0, -1, cv::Scalar::all(255));

    //cv::imshow("contours", binaryImage);
    // cv::imshow("contours", cannyImage);
    // cv::waitKey(500);

    double maxarea0=0;
    int maxAreaIdx0=0;
    for (int index0 = 0; index0 < contours0.size(); index0++)  
    {  
        double tmparea0 = fabs(cv::contourArea(contours0[index0]));  
        if (tmparea0 > maxarea0)  
        {  
            maxarea0 = tmparea0;  
            maxAreaIdx0 = index0; //记录最大轮廓的索引号  
        }  
    //cv::approxPolyDP(contours[index], polyContours[index], 3, true);
    }

    std::cout << "The contour0's size is" << contours0.size() << std::endl;
    double contourLength0 = cv::arcLength(contours0[maxAreaIdx0], true); 
    std::cout << "The contourLength0 is" <<contourLength0<<endl;

    ROS_INFO_STREAM("output contourLength0...");
    outfile.open("contourLength0.dat", ios::app); 
    outfile << contourLength0 << " ";
    outfile << "\n";
    outfile.close();

    //********************************************************************************

    // std::cout <<"the paperRect is "<<paperRect<< std::endl;

    // // std::ofstream outfile;
    // float depthMap[paperRect.width][paperRect.height];
    // ROS_INFO_STREAM("output depthMap...");
    // outfile.open("depthMap.dat", ios::app); 
    // outfile << "    ";

    // for(int i = paperRect.x; i < (paperRect.x + paperRect.width); i++)
    // {
    //     for(int j = paperRect.y; j < (paperRect.y + paperRect.height); j++)
    //     {
    //         double indicator = pointPolygonTest(contour,point,true);
    //         if (indicator >= 0) {
    //             depthMap[i-paperRect.x][j-paperRect.y] = coordinateMap({i,j}).point.z;
                
    //         }
    //         else
    //         {
    //             depthMap[i-paperRect.x][j-paperRect.y] = 0;
    //         }
    //         outfile << depthMap[i-paperRect.x][j-paperRect.y] << " ";
    //     }
    //     outfile << "\n";
    // }

    // outfile.close();

    //********************************************************************************

    // change the image space
    cv::cvtColor( denoiseImage1, paperHsv, cv::COLOR_BGR2HSV);

    // obtain a binary image
    //cv::Scalar scalarL = cv::Scalar(80, 43, 46); //desk color
    //cv::Scalar scalarH = cv::Scalar(95, 255, 255);
    cv::Scalar scalarL = cv::Scalar(0, 43, 46); //red color
    cv::Scalar scalarH = cv::Scalar(1, 255, 255);
    //the green background will become to white color//
    cv::inRange(paperHsv, scalarL, scalarH, mask); //mask is a binary image//

    // dilate the image, avoid noise
    // dilate(mask, mask, cv::Mat());

    // find the paper contours
    cv::findContours
    (
        mask,
        contours,
        cv::noArray(),
        cv::RETR_EXTERNAL,
        cv::CHAIN_APPROX_SIMPLE
    );

    // black background//
    mask = cv::Scalar::all(0);
    cv::drawContours( mask, contours, -1, cv::Scalar::all(255)); // white contours
    //cv::imshow( "Contours", binaryImage);
    if(writeImage)
    {
        cv::imwrite("/home/dongyi/Pictures/paperContours.jpg", mask);
        cv::waitKey(500);
    }
    
    // find the biggest contour
    for (int index = 0; index < contours.size(); index++)  
    {  
        double tmparea = fabs(contourArea(contours[index]));  
        if (tmparea > maxarea)  
        {  
            maxarea = tmparea;  
            maxAreaIdx = index;//记录最大轮廓的索引号  
        }  
    }

    //********************************************************************************

    // double contour_area = fabs((cv::contourArea(contours[maxAreaIdx])));
    // //std::cout << "The paper's contour_area is" << contour_area << std::endl;

    // double contour_length = cv::arcLength(contours[maxAreaIdx], true); 
    // //std::cout << "The contour_length is" <<contour_length<<endl;

    // ROS_INFO_STREAM("output contourData ...");
    // // std::ofstream outfile;
    // outfile.open ("contourData.dat", ios::app);
    // if (!outfile){
    //     std::cerr << "open error!" << std::endl;
    //     exit(1);
    // }
                                                
    // outfile << contour_length << " " << contour_area << "\n";
    // outfile.close();

    //********************************************************************************************

    // paperImage = colorImage(paperRect);
    // cv::cvtColor( paperImage, grayImage, cv::COLOR_BGR2GRAY);

    // Canny(grayImage, cannyImage, 50, 100, 3, true);
    // imshow("Canny edge0", cannyImage);
    // cv::waitKey(500);

    // int edgeLength = 0;
    // // int rowNumber = cannyImage.rows;
    // // int colNumber = cannyImage.cols;
    // for(int i=0; i<cannyImage.cols; i++)
    // {
    //     uchar* data = cannyImage.ptr<uchar>(i);

    //     for(int j=0; j<cannyImage.rows; j++)
    //     {
    //         double indicator = pointPolygonTest(contours[maxAreaIdx], {i+paperRect.x, j+paperRect.y}, true); // contour0 - small figure; contour - big figure
    //         if (indicator > 0 && data[j] == 255) 
    //         {
    //             edgeLength = edgeLength +1;
    //         }
    //     }
    // }
    // //white contours//
    // std::cout << "The edgeLength of creases is " <<edgeLength << std::endl;

    // ROS_INFO_STREAM("output edgeLength...");
    // outfile.open("edgeLength.dat", ios::app); 
    // outfile << edgeLength << " ";
    // outfile << "\n";
    // outfile.close();

    //********************************************************************************************

    // find the paper rectangle and rectangle center//
    paper_rotatedRect = cv::minAreaRect (contours[maxAreaIdx]);

    std::cout <<"the paperBox is "<<paperBox<< std::endl;

    // std::ofstream outfile;
    float depthMap[paperBox.height][paperBox.width];
    ROS_INFO_STREAM("output depthMap...");
    outfile.open("depthMap.dat", ios::app); 
    outfile << "    ";

    for(int j = (paperBox.y + paperBox.height); j > paperBox.y; j--)
    {
        for(int i = paperBox.x; i < (paperBox.x + paperBox.width); i++)
        {
            double indicator = pointPolygonTest(contours[maxAreaIdx], {i,j}, true); // contour0 - small figure; contour - big figure
            if (indicator >= 0) 
            {
                depthMap[j-paperBox.y-1][i-paperBox.x] = coordinateMap({i,j}).point.z;
            }
            else
            {
                depthMap[j-paperBox.y-1][i-paperBox.x] = 0;
            }
            outfile << depthMap[j-paperBox.y-1][i-paperBox.x] << " ";
        }
        outfile << "\n";
    }

    outfile.close();

    //********************************************************************************
    
    // get the four corners of the rectangle//
    paper_rotatedRect.points(rectVertex);

    // plot the paper rectangle
    for (int j = 0; j < 4; j++) 
    {
		
        cornersB[j].x = rectVertex[j].x;
		cornersB[j].y = rectVertex[j].y;
        //cv::line(colorImage, cornersB[j], cornersB[(j + 1) % 4], cv::Scalar::all(255), 2, 8);
        cv::circle(colorImage, cornersB[j], 10, cv::Scalar(0, 255, 0), 2, 8, 0); 
        if(writeImage)
        {
            cv::imwrite("/home/dongyi/Pictures/paperCorners.jpg", colorImage);
            cv::waitKey(500);
        }

        //std::cout <<"the paper's cornerSubPix is "<<cornersB[j]<< '\n' << std::endl;
    }

	// find the grasp point//
    // first find the center point//
    centerx = (colorImage.size()).width / 2;
    centery = (colorImage.size()).height / 2;
    //std::cout <<"the center of the image is "<<"("<<centerx<<", "<<centery<<")"<<std::endl;
    cv::Point2f center(centerx, centery);
    //std::cout <<center.x<<std::endl; 

    // second find the first corner point1//
    cv::Point2f point1 = getClosestPoint (center, cornersB);
    //std::cout <<"the paper's point1 is "<<point1<< std::endl;
    //std::cout <<"cornersB.size is "<<cornersB.size()<< std::endl;
    
    // third find the second corner point2//
    // k为类内区域变量，detectPaper函数中用于剔除corners[k]//
    cornersB.erase(remove(cornersB.begin(),cornersB.end(),cornersB[k]),cornersB.end()); 
    //std::cout <<cornersA<< std::endl;
    cv::Point2f point2 = getClosestPoint (point1, cornersB);
    //std::cout <<"the paper's point2 is "<<point2<< std::endl;

    // grasp point
    graspPoint.x = (point1.x + point2.x) / 2;
    graspPoint.y = (point1.y + point2.y) / 2;
    std::cout <<"the paper's grasppoint is "<<graspPoint<< std::endl;
    //cv::circle(colorImage, graspPoint, 5, cv::Scalar(0, 0, 255), 2, 8, 0);
    //cv::imshow("paper_corner_sub", colorImage); 
    //cv::waitKey(1000);

    midPoint1 = graspPoint;
    cornersB.erase(remove(cornersB.begin(),cornersB.end(),cornersB[k]),cornersB.end()); 
    //std::cout <<"cornersC is "<<cornersC<< std::endl;
    midPoint2.x = (cornersB[0].x + cornersB[1].x) / 2;
    midPoint2.y = (cornersB[0].y + cornersB[1].y) / 2;
    //std::cout <<"the box's grasppoint2_temp is "<<grasppoint2_temp<< std::endl;
    //cv::circle(colorImage, grasppoint2_temp, 5, cv::Scalar(0, 0, 255), 2, 8, 0);

    float depthMax = 0.001;
    float aaa;

    // calculate the table height
    for (int i = 490; i <= 510; i++) 
    {
        pointMax.x = midPoint1.x * i/500 + midPoint2.x * (500 - i)/500;
        pointMax.y = midPoint1.y * i/500 + midPoint2.y * (500 - i)/500;
        aaa = coordinateMap(pointMax).point.z;
        
        if (aaa > depthMax)
        {
            depthMax = aaa; //depth is table depth in fact
            //real_grasppoint1 = pointMax; //graspPoint is more accurate position
        }
    }

    //********************************************************************************

    float depth_line[501];
    ROS_INFO_STREAM("output depthLine...");
    outfile.open("depthLine.dat", ios::app); 

    for (int i = 0; i <= 500; i++) 
    {
        pointMax.x = midPoint1.x * i/500 + midPoint2.x * (500 - i)/500;
        pointMax.y = midPoint1.y * i/500 + midPoint2.y * (500 - i)/500;
        depth_line[i] = coordinateMap(pointMax).point.z;
        outfile << depth_line[i] << " ";
    }

    // if (!outfile){
    //     cerr << "open error!" << std::endl;
    //     exit(1);
    // }

    outfile << "\n";
    outfile.close();

    //********************************************************************************

	//先查询对齐的深度图像的深度信息，根据读取的camera info内参矩阵求解对应三维坐标
	//float real_z = 0.001 * depthImage.at<u_int16_t>( graspPoint.y, graspPoint.x );    //x y who is the first!!!
    float real_z = depthMax; 
	float real_x = (graspPoint.x - camera_info.K.at( 2 ) ) / camera_info.K.at( 0 ) * real_z;  //real_z is very important
	float real_y = (graspPoint.y - camera_info.K.at( 5 ) ) / camera_info.K.at( 4 ) * real_z;

	// //将抓取点坐标打印到图像上
    char tam[100];
	sprintf( tam, "(%0.3f,%0.3f,%0.3f)", real_x, real_y, real_z );
	putText( colorImage, tam, graspPoint, cv::FONT_HERSHEY_SIMPLEX, 0.5, cvScalar(26, 140, 182), 1 );//打印到屏幕上
	cv::circle(colorImage, graspPoint, 10, cv::Scalar(26, 140, 182), 2, 8, 0);
	
    //output_point.header.frame_id = "/camera_depth_optical_frame";
	//output_point.header.stamp = ros::Time::now();
	//output_point.point.x = real_x;
	//output_point.point.y = real_y;
	//output_point.point.z = real_z;
	//arm_point_pub_.publish( output_point );
  
    // paper_graspPose.header.frame_id = "camera_color_frame";
    // paper_graspPose.header.stamp = ros::Time();
    // paper_graspPose.pose.position.x = real_x;
    // paper_graspPose.pose.position.y = real_y;
    // paper_graspPose.pose.position.z = real_z;

    // r = 0;
    // p = 0;
    // y = - atan ((point2.x - point1.x) / (point2.y - point1.y)) - 3.14/2; // exchange x and y position, = exchange x and y axis
    // std::cout <<"the paper's angle is "<< y << std::endl;
    // // Z coordinate of the camera's frame down --> -y
    // //只通过yaw即绕z的旋转角度计算四元数，用于平面小车。返回四元数
    // paper_graspPose.pose.orientation = tf::createQuaternionMsgFromYaw( y );
    // //tf::createQuaternionMsgFromRollPitchYaw( - r, p, y);//返回四元数

    // paper_graspPose_pub_.publish(paper_graspPose);

    //将抓取点坐标打印到图像上
    // char tam[100];
	// sprintf( tam, "(%0.3f,%0.3f,%0.3f)", paper_graspPose.pose.position.x, paper_graspPose.pose.position.y, paper_graspPose.pose.position.z );
	// putText( colorImage, tam, graspPoint, cv::FONT_HERSHEY_SIMPLEX, 0.6, cvScalar( 0, 0, 255 ), 1 );//打印到屏幕上
	// cv::circle(colorImage, graspPoint, 5, cv::Scalar(0, 0, 255), 2, 8, 0);
    
    if(writeImage)
    {
        cv::imwrite("/home/dongyi/Pictures/paperGraspPose.jpg", colorImage);
        cv::waitKey(500);
    }
    cv::imshow("paper_corner_sub", colorImage);
    //paper_depthImage = depthImage( paperBox );
    //cv::imshow("depthImage", paper_depthImage);
	cv::waitKey(500);
    
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//detect box

void VisionManager::detectBox(const sensor_msgs::ImageConstPtr &msg, float &pixel_x, float &pixel_y, cv::Rect &boxBox)
{
    std::vector < cv::Point2f > cornersA(10), cornersB(4);
    float centerx, centery;
    cv::Point2f drawPoint, grasppoint1, grasppoint2, grasppoint3, grasppoint1_temp, grasppoint2_temp, grasppoint3_temp, box_rectCenter13, box_rectCenter23;
    cv::Point2f rectVertex[4], rectPoint[4], rectVertex13[4], rectVertex23[4], rotatedRectCenter13, rotatedRectCenter23, pointMax;
    cv::Mat boxImage, binaryImage, binaryImage2, denoiseImage1, denoiseImage2, boxHsv, mask, mask2;
    //cv::Mat image_rotatedRect, boxImage13, boxImage23;
    double r, p, y1, y2, y3;
    cv::Rect boxRect1, boxRect2;
    cv::RotatedRect box_rotatedRect, box_rotatedRect13, box_rotatedRect23; //
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Point> polyContours;
    double maxarea = 0; 
    int maxAreaIdx = 0; 

    //********************************************************************************
    //get boxRect

    box_rectCenter13.x = boxBox.x + boxBox.width / 3;
    box_rectCenter13.y = boxBox.y + boxBox.height / 2;
    box_rectCenter23.x = boxBox.x + boxBox.width * 2 / 3;
    box_rectCenter23.y = boxBox.y + boxBox.height / 2;

	//colorImage = cv_ptr_->image;
	cv::medianBlur(colorImage, denoiseImage1, 3);
    denoiseImage2 = denoiseImage1.clone();
    cv::cvtColor( denoiseImage2, boxHsv, cv::COLOR_BGR2HSV);

    // output the HSV of the point
    //std::cout << static_cast<int>( boxHsv.at<cv::Vec3b>(box_rectCenter23)[0]) << ",";
    //std::cout << static_cast<int>( boxHsv.at<cv::Vec3b>(box_rectCenter23)[1]) << ",";
    //std::cout << static_cast<int>( boxHsv.at<cv::Vec3b>(box_rectCenter23)[2]) << std::endl;

	//cv::floodFill
    //(
    //    denoiseImage1, 
    //    box_rectCenter13, 
    //    cv::Scalar(0, 0, 255), 
    //    &boxRect1, 
    //    cv::Scalar(5, 5, 5),
    //    cv::Scalar(5, 5, 5)
    //);
	//cv::imshow("redBox1", denoiseImage1);
    //cv::waitKey(500);
    
	cv::floodFill
    (
        boxHsv, 
        box_rectCenter23, 
        cv::Scalar(1, 255, 255), 
        &boxRect2, 
        cv::Scalar(3, 100, 100), 
        cv::Scalar(3, 100, 100)
    );
	//cv::imshow("redBox2", boxHsv);
    //cv::waitKey(500);
    if(writeImage)
    {
        cv::imwrite("/home/dongyi/Pictures/boxSegmentation.jpg", boxHsv);
        cv::waitKey(500);
    }
    
    //********************************************************************

    //cv::cvtColor( denoiseImage2, boxHsv, cv::COLOR_BGR2HSV);
    //cv::Scalar scalarL = cv::Scalar(80, 43, 46);
    //cv::Scalar scalarH = cv::Scalar(95, 255, 255);
    cv::Scalar scalarL = cv::Scalar(0, 255, 255); //red color
    cv::Scalar scalarH = cv::Scalar(1, 255, 255);

    //the green background will become to white color//
    cv::inRange(boxHsv, scalarL, scalarH, mask);
    //black and white inversion//
    //mask = 255-mask;
    //cv::imshow("hsv", mask);
    //cv::waitKey(500);

    //find the selected area from mask image//
    dilate(mask, mask, cv::Mat());

    //find the box contour//
    //cv::namedWindow( "Contours", 1 );
    cv::findContours
    (
        mask,
        contours,
        cv::noArray(),
        cv::RETR_EXTERNAL,
        cv::CHAIN_APPROX_SIMPLE
    );

    //black background//
    mask = cv::Scalar::all(0);
    //white contours//
    cv::drawContours( mask, contours, -1, cv::Scalar::all(255) );
    //cv::imshow( "contours", mask );
    //cv::waitKey(500);
    if(writeImage)
    {
        cv::imwrite("/home/dongyi/Pictures/boxContours.jpg", mask);
        cv::waitKey(500);
    }
    
    // finding the biggest contour is more robust
    //查找最大轮廓  
 
    for (int index = 0; index < contours.size(); index++)  
    {  
        double tmparea = fabs(cv::contourArea(contours[index]));  
        if (tmparea > maxarea)  
        {  
            maxarea = tmparea;  
            maxAreaIdx = index;//记录最大轮廓的索引号  
        }  

       //cv::approxPolyDP(contours[index], polyContours[index], 3, true);
    }

    std::cout <<"the biggestContour's length is "<< contours[maxAreaIdx].size() << '\n' << std::endl;
    cv::approxPolyDP(contours[maxAreaIdx], polyContours, 10, true);

    //black background//
    //mask = cv::Scalar::all(0);
    //white contours//
    //cv::drawContours( polyPic, polyContours, maxAreaIdx, cv::Scalar::all(255));
    for (int i = 0; i < polyContours.size(); ++i)
    {
        cv::line(mask, polyContours[i], polyContours[(i + 1) % polyContours.size()], cv::Scalar::all(255), 2, 8);
        
        //cv::circle(mask, polyContours[i], 1, cv::Scalar(0, 255, 0), 2, 8, 0);
    }
    //cv::imshow( "contours", mask);
    //cv::waitKey(500);
    if(writeImage)
    {
        cv::imwrite("/home/dongyi/Pictures/boxPolyContours.jpg", mask);
        cv::waitKey(500);
    }
    

    std::vector<int> hull;
    convexHull(polyContours, hull, false);
    
    //cornersA.size() = hull.size();
    for (int i = 0; i < hull.size(); ++i)
    {
        cornersA[i] = polyContours[hull[i]];
        //cv::circle(colorImage, polyContours[hull[i]], 5, cv::Scalar(0, 255, 0), 2, 8, 0);
        
    }
    //cv::imwrite("/home/dongyi/Pictures/PolyContours", mask);
    //cv::waitKey(500);
    
    //**************************************************************************************
    // rotatedRect of Box
    // get four corners
    
    // get the corners of the contours
    // precision is not good
    //cornersA = contourCorner(contours);

    //将检测到的亚像素角点绘制到原图上//
    //cornersB = cornersA;
    
    //for (int j = 0; j < cornersA.size(); j++) 
    //{
    //    cornersB[j].x = cornersA[j].x + boxRect2.x;
	//	  cornersB[j].y = cornersA[j].y + boxRect2.y;
    //    cv::circle(colorImage, cornersB[j], 5, cv::Scalar(0, 255, 0), 2, 8, 0); 
    //    //std::cout <<"the box's cornerSubPix is "<<cornersB[j]<< '\n' << std::endl; 
    //}
    
    //find the paper rectangle and rectangle center//
    box_rotatedRect = cv::minAreaRect (contours[maxAreaIdx]); 
    
    box_rotatedRect.points(rectVertex);
    for (int i = 0; i < 4; ++i)
    {
        cv::line(mask, rectVertex[i], rectVertex[(i + 1) % 4], cv::Scalar::all(255), 2, 8);
       
        cornersB[i] = getClosestPoint(rectVertex[i], cornersA);
        cv::circle(colorImage, cornersB[i], 10, cv::Scalar(0, 255, 0), 2, 8, 0);
    }
    if(writeImage)
    {
        cv::imwrite("/home/dongyi/Pictures/boxMinRect.jpg", mask);
        cv::waitKey(500);
    }
    if(writeImage)
    {
        cv::imwrite("/home/dongyi/Pictures/boxCorners.jpg", colorImage);
        cv::waitKey(500);
    }
 
    

    //box_rotatedRect.center.x = box_rotatedRect.center.x + boxRect2.x;
    //box_rotatedRect.center.y = box_rotatedRect.center.y + boxRect2.y;
    //getRotatedRect(mask, box_rotatedRect);
    
    // left part and right part
    //image_rotatedRect = binaryImage2(box_rotatedRect);
    //rect13.x = int(std::floor(box_rotatedRect.center.x - (box_rotatedRect.size.width / 2)));
    //rect13.y = int(std::floor(box_rotatedRect.center.y - (box_rotatedRect.size.height/2)));
    //rect13.width = int(std::floor(box_rotatedRect.size.width / 3));
    //rect13.height = int(std::ceil(box_rotatedRect.size.height));
    
    //rect13 &= cv::Rect(0, 0, mask2.cols, mask2.rows);//求交集
    //boxImage13 = mask2(rect13);
    //boxImage23 = mask2(rect23);
    //cv::imshow( "boxImage13", boxImage13);
    //cv::waitKey(500);

    //***********************************************************************************************
    //find the box contour//

    //cv::namedWindow( "Contours23", 1 );
    //cv::findContours
    //(
    //    boxImage23,
    //    contours,
    //    cv::noArray(),
    //    cv::RETR_EXTERNAL,
    //    cv::CHAIN_APPROX_SIMPLE
    //);

    //black background//
    //boxImage23 = cv::Scalar::all(0);
    //white contours//
    //cv::drawContours( boxImage23, contours, -1, cv::Scalar::all(255));
    //cv::imshow( "Contours13", boxImage23);
    //cv::waitKey(500);
    
    // finding the biggest contour is more robust
    //查找最大轮廓  
    //maxarea = 0;  
    //maxAreaIdx = 0;
    //for (int index = 0; index < contours.size(); index++)  
    //{  
    //    double tmparea = fabs(cv::contourArea(contours[index]));  
    //    if (tmparea > maxarea)  
    //    {  
    //        maxarea = tmparea;  
    //        maxAreaIdx = index;//记录最大轮廓的索引号  
    //    }  
    //}

    //find the paper rectangle and rectangle center//
    //box_rotatedRect23 = cv::minAreaRect (contours[maxAreaIdx]); 

    //get the four corners of the rectangle//
    //box_rotatedRect23.points(rectVertex23);

    //for (int j = 0; j < 4; j++) 
    //{
		
    //    cv::line(boxImage23, rectVertex23[j], rectVertex23[(j + 1) % 4], cv::Scalar::all(255), 2, 8);
    //}
    //cv::imshow( "Contours23", boxImage23);
    //cv::waitKey(500);

    //取較小的兩個角點
    //for (int j = 0; j < 4; j++) 
    //{
    //    if (rectVertex23[j].x > box_rotatedRect.width * 3 / 4)
    //    {
    //        cornersA[i] = rectVertex23[j];
    //        i = i + 1;
    //        std::cout <<"the rectVertex23's cornerSubPix is "<<rectVertex23[j]<< '\n' << std::endl;
    //    }
    //}

    //***************************************************************************************

    //find the grasp point//
    //first find the center point//
    centerx = (colorImage.size()).width / 2;
    centery = (colorImage.size()).height / 2;
    std::cout <<"the center of the boxImage is "<<"("<<centerx<<", "<<centery<<")"<<std::endl;
    cv::Point2f center(centerx, centery);
    //std::cout <<center.x<<std::endl;

    //second find the first corner point1//
    cv::Point2f point1 = getClosestPoint (center, cornersB);
    std::cout <<"the box's point1 is "<<point1<< std::endl;
    
    //std::cout <<k<< std::endl;
    //third find the second corner point2//
    cornersB.erase(remove(cornersB.begin(),cornersB.end(),cornersB[k]),cornersB.end()); 
    //std::cout <<cornersA<< std::endl;
    cv::Point2f point2 = getClosestPoint (center, cornersB);
    std::cout <<"the box's point2 is "<<point2<< std::endl;

    //finally grasp point
    grasppoint1.x = (point1.x + point2.x) / 2;
    grasppoint1.y = (point1.y + point2.y) / 2;
    std::cout <<"the box's grasppoint1 is "<<grasppoint1<< std::endl;
    //cv::circle(colorImage, grasppoint1_temp, 5, cv::Scalar(0, 0, 255), 2, 8, 0);



    //***************************************************************************************

    cornersB.erase(remove(cornersB.begin(),cornersB.end(),cornersB[k]),cornersB.end()); 
    cv::Point2f point3 = getClosestPoint (center, cornersB);
    std::cout <<"the box's point3 is "<<point3<< std::endl;

    cornersB.erase(remove(cornersB.begin(),cornersB.end(),cornersB[k]),cornersB.end()); 
    cv::Point2f point4 = getClosestPoint (center, cornersB);
    std::cout <<"the box's point4 is "<<point4<< std::endl;

    grasppoint2.x = (point3.x + point4.x) / 2;
    grasppoint2.y = (point3.y + point4.y) / 2;
    std::cout <<"the box's grasppoint2 is "<<grasppoint2<< std::endl;
    cv::circle(colorImage, grasppoint2, 5, cv::Scalar(0, 0, 255), 2, 8, 0);

    //***************************************************************************************

    // cornersB.erase(remove(cornersB.begin(),cornersB.end(),cornersB[k]),cornersB.end()); 
    // //std::cout <<"cornersC is "<<cornersC<< std::endl;
    // grasppoint2_temp.x = (cornersB[0].x + cornersB[1].x) / 2;
    // grasppoint2_temp.y = (cornersB[0].y + cornersB[1].y) / 2;
    // std::cout <<"the box's grasppoint2_temp is "<<grasppoint2_temp<< std::endl;
    // //cv::circle(colorImage, grasppoint2_temp, 5, cv::Scalar(0, 0, 255), 2, 8, 0);

    // rotatedRectCenter13.x = grasppoint1_temp.x * 2/3 + grasppoint2_temp.x * 1/3;
    // rotatedRectCenter13.y = grasppoint1_temp.y * 2/3 + grasppoint2_temp.y * 1/3;
    // rotatedRectCenter23.x = grasppoint1_temp.x * 1/3 + grasppoint2_temp.x * 2/3;
    // rotatedRectCenter23.y = grasppoint1_temp.y * 1/3 + grasppoint2_temp.y * 2/3;
    
    // if (coordinateMap(rotatedRectCenter13).point.z < coordinateMap(rotatedRectCenter23).point.z)
    // {
    //     grasppoint1 = grasppoint1_temp;
    //     grasppoint2 = grasppoint2_temp;
    // }
    // else
    // {
    //     grasppoint1 = grasppoint2_temp;
    //     grasppoint2 = grasppoint1_temp;
    // }

    // cv::Point2f pointMin, real_grasppoint1, real_grasppoint2;
    // real_grasppoint1 = grasppoint1;
    // real_grasppoint2 = grasppoint2;

    /*for(float th=0.0; th<6.28; th=th+0.01)
    {
        pointMin.x = grasppoint1.x + 20 * cos(th);
        pointMin.y = grasppoint1.y + 20 * sin(th);

        if (coordinateMap(pointMin).point.z < coordinateMap( real_grasppoint1 ).point.z)
        {
            real_grasppoint1 = pointMin;
        }
    }*/

    float depthMax = 0.0001;
    // float aaa, bbb, ccc, ddd;

    for (int i = 500; i <= 625; i++) 
    {
        pointMax.x = grasppoint1.x * i/500 + grasppoint2.x * (500 - i)/500;
        pointMax.y = grasppoint1.y * i/500 + grasppoint2.y * (500 - i)/500;
        float depthTemp = coordinateMap(pointMax).point.z;
        
        if (depthTemp > depthMax)
        {
            depthMax = depthTemp;
            // aaa = depthTemp;
            // real_grasppoint1 = pointMin;
        }
    }

    // depthMin = 10000;

    // for (int i = -20; i <= 20; i++) 
    // {
    //     pointMin.x = grasppoint1.x * i/500 + grasppoint2.x * (500 - i)/500;
    //     pointMin.y = grasppoint1.y * i/500 + grasppoint2.y * (500 - i)/500;
    //     float depthTemp = coordinateMap(pointMin).point.z;
        
    //     if (depthTemp < depthMin && fabs(depthTemp) > 0.001)
    //     {
    //         depthMin = depthTemp; 
    //         bbb = depthTemp;
    //         real_grasppoint2 = pointMin;
    //     }

    //     if (depthTemp > depthMax)
    //     {
    //         depthMax = depthTemp; 
    //         ddd = depthTemp;
    //     }
    // }

    // grasppoint1 = real_grasppoint1;
    // grasppoint2 = real_grasppoint2;
    cv::circle(colorImage, grasppoint1, 10, cv::Scalar(147, 62, 155), 2, 8, 0);
    // cv::circle(colorImage, grasppoint2, 10, cv::Scalar(147, 62, 155), 2, 8, 0);
    if(writeImage)
    {
        cv::imwrite("/home/dongyi/Pictures/boxGraspPoint12.jpg", colorImage);
        cv::waitKey(500);
    }
    
    /*
    //***************************************************************************************
    //graspPoint3
    int deltaX = fabs((grasppoint2.x - grasppoint1.x) * 2 / 3);
    float depthMin = 100000;
    cv::Point2f pointMin;

    for (int i = 0; i < deltaX; i++) 
    {
        //grasppoint2 is on the right of grasppoint1
        float k = (grasppoint2.y - grasppoint1.y) / (grasppoint2.x - grasppoint1.x);
        int yTemp = round( k * i + grasppoint1.y );
        cv::Point2f pointTemp;
        pointTemp.y = yTemp;
        pointTemp.x = (k > 0)?(grasppoint1.x + i):(grasppoint1.x - i);

        float depthTemp = coordinateMap(pointTemp).point.z;
        if (depthTemp < depthMin)
        {
            depthMin = depthTemp;
            pointMin = pointTemp;
        }
    }

    grasppoint3 = pointMin;
    std::cout <<"the box's grasppoint3 is "<<grasppoint3<< std::endl;
    */

    //***************************************************************************************
    // //graspPoint3
    // depthMin = 10000;
    // //cv::Point2f pointMin;

    // for (int i = 100; i < 400; i++) 
    // {
    //     pointMin.x = grasppoint1.x * i/500 + grasppoint2.x * (500 - i)/500;
    //     pointMin.y = grasppoint1.y * i/500 + grasppoint2.y * (500 - i)/500;
    //     float depthTemp = coordinateMap(pointMin).point.z;
        
    //     if (depthTemp < depthMin && fabs(depthTemp) > 0.001)
    //     {
    //         depthMin = depthTemp;
    //         ccc = depthTemp;
    //         grasppoint3 = pointMin;
    //     }
    // }

    // std::cout <<"the box's grasppoint3 is "<<grasppoint3<< std::endl;

    // cv::circle(colorImage, grasppoint3, 10, cv::Scalar(147, 62, 155), 2, 8, 0);
    // if(writeImage)
    // {
    //     cv::imwrite("/home/dongyi/Pictures/boxGraspPoint123.jpg", colorImage);
    //     cv::waitKey(500);
    // }

    //***************************************************************************************

	//先查询对齐的深度图像的深度信息，根据读取的camera info内参矩阵求解对应三维坐标
    //grasppoint3 mark
    // float real_z = ccc;
	// //float real_z = 0.001 * depthImage.at<u_int16_t>( grasppoint3.y, grasppoint3.x ); //x y who is the first!!!
	// float real_x = (grasppoint3.x - camera_info.K.at( 2 ) ) / camera_info.K.at( 0 ) * real_z; //real_z is very important
	// float real_y = (grasppoint3.y - camera_info.K.at( 5 ) ) / camera_info.K.at( 4 ) * real_z;
	// char tam[100];
	// sprintf( tam, "(%0.3f, %0.3f, %0.3f)", real_x, real_y, real_z );
	// putText( colorImage, tam, grasppoint3, cv::FONT_HERSHEY_SIMPLEX, 0.5, cvScalar(147, 62, 155), 1 );//打印到屏幕上
    // box_graspPose3.position.x = real_x; 
    // box_graspPose3.position.y = real_y;
    // box_graspPose3.position.z = real_z;

    // //grasppoint2 mark
    // real_z = ccc;
    // //real_z = 0.001 * depthImage.at<u_int16_t>( grasppoint2.y, grasppoint2.x );    //x y who is the first!!!
	// real_x = (grasppoint2.x - camera_info.K.at( 2 ) ) / camera_info.K.at( 0 ) * real_z;  //real_z is very important
	// real_y = (grasppoint2.y - camera_info.K.at( 5 ) ) / camera_info.K.at( 4 ) * real_z;
	// //char tam[100];
	// sprintf( tam, "(%0.3f, %0.3f, %0.3f)", real_x, real_y, real_z );
	// putText( colorImage, tam, grasppoint2, cv::FONT_HERSHEY_SIMPLEX, 0.5, cvScalar(147, 62, 155), 1 );//打印到屏幕上
    // box_graspPose2.position.x = real_x; 
    // box_graspPose2.position.y = real_y;
    // box_graspPose2.position.z = real_z;

    // //grasppoint4 mark
    // box_graspPose4.position.x = real_x;
    // box_graspPose4.position.y = real_y;
    // box_graspPose4.position.z = ddd;

    //grasppoint1 mark
    // real_z = aaa;
    float real_z = 0.001 * depthImage.at<u_int16_t>( grasppoint1.y, grasppoint1.x );    //x y who is the first!!!
    float real_x = (grasppoint1.x - camera_info.K.at( 2 ) ) / camera_info.K.at( 0 ) * real_z;  //real_z is very important
	float real_y = (grasppoint1.y - camera_info.K.at( 5 ) ) / camera_info.K.at( 4 ) * real_z;
	char tam[100];
	sprintf( tam, "(%0.3f, %0.3f, %0.3f)", real_x, real_y, real_z );
	putText( colorImage, tam, grasppoint1, cv::FONT_HERSHEY_SIMPLEX, 0.5, cvScalar(147, 62, 155), 1 );//打印到屏幕上
    box_graspPose1.position.x = real_x; 
    box_graspPose1.position.y = real_y;
    // box_graspPose1.position.z = real_z;
    box_graspPose1.position.z = depthMax;

    //***************************************************************************************
    
    // box_graspPose1.position = coordinateMap( grasppoint1 ).point;
    // box_graspPose2.position = coordinateMap( grasppoint2 ).point;
    // box_graspPose3.position = coordinateMap( grasppoint3 ).point;

    //box_graspPose2.position.z = box_graspPose3.position.z;

    //实际可以只定义一个位姿，其他的抓取点都可以使用定义的位姿
    r = 0;
    p = 0;
    y1 = - atan ((point2.x - point1.x) / (point2.y - point1.y)) - 3.14/2; // exchange x and y position, = exchange x and y axis
    // y3 = - atan ((point2.x - point1.x) / (point2.y - point1.y)) + 3.14/2;
    //std::cout <<"the box's angle is "<< y1 << std::endl;
    // Z coordinate of the camera's frame down --> -y
    //只通过yaw即绕z的旋转角度计算四元数，用于平面小车。返回四元数
    box_graspPose1.orientation = tf::createQuaternionMsgFromYaw( y1 );
    // box_graspPose2.orientation = tf::createQuaternionMsgFromYaw( y2 );
    // box_graspPose4.orientation = tf::createQuaternionMsgFromYaw( y2 );
    // box_graspPose3.orientation = tf::createQuaternionMsgFromYaw( y3 );
    //tf::createQuaternionMsgFromRollPitchYaw( - r, p, y);//返回四元数

    box_graspPose.header.frame_id = "camera_color_frame";
    box_graspPose.header.stamp = ros::Time();
    box_graspPose.poses.clear();
    box_graspPose.poses.push_back(box_graspPose1);
    // box_graspPose.poses.push_back(box_graspPose2);
    // box_graspPose.poses.push_back(box_graspPose3);
    // box_graspPose.poses.push_back(box_graspPose4);

    // std::cout <<"box_graspPose4 is "<<box_graspPose4<< std::endl;

    box_graspPose_pub_.publish(box_graspPose);

    // char tam[100];
	// sprintf( tam, "(%0.3f, %0.3f, %0.3f)", box_graspPose3.position.x, box_graspPose3.position.y, box_graspPose3.position.z);
	// //putText( colorImage, tam, grasppoint3, cv::FONT_HERSHEY_SIMPLEX, 0.6, cvScalar( 0, 0, 255 ), 1 );//打印到屏幕上

    // sprintf( tam, "(%0.3f, %0.3f, %0.3f)", box_graspPose2.position.x, box_graspPose2.position.y, box_graspPose2.position.z);
	// putText( colorImage, tam, grasppoint2, cv::FONT_HERSHEY_SIMPLEX, 0.6, cvScalar( 0, 0, 255 ), 1 );//打印到屏幕上

    // sprintf( tam, "(%0.3f, %0.3f, %0.3f)", box_graspPose1.position.x, box_graspPose1.position.y, box_graspPose1.position.z);
	// putText( colorImage, tam, grasppoint1, cv::FONT_HERSHEY_SIMPLEX, 0.6, cvScalar( 0, 0, 255 ), 1 );//打印到屏幕上

    imshow("box_corner_sub", colorImage); 
    cv::waitKey(500);

    //cv::imwrite("/home/dongyi/Pictures/21LUX.jpg", colorImage);
    //cv::waitKey(500);
    
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
//*****************************************help function********************************************//
////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////
//get closest point

//get distance between two points//
float VisionManager::getDistance(cv::Point2f pointA, cv::Point2f pointB )
{
    float distance;
    distance = powf((pointA.x - pointB.x),2) + powf((pointA.y - pointB.y),2);
    distance = sqrtf(distance); 

    return distance;
}

//get the closest point between the pointC and corners (points)//
cv::Point2f VisionManager::getClosestPoint(cv::Point2f pointC, std::vector < cv::Point2f > corners )
{
    int k_temp = 0;
	float mindistance, realdistance;
    cv::Point2f realPoint;

	//change the point of paperImage to colorImage
	//realPoint.x = corners[0].x;
	//realPoint.y = corners[0].y;
    mindistance = getDistance(pointC, corners[0]);

    for (int j = 0; j < corners.size(); j++) 
    {
		realdistance = getDistance(pointC, corners[j]);

        if (mindistance > realdistance)
        {
            mindistance = realdistance;
            k_temp = j;
        }
    }

	//k为类内区域变量，detectPaper函数中用于剔除corners[k]//
    realPoint = corners[k_temp];
    k = k_temp;
    return realPoint;
}

///////////////////////////////////////////////////////////////////////////////////////////////
//get depth data 

//get cameraInfo
void VisionManager::cameraInfoCb( const sensor_msgs::CameraInfo &msg )
{
    camera_info = msg;
}

void VisionManager::imageDepthCb( const sensor_msgs::ImageConstPtr &msg )
{
	cv_bridge::CvImagePtr cv_ptr;

	try 
    {
		cv_ptr = cv_bridge::toCvCopy( msg, sensor_msgs::image_encodings::TYPE_16UC1 );
		depthImage = cv_ptr->image;
	} 
    catch ( cv_bridge::Exception &e ) 
    {
		ROS_ERROR( "cv_bridge exception: %s", e.what() );
		return;
	}
}

//////////////////////////////////////////////////////////////////////////////////////////////
//pixel coordinate to camera coordinate

geometry_msgs::PointStamped VisionManager::coordinateMap(cv::Point2f graspPoint)
{
	geometry_msgs::PointStamped	outputPoint;

    //float distance_list[100]; 
 
    // mid_pos = [int(shoeCenter[0]), int(shoeCenter[1])] #确定索引深度的中心像素位置
    // min_val = 6 #确定深度搜索范围
    // for i in range(randnum):
    //     bias = random.randint(-min_val//2, min_val//2)
    //     dist = depthImage[int(mid_pos[1] + bias), int(mid_pos[0] + bias)]
    //     # cv2.circle(frame, (int(mid_pos[0] + bias), int(mid_pos[1] + bias)), 4, (255,0,0), -1)
    //     # print(int(mid_pos[1] + bias), int(mid_pos[0] + bias))
    //     if dist:
    //         distance_list.append(dist)
    // distance_list = np.array(distance_list)
    // distance_list = np.sort(distance_list)[randnum//2-randnum//4:randnum//2+randnum//4] #冒泡排序+中值滤波
    // #print(distance_list, np.mean(distance_list))
    // depth1 = np.mean(distance_list)

    //先查询对齐的深度图像的深度信息，根据读取的camera info内参矩阵求解对应三维坐标
	float real_z = 0.001 * depthImage.at<u_int16_t>( graspPoint.y, graspPoint.x );    //x y who is the first!!!
	float real_x = (graspPoint.x - camera_info.K.at( 2 ) ) / camera_info.K.at( 0 ) * real_z;  //real_z is very important
	float real_y = (graspPoint.y - camera_info.K.at( 5 ) ) / camera_info.K.at( 4 ) * real_z;

	/*
    char tam[100];
	sprintf( tam, "(%0.2f,%0.2f,%0.2f)", real_x, real_y, real_z );
	putText( colorImage, tam, graspPoint, cv::FONT_HERSHEY_SIMPLEX, 0.6, cvScalar( 0, 0, 255 ), 1 );//打印到屏幕上
	*/
    outputPoint.header.frame_id = "/camera_depth_optical_frame";
	outputPoint.header.stamp = ros::Time::now();
	outputPoint.point.x = real_x;
	outputPoint.point.y = real_y;
	outputPoint.point.z = real_z;
	return outputPoint;
    
}

/////////////////////////////////////////////////////////////////////////////////////////////////
//求出轮廓的尖锐角点

std::vector <cv::Point2f>  VisionManager::contourCorner(std::vector<std::vector<cv::Point>> &contours)
{
    //遍历轮廓，求出所有支撑角度
    float fmax = -1; //用于保存局部最大值
    int   imax = -1;
    bool  bstart = false;
    int j = 0;
    std::vector <cv::Point2f> cornersA(100);
    //std::vector< std::vector< cv::Point2f > > contours;
    double maxarea = 0;  
    int maxAreaIdx = 0; 

    // finding the biggest contour is more robust
    //查找最大轮廓  
    for (int index = 0; index < contours.size(); index++)  
    {  
        double tmparea = fabs(cv::contourArea(contours[index]));  
        if (tmparea > maxarea)  
        {  
            maxarea = tmparea;  
            maxAreaIdx = index;//记录最大轮廓的索引号  
        }  
    }

    int icount = contours[maxAreaIdx].size();
    for (int i=0; i < contours[maxAreaIdx].size(); i++)
    {
        cv::Point2f pa = contours[maxAreaIdx][(i + icount - 30) % icount];
        cv::Point2f pb = contours[maxAreaIdx][(i + icount + 30) % icount];
        cv::Point2f pc = contours[maxAreaIdx][i];

        //两支撑点距离
        float fa = getDistance(pa, pb);
        float fb = getDistance(pa, pc) + getDistance(pb, pc);
        float fang = fa / fb;
        float fsharp = 1 - fang;

        if (fsharp>0.1)
        {
            bstart = true;
            if (fsharp>fmax)
            {
                fmax = fsharp;
                imax = i;
            }
        }
        else
        {
            if (bstart)
            {
                cornersA[j] = contours[maxAreaIdx][imax];
                //circle(board,bigestContour[imax],10,Scalar(255),1);
                //circle(src,bigestContour[imax],10,Scalar(255,255,255),1);
                imax  = -1;
                fmax  = -1;
                bstart = false;
                j = j + 1;
            }
        }
    }  

    return cornersA;
}

//从图像变换为旋转矩形的区域
void VisionManager::getRotatedRect(cv::Mat image, cv::RotatedRect rotatedRect)
{
    
	cv::Point2f vertices[4];
	rotatedRect.points(vertices);//外接矩形的4个顶点
	//for (int i = 0; i < 4; i++)//画矩形
    //{
    //    line(imageSource, vertices[i], vertices[(i + 1) % 4], Scalar(255, 0, 0));
    //}
	
	/*Rect brect = rect.boundingRect();
	rectangle(imageSource, brect, Scalar(255, 0, 0));*/
	//imshow("Source Image1", imageSource);
	cv::Point2f center = rotatedRect.center;//外接矩形中心点坐标
	cv::Mat rot_mat = getRotationMatrix2D(center, rotatedRect.angle, 1.0);//求旋转矩阵
	//cv::Mat rot_image;
	cv::Size dst_sz(image.size());
	//cv::warpAffine(imageSource, rot_image, rot_mat, dst_sz);//原图像旋转
    cv::warpAffine(image, image, rot_mat, dst_sz);//原图像旋转
    //图像是以旋转长方形的中心进行旋转的
	//image = image(cv::Rect(center.x - (rotatedRect.size.width / 2), center.y - (rotatedRect.size.height/2), rotatedRect.size.width, rotatedRect.size.height));//提取ROI
	//imshow("result", result1);

}

////////////////////////////////////////////////////////////////////////
//*************************main function******************************//
////////////////////////////////////////////////////////////////////////

//Main Function
int main(int argc, char** argv ) 
{
  	ros::init(argc, argv, "simple_grasping_vision_detection");
  	ros::NodeHandle n_;

  	ROS_INFO_STREAM("Waiting for two seconds..");
  	ros::WallDuration(2.0).sleep();

	VisionManager vm(n_);

	while (ros::ok())
	{
		// Process image callback
		ros::spinOnce();

	}
	return 0;
}
