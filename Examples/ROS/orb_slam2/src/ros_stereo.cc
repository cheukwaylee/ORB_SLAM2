/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <opencv2/core/core.hpp>

#include "../../../include/System.h"

using namespace std;
using namespace cv;

cv::Mat K; //camera matrix
cv::Mat D; //distoration matrix
int img_width;
int img_height;
float bf; //baseline*f
float fps;
int rgb; //0: BGR, 1: RGB. It is ignored if images are grayscale


bool camera_info_initialized = false;

ros::Subscriber camera_info_sub;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight);

    ORB_SLAM2::System* mpSLAM;
};

void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg)
{
	ROS_INFO("camera info msg");
	K = Mat::zeros(3, 3, CV_32FC1);
    D = Mat::zeros(5, 1, CV_32FC1);
    cout << "line: " << __LINE__ << endl;
    float fx = msg->K[0];
    float fy = msg->K[4];
    float cx = msg->K[2];
    float cy = msg->K[5];
    cout << "line: " << __LINE__ << endl;
    K.at<float>(0, 0) = fx;
    K.at<float>(1, 1) = fy;
    K.at<float>(0, 2) = cx;
    K.at<float>(1, 2) = cy;
    K.at<float>(2, 2) = 1;
    cout << "line: " << __LINE__ << endl;
    if(msg->D.size() == 5)
    {
	    D.at<float>(0) = msg->D[0];
	    D.at<float>(1) = msg->D[1];
	    D.at<float>(2) = msg->D[2];
	    D.at<float>(3) = msg->D[3];
	    D.at<float>(4) = msg->D[4];
    }

    img_width = msg->width;
    img_height = msg->height;
    cout << "line: " << __LINE__ << endl;
	// baseline = fabs(msg->P[3]/fx);  
	bf = fabs(msg->P[3]);  
	rgb = 1; //todo
	fps = 30;
    cout << "baseline: " << bf/fx << "\n fx:  " << fx << "\nfy: " << fy << "\ncx: " << cx << "\ncy: " << cy << endl;
    camera_info_initialized = true;
    cout << "line: " << __LINE__ << endl;
    camera_info_sub.shutdown();
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "Stereo");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Stereo path_to_vocabulary path_to_settings" << endl;
        ros::shutdown();
        return 1;
    }

    ros::NodeHandle nh("~");

    camera_info_sub = nh.subscribe("right_camera_info", 1000, cameraInfoCallback);
    ros::Rate loop_rate(50);
    while(ros::ok())
    {

    	ros::spinOnce();
    	if(camera_info_initialized)
    	{
    		break;
    	}
    	loop_rate.sleep();
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2], K,  D, img_width, img_height, bf, fps, rgb, ORB_SLAM2::System::STEREO,true);
    ImageGrabber igb(&SLAM);

    message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, "left_image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, "right_image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub,right_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabStereo,&igb,_1,_2));


    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight)
{
    
	if(!camera_info_initialized) return;

    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrLeft,cv_ptrRight;
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    mpSLAM->TrackStereo(cv_ptrLeft->image,cv_ptrRight->image,cv_ptrLeft->header.stamp.toSec());
}


