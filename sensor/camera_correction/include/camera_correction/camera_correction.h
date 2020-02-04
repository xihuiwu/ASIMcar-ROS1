#ifndef CAMERA_H_
#define CAMERA_H_

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/cudawarping.hpp> // remap
#include <opencv2/cudaimgproc.hpp> // cvtColor

#include <iostream>
#include <string.h>

#include <time.h>

class Correction
{
public:
	Correction(ros::NodeHandle nh, ros::NodeHandle private_nh);
	void run();
	void correction();
  
private:
	int width, height;
	std::string paramK, paramD;
	double dataK[9], dataD[4];
	cv::Mat K, D; // camera intrinsic and extrinsic matrices
	cv::Mat mapx, mapy; // map function 
	cv::cuda::GpuMat mapx_gpu, mapy_gpu, frame_raw_gpu, frame_undistort_gpu; // variables for un-distortation
	
	cv_bridge::CvImagePtr img_ptr;
	ros::Subscriber raw_sub;
	ros::Publisher correction_pub; // publisher for color and gray images
	
	void rawCB(const sensor_msgs::Image::ConstPtr& raw_msg);
};
  
#endif // CAMERA_H_
