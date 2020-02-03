#ifndef CAMERA_H_
#define CAMERA_H_

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <std_msgs/Header.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/cudacodec.hpp>
#include <opencv2/cudawarping.hpp> // remap
#include <opencv2/cudaimgproc.hpp> // cvtColor

#include <iostream>
#include <string.h>

#include <boost/thread/thread.hpp>
#include <time.h>

namespace camera
{

class Camera
{
public:
	Camera(ros::NodeHandle nh, ros::NodeHandle private_nh);
	void run();
	void capture();
  
private:
	std::string filename; // pipeline definition
	int width, height, fps;
	bool pipeline_flag; // flag for whether choosing pipeline

	cv::VideoCapture cap; // video capture object
	bool captured; // check whether next frame is captured
	cv::Mat frame, frame_raw; // frame

	std::string paramK, paramD;
	double dataK[9], dataD[4];
	cv::Mat K, D; // camera intrinsic and extrinsic matrices
	cv::Mat mapx, mapy; // map function 
	cv::cuda::GpuMat mapx_gpu, mapy_gpu, frame_raw_gpu, frame_gpu, undistort_gpu, undistort_gray_gpu; // variables for un-distortation
	cv::Mat undistort, compressed;
	
	std_msgs::Header header;
	sensor_msgs::ImagePtr color_msg, compressed_msg; // image messages of colored and gray images

	ros::Publisher color_pub, compress_pub; // publisher for color and gray images
};

} // namespace camera
  
#endif // CAMERA_H_
