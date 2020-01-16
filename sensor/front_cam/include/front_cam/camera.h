#ifndef CAMERA_H_
#define CAMERA_H_

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/cudawarping.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudaarithm.hpp>

#include <iostream>
#include <linux/videodev2.h>
#include <linux/ioctl.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>

namespace camera
{

class Camera
{
public:
	Camera(ros::NodeHandle nh, ros::NodeHandle private_nh);
  
private:
	string filename; // pipeline definition
	int width, height;

	VideoCapture cap; // video capture object
	Mat frame(width, height, cv::CV_8UC3); // frame
	cuda::GpuMat frame_gpu; // frame on GPU

	double dataK[9], dataD[4];
	Mat K, D; // camera intrinsic and extrinsic matrices
	Mat mapx, mapy; // map function 
	cuda::GpuMat mapx_gpu, mapy_gpu, frame_gpu, undistort_gpu, undistort_gray_gpu; // variables for un-distortation
	Mat undistort, undistort_gray;
	
	std_msgs::Header header;
	sensor_msgs::ImagePtr color_ptr, gray_ptr; // image messages of colored and gray images

	ros::Publisher color_pub, gray_pub; // publisher for color and gray images
};

} // namespace camera
  
#endif // CAMERA_H_