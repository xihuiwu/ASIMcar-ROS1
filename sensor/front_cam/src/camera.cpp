#include "front_cam/camera.h"

namespace camera
{

inline void string2double(std::string array, double *data, int size);

Camera::Camera(ros::NodeHandle nh, ros::NodeHandle private_nh)
{
	private_nh.param<int>("width", width, 720);
	private_nh.param<int>("height", height, 1280);
	private_nh.param<int>("fps", fps, 1280);
	private_nh.param<bool>("pipeline_flag", pipeline_flag, false);
	//frame_raw_gpu = cv::cuda::GpuMat(height, width, CV_8UC1);
	frame_gpu = cv::cuda::GpuMat(height, width, CV_8UC3);	
	
	private_nh.param<std::string>("intrinsic", paramK, "0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0");
	private_nh.param<std::string>("extrinsic", paramD, "0.0, 0.0, 0.0, 0.0");
	string2double(paramK, dataK, 9);
	string2double(paramD, dataD, 4);
	K = cv::Mat(cv::Size(3, 3), CV_64FC1, dataK);
	D = cv::Mat(cv::Size(4, 1), CV_64FC1, dataD);
	cv::fisheye::initUndistortRectifyMap(K, D, cv::Matx33d::eye(), K, cv::Size(height, width), CV_32FC1, mapx, mapy);
	mapx_gpu.upload(mapx);
	mapy_gpu.upload(mapy);
	
	private_nh.param<std::string>("filename", filename, "");

	color_pub = private_nh.advertise<sensor_msgs::Image>("undistort", 1);
	gray_pub = private_nh.advertise<sensor_msgs::Image>("undistort_gray", 1);

}

inline void string2double(std::string array, double *data, int size)
{
	std::stringstream ss(array);
	for (int i=0; i<size; ++i)
	{
		ss >> *(data+i);
	}
}

void Camera::run()
{	
	boost::thread thread1(&Camera::capture, this);
	
	while (ros::ok())
	{	
		if (captured)
		{
			// clock_t t;
			// t = clock();
			std::cout << frame_raw.dims << std::endl;
			frame_raw_gpu.upload(frame_raw);
			std::cout << "done uploading" << std::endl;
			cv::cuda::cvtColor(frame_raw_gpu, frame_gpu, cv::COLOR_YUV2BGR);
			std::cout << "done decoding" << std::endl;
			cv::cuda::remap(frame_gpu, undistort_gpu, mapx_gpu, mapy_gpu, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
			cv::cuda::cvtColor(undistort_gpu, undistort_gray_gpu, cv::COLOR_BGR2GRAY);

			undistort_gpu.download(undistort);
			undistort_gray_gpu.download(undistort_gray);
			//t = clock() - t;
			//std::cout << (double)t/CLOCKS_PER_SEC << std::endl;
			
			/*header.stamp = ros::Time::now();
			color_msg = cv_bridge::CvImage(header, "bgr8", undistort).toImageMsg();
			gray_msg = cv_bridge::CvImage(header, "mono8", undistort_gray).toImageMsg();

			color_pub.publish(color_msg);
			gray_pub.publish(gray_msg);
			
			captured = !captured;*/
		}
	}
	cap.release();
}

void Camera::capture()
{
	if (pipeline_flag == 1)
	{
		cap.open(filename, cv::CAP_GSTREAMER);
	}
	else
	{
		cap.open(filename, cv::CAP_V4L2);
		cap.set(cv::CAP_PROP_FRAME_WIDTH, width);
		cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);
		cap.set(cv::CAP_PROP_FPS, fps);
		cap.set(cv::CAP_PROP_CONVERT_RGB, false);
	}
	
	while (cap.isOpened())
	{
		captured = cap.read(frame_raw);
	}
}


}
