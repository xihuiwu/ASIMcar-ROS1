#include "camera_correction/camera_correction.h"

inline void string2double(std::string array, double *data, int size);

Correction::Correction(ros::NodeHandle nh, ros::NodeHandle private_nh)
{
	private_nh.param<int>("width", width, 1920);
	private_nh.param<int>("height", height, 1080);
	
	private_nh.param<std::string>("intrinsic", paramK, "0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0");
	private_nh.param<std::string>("extrinsic", paramD, "0.0, 0.0, 0.0, 0.0");
	string2double(paramK, dataK, 9);
	string2double(paramD, dataD, 4);
	K = cv::Mat(cv::Size(3, 3), CV_64FC1, dataK);
	D = cv::Mat(cv::Size(4, 1), CV_64FC1, dataD);
	cv::fisheye::initUndistortRectifyMap(K, D, cv::Matx33d::eye(), K, cv::Size(width, height), CV_32FC1, mapx, mapy);
	mapx_gpu.upload(mapx);
	mapy_gpu.upload(mapy);

	raw_sub = private_nh.subscribe("image_raw", 1, &Correction::rawCB, this);
	correction_pub = private_nh.advertise<sensor_msgs::Image>("image_undistort", 1);
}

inline void string2double(std::string array, double *data, int size)
{
	std::stringstream ss(array);
	for (int i=0; i<size; ++i)
	{
		ss >> *(data+i);
	}
}

void Correction::rawCB(const sensor_msgs::Image::ConstPtr& raw_msg)
{	
	img_ptr = cv_bridge::toCvCopy(raw_msg, sensor_msgs::image_encodings::RGB8);
	frame_raw_gpu.upload(img_ptr->image);
	//std::cout << "done uploading" << std::endl;
	cv::cuda::remap(frame_raw_gpu, frame_undistort_gpu, mapx_gpu, mapy_gpu, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
	frame_undistort_gpu.download(img_ptr->image);
	//std::cout << "done correction" << std::endl;
	correction_pub.publish(img_ptr->toImageMsg());
}

void Correction::run()
{
	ros::spin();
}
