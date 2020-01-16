#include "camera/camera.h"

namespace camera
{
Camera::Camera(ros::NodeHandle nh, ros::NodeHandle private_nh)
{
	if (private_nh.hasParam("intrinsic")) { private_nh.getParam("intrinsic", dataK) }
	if (private_nh.hasParam("extrinsic")) { private_nh.getParam("extrinsic", dataD) }
	K = Mat(3, 3, CV_64FC1, dataK);
	D = Mat(4, 1, CV_64FC1, dataD);
	cv::fisheye::initUndistortRectifyMap(K, D, Matx33d::eye(), K, frame.size(), CV_32FC1, mapx, mapy);
	mapx_gpu.upload(mapx);
	mapy_gpu.upload(mapy);

	if (!private_nh.hasParam("pipeline")) {"port", filename}
	else { private_nh.getParam("pipeline", filename) }

	color_pub = private_nh.advertise<sensor_msgs::Image>("undistort", 1);
	gray_pub = private_nh.advertise<sensor_msgs::Image>("undistort_gray", 1);

}

Camera::run()
{	
	cap.open(filename);
	while (ros::ok())
	{
		cap.read(frame)
		if (frame.empty) break;
		else
		{
			frame_gpu.upload(frame);
			cuda::remap(frame_gpu, frame_undistort_gpu, mapx_gpu, mapy_gpu, INTER_LINEAR, BORDER_CONSTANT);
			cuda::cvtColor(undistort_gpu, undistort_gray_gpu, CV_BGR2GRAY);

			undistort_gpu.convertTo(undistort, cv::CV_8UC3);
			undistort_gray_gpu.convertTo(undistort_gray, cv::CV_8UC3);

			header.stamp = ros::Time::now();
			color_msg = cv_bridge::CvImage(header, "bgr8", undistort).toImageMsg();
			gray_msg = cv_bridge::CvImage(header, "mono8", undistort_gray).toImageMsg();

			color_pub.publish(color_msg);
			gray_pub.publish(gray_msg);
		}
	}
}

}