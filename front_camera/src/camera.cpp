#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Header.h>
#include <boost/thread/thread.hpp>

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

using namespace cv;
using namespace std;


Mat frame(1080, 1920, CV_8UC3);
string pipeLine = "v4l2src device=/dev/video1 extra-controls=\"exposure_auto=0, white_balance_temperature_auto=1\" ! jpegdec ! video/x-raw, width=1920, height=1080, framerate=30/1 ! videoconvert ! appsink";

void capture(){
	VideoCapture cap;
	cap.open(pipeLine);
	while(ros::ok()){
		cap.read(frame);
	}
}


int main(int argc, char **argv){
	
	string pipeLine;
	cuda::GpuMat frame_gpu, frame_undistort_gpu, frame_gray_gpu; // initialize gpumat for un-distortion
	cv_bridge::CvImage img_bridge1;
	cv_bridge::CvImage img_bridge2;
	sensor_msgs::Image img_msg1;
	sensor_msgs::Image img_msg2;
	std_msgs::Header header;

	ros::init(argc, argv, "front_camera");
	ros::NodeHandle n;
	ros::Publisher pub1 = n.advertise<sensor_msgs::Image>("/front_cam/undistort",1);
	ros::Publisher pub2 = n.advertise<sensor_msgs::Image>("/front_cam/undistort_mono",1);
	
	
	// Define mapping matrix
	double dataK[9] = {797.291347853076, 0.0, 954.0911522248246, 0.0, 797.7486635926731, 524.5785087770129 ,0.0, 0.0, 1.0};
	double dataNewK[9] = {797.291347853076/4, 0.0, 954.0911522248246, 0.0, 797.7486635926731/4, 524.5785087770129 ,0.0, 0.0, 1.0};
	double dataD[4] = {-0.022046185426876273, -0.0014811900456108225, -0.002085499019587609, 0.0005973311977065658};
	Mat K = Mat(3,3,CV_64FC1,dataK);
	Mat newK = Mat(3,3,CV_64FC1,dataNewK);
	Mat D = Mat(4,1,CV_64FC1,dataD);
	Mat mapx, mapy;
	fisheye::initUndistortRectifyMap(K,D,Matx33d::eye(),newK,frame.size(),CV_32FC1,mapx,mapy);
	cuda::GpuMat mapx_gpu, mapy_gpu;
	mapx_gpu.upload(mapx);
	mapy_gpu.upload(mapy);
	
	ROS_INFO("Initialize Camera...");
	boost::thread thread1(capture);
	ROS_INFO("Done Initialization!");
	while(ros::ok()){
		if (!frame.empty()){
			frame_gpu.upload(frame);
			cuda::remap(frame_gpu, frame_undistort_gpu, mapx_gpu, mapy_gpu, INTER_LINEAR, BORDER_CONSTANT);
			Mat frame(frame_gpu);
			
			// Gray Image
			cuda::cvtColor(frame_undistort_gpu, frame_gray_gpu, CV_BGR2GRAY);
			Mat frame_gray(frame_gray_gpu);
			
			// Convert Image to Msg and Publish
			img_bridge1 = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, frame);
			img_bridge2 = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, frame_gray);
			img_bridge1.toImageMsg(img_msg1);
			img_bridge2.toImageMsg(img_msg2);
			pub1.publish(img_msg1);
			pub2.publish(img_msg2);
			
			/*imshow("test",lane_hls);
			waitKey(1);*/
		}
	}
	ROS_INFO("Turn off camera...");
	
	return 0;
}
