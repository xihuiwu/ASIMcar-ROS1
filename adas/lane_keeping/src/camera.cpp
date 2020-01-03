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
	int direction; // direction of lane keeping
	cuda::GpuMat mask_gpu, frame_corp_gpu;
	Mat lane_region_hls, lane_hls, lane_hls_erosion; // initialize mat for imgproc
	cuda::GpuMat frame_gpu, frame_mono_gpu, frame_undistort_gpu, frame_gray_gpu; // initialize gpumat for un-distortion
	cuda::GpuMat lane_region_gpu, lane_region_hls_gpu; // initialize gpumat for imgproc
	cv_bridge::CvImage img_bridge1;
	cv_bridge::CvImage img_bridge2;
	cv_bridge::CvImage img_bridge3;
	sensor_msgs::Image img_msg1;
	sensor_msgs::Image img_msg2;
	sensor_msgs::Image img_msg3;
	std_msgs::Header header;

	ros::init(argc, argv, "front_camera");
	ros::NodeHandle n;
	ros::Publisher pub1 = n.advertise<sensor_msgs::Image>("/front_cam/lane",1);
	ros::Publisher pub2 = n.advertise<sensor_msgs::Image>("/front_cam/undistort_mono",1);
	ros::Publisher pub3 = n.advertise<sensor_msgs::Image>("/front_cam/mono",1);
	n.param("/lane_detection/direction", direction, 2);
	

	// Define HSL white range
	Scalar low_white = Scalar(0, 200, 0);
	Scalar up_white = Scalar(255, 255, 255);
	
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
	
	// Define corp mask matrix
	int num_points = 4;
	Mat mask(1080, 1920, CV_8UC3);
	Point corners[4];
	if (direction==1){
		corners[0] = Point(750,580);
		corners[1] = Point(960,580);
		corners[2] = Point(960,690);
		corners[3] = Point(380,690);
		const Point* corner_list[1] = {corners};
		fillPoly(mask, corner_list, &num_points, 1, Scalar(255,255,255), 8);
		mask_gpu.upload(mask);
	}
	else if (direction==2){
		corners[0] = Point(960,580);
		corners[1] = Point(1170,580);
		corners[2] = Point(1540,690);
		corners[3] = Point(960,690);
		const Point* corner_list[1] = {corners};
		fillPoly(mask, corner_list, &num_points, 1, Scalar(255,255,255), 8);
		mask_gpu.upload(mask);
	}
	
	// Define perspective matrix
	Point2f source_points[4];
	source_points[0] = Point2f(700,580);
	source_points[1] = Point2f(1220,580);
	source_points[2] = Point2f(1540,690);
	source_points[3] = Point2f(380,690);
	Point2f dest_points[4];
	dest_points[0] = Point2f(0,0);
	dest_points[1] = Point2f(800,0);
	dest_points[2] = Point2f(700,400);
	dest_points[3] = Point2f(100,400);
	Mat perspectiveM = getPerspectiveTransform(source_points, dest_points);
	
	// Define erosion matrix
	Mat M = getStructuringElement(MORPH_RECT, Size(3, 3), Point(1, 1));
	
	
	boost::thread thread1(capture);
	while(ros::ok()){
		if (!frame.empty()){
			frame_gpu.upload(frame);
			cuda::cvtColor(frame_gpu, frame_mono_gpu, CV_BGR2GRAY);
			Mat frame_mono(frame_mono_gpu);
			
			cuda::remap(frame_gpu, frame_undistort_gpu, mapx_gpu, mapy_gpu, INTER_LINEAR, BORDER_CONSTANT);
			
			// Gray Image
			cuda::cvtColor(frame_undistort_gpu, frame_gray_gpu, CV_BGR2GRAY);
			Mat frame_gray(frame_gray_gpu);
		
			// Corp Image
			cuda::bitwise_and(frame_undistort_gpu, mask_gpu, frame_corp_gpu);
		
			// Perspective Transfrorm
			cuda::warpPerspective(frame_corp_gpu, lane_region_gpu, perspectiveM, Size(800,400));

			// Color threshold
			cuda::cvtColor(lane_region_gpu, lane_region_hls_gpu, CV_BGR2HLS);
			Mat lane_region_hls(lane_region_hls_gpu);
			inRange(lane_region_hls,low_white,up_white,lane_hls); // lane_hls value 0-255
			img_bridge1 = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, lane_hls);
			img_bridge2 = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, frame_gray);
			img_bridge3 = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, frame_mono);
			img_bridge1.toImageMsg(img_msg1);
			img_bridge2.toImageMsg(img_msg2);
			img_bridge3.toImageMsg(img_msg3);
			pub1.publish(img_msg1);
			pub2.publish(img_msg2);
			pub3.publish(img_msg3);
			
			/*imshow("test",lane_hls);
			waitKey(1);*/
		}
	}
	
	return 0;
}
