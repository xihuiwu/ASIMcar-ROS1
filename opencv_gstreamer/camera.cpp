#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/core/mat.hpp>
#include <string>
#include <iostream>

std::string pipeline = "v4l2src device=/dev/video0 io-mode=2 ! image/jpeg, width=1920, height=1080, framerate=30/1 ! jpegparse ! nvjpegdec ! videoconvert ! appsink sync=false";
cv::VideoCapture cap;
cv::cuda::GpuMat frame_gpu;
cv::Mat frame;

int main(void)
{	
	std::cout << "Opening" << std::endl;
	cap.open(0, cv::CAP_V4L2);
	cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
	cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
	cap.set(cv::CAP_PROP_FPS, 60);
	std::cout << "Open Successfully" << std::endl;
	while (1)
	{
		cap.read(frame);
		//frame_gpu.download(frame);
		
		cv::imshow("frame",frame);
		cv::waitKey(1);
	}
	return 0;
}

//v4l2src device=/dev/video2 io-mode=2 ! 'video/x-raw,framerate=9/1,width=1280,height=720' ! jpegparse ! nvjpegdec ! video/x-raw ! nvvidconv ! 'video/x-raw(memory:NVMM)' ! appsink sync=false

