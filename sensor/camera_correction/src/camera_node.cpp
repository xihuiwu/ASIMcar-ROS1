#include <ros/ros.h>

#include "front_cam/camera.h"

int main(int argc, char** argv)
{	
	ros::init(argc, argv, "camera_node");
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");

	camera::Camera camera(nh, private_nh);
	camera.run();

	return 0;
}
