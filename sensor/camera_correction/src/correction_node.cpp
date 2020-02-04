#include <ros/ros.h>

#include "camera_correction/camera_correction.h"

int main(int argc, char** argv)
{	
	ros::init(argc, argv, "camera_node");
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");

	Correction correction(nh, private_nh);
	correction.run();

	return 0;
}
