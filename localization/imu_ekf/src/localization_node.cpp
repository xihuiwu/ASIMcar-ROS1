#include "ekf_localization/ekf_localization.h"

#include <math.h>

namespace ekf_localization{
	EkfLocalization::EkfLocalization(ros::NodeHandle nh,
									 ros::NodeHandle private_nh);

	// Get wheelbase info
	wheelbase = nh.getParam("/vesc/wheelbase","0.256");

	// publish 2D pose
	state_pub_ = nh.advertise<geometry_msgs::PoseStamped>("asimcar/pose", 10);

	// subscribe to speedometer and IMU topics
	speed_sub_ = nh.subscribe("sensors/core", 10, &EkfLocalization::speedCallback, this);
	imu_sub_ = nh.subscribe("imu/data_raw", 10, &EkfLocalization::imuCallback, this);

	// initialize pose
	pose->x = 0;
	pose->y = 0;
	pose->theta = 0;

	// initialize sampling time
	pre_time = ros::Time::now();
}

void EkfLocalization::speedCallback(const std_msgs::Float64::ConstPtr& speed_msg){
	speed = speed_msg->data;
}

void EkfLocalization::steerCallback(const std_msgs::Float64::ConstPtr& steer_msg){
	steer = steer_msg->data;
}

// Update car pose based on kinematic bicycle model
void EkfLocalization::imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg){
	// prediction
	time = ros::Time::now();
	dt = time - pre_time;
	prex = pose->x + speed*cos(pose->theta)*dt;
	prey = pose->y + speed*sin(pose->theta)*dt;
	beta = atan(lr/wheelbase*tan(steer));
	pretheta = pose->theta + speed/lr*sin(beta);

	// correction
	

	pre_time = time;
	state_pub_.publish(pose);
}
