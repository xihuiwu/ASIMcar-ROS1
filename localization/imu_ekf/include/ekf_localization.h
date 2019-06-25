// -*- mode:c++; fill-column: 100; -*-

#ifndef EKF_LOCALIZATION_EKF_LOCALIZATION_H_
#define EKF_LOCALIZATION_EKF_LOCALIZATION_H_

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose2D.h>

namespace ekf_localization
{

class EkfLocalization{
public:
	EkfLocalization(ros::NodeHandle nh,
					ros::NodeHandle private_nh);

private:
	// variables
	float speed, steer;
	geometry_msgs::Pose2D pose;
	float prex, prey, pretheta, beta;
	float pre_time, time, dt;

	// ROS services
	ros::Publisher state_pub_;
	ros::Subscriber speed_sub_;
	ros::Subscriber imu_sub_;

	// ROS callbacks
	void steeringCallback(const std_msgs::Float64::ConstPtr& speed_msg);
	void speedCallback(const std_msgs::Float64::ConstPtr& speed_msg);
	void steerCallback(const std_msgs::Float64::ConstPtr& steer_msg);
	void imuCallback(const sensor_msgs::Imu::ConstPtr& imu);
};

}

#endif // EKF_LOCALIZATION_EKF_LOCALIZATION_H_
