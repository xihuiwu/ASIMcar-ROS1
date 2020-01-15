// -*- mode:c++; fill-column: 100; -*-

#include "vesc_ackermann/ackermann_to_vesc.h"

#include <cmath>
#include <sstream>

#include <std_msgs/Float64.h>

namespace vesc_ackermann
{

template <typename T>
inline bool getRequiredParam(const ros::NodeHandle& nh, std::string name, T& value);

AckermannToVesc::AckermannToVesc(ros::NodeHandle nh, ros::NodeHandle private_nh)
{
  // get conversion parameters
  if (!getRequiredParam(nh, "/vesc/speed_to_erpm_gain", speed_to_erpm_gain_))
    return;
  if (!getRequiredParam(nh, "/vesc/erpm_offset", erpm_offset_))
    return;
  if (!getRequiredParam(nh, "/vesc/steering_angle_to_servo_gain", steering_to_servo_gain_))
    return;
  if (!getRequiredParam(nh, "/vesc/servo_offset", servo_offset_))
    return;
  if (!getRequiredParam(nh, "/vesc/torque_constant", torque_constant_))
    return;
  if (!getRequiredParam(nh, "/asimcar/mass", vehicle_mass_))
    return;
  if (!getRequiredParam(nh, "/asimcar/wheel_radius", wheel_radius_))
    return;
  if (!getRequiredParam(nh, "/asimcar/gear_ratio", gear_ratio_))
    return;
  if (!getRequiredParam(nh, "/vesc/current_offset", current_offset_))
    return;

  // create publishers to vesc electric-RPM (speed) and servo commands
  erpm_pub_ = nh.advertise<std_msgs::Float64>("commands/motor/speed", 10);
  current_pub_ = nh.advertise<std_msgs::Float64>("commands/motor/current", 10);
  servo_pub_ = nh.advertise<std_msgs::Float64>("commands/servo/position", 10);

  // subscribe to ackermann topic
  ackermann_sub_ = nh.subscribe("ackermann_cmd", 10, &AckermannToVesc::ackermannCmdCallback, this);
}

typedef ackermann_msgs::AckermannDriveStamped::ConstPtr AckermannMsgPtr;
void AckermannToVesc::ackermannCmdCallback(const AckermannMsgPtr& cmd)
{
  // calc vesc electric RPM (speed)
  std_msgs::Float64::Ptr erpm_msg(new std_msgs::Float64);
  erpm_msg->data = speed_to_erpm_gain_ * cmd->drive.speed + erpm_offset_;

  // calc steering angle (servo)
  std_msgs::Float64::Ptr servo_msg(new std_msgs::Float64);
  servo_msg->data = steering_to_servo_gain_ * cmd->drive.steering_angle + servo_offset_;
  
  // calc current (acceleration)
  std_msgs::Float64::Ptr current_msg(new std_msgs::Float64);
  current_msg->data = cmd->drive.acceleration * vehicle_mass_ * wheel_radius_ * 4 / gear_ratio_ / torque_constant_ + current_offset_;

  // publish
  if (ros::ok()) {
    erpm_pub_.publish(erpm_msg);
    servo_pub_.publish(servo_msg);
    current_pub_.publish(current_msg);
  }
}

template <typename T>
inline bool getRequiredParam(const ros::NodeHandle& nh, std::string name, T& value)
{
  if (nh.getParam(name, value))
    return true;

  ROS_FATAL("AckermannToVesc: Parameter %s is required.", name.c_str());
  return false;
}

} // namespace vesc_ackermann
