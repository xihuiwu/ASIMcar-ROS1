#!/usr/bin/env python
import rospy

from std_msgs.msg import Float64

# import some utils.
import numpy as np
import copy as copy

class InterpolateThrottle:
	def __init__(self):

		# Allow our topics to be dynamic.
		self.rpm_input_topic   = rospy.get_param('~rpm_input_topic', '/vesc/commands/motor/speed_unsmoothed')
		self.rpm_output_topic  = rospy.get_param('~rpm_output_topic', '/vesc/commands/motor/speed')

		self.servo_input_topic   = rospy.get_param('~servo_input_topic', '/vesc/commands/servo/position_unsmoothed')
		self.servo_output_topic  = rospy.get_param('~servo_output_topic', '/vesc/commands/servo/position')

		self.current_input_topic =  rospy.get_param('~current_input_topic', '/vesc/commands/motor/current_unsmoothed')
		self.current_output_topic = rospy.get_param('~current_input_topic', '/vesc/commands/motor/current')
		
		self.smooth_rate = rospy.get_param('/vesc/smooth_rate')
		
		self.max_acceleration = rospy.get_param('/vesc/max_acceleration')
		self.speed_to_erpm_gain = rospy.get_param('/vesc/speed_to_erpm_gain')
		self.max_rpm = rospy.get_param('/vesc/vesc_driver/speed_max')
		self.min_rpm = rospy.get_param('/vesc/vesc_driver/speed_min')

		self.max_steering_speed = rospy.get_param('/vesc/max_steering_speed')
		self.steering_angle_to_servo_gain = rospy.get_param('/vesc/steering_angle_to_servo_gain')
		self.max_servo = rospy.get_param('/vesc/vesc_driver/servo_max')
		self.min_servo = rospy.get_param('/vesc/vesc_driver/servo_min')

		self.max_jerk = rospy.get_param('/vesc/max_jerk')
		self.torque_constant = rospy.get_param('/vesc/torque_constant')
		self.max_current = rospy.get_param('/vesc/vesc_driver/current_max')
		self.min_current = rospy.get_param('/vesc/vesc_driver/current_min')

		# Variables
		self.vehicle_mass = rospy.get_param('/asimcar/mass')
		self.wheel_radius = rospy.get_param('/asimcar/wheel_radius')
		self.gear_ratio = rospy.get_param('/asimcar/gear_ratio')
		self.torque_constant = rospy.get_param('/vesc/torque_constant')
		
		self.last_rpm = rospy.get_param('/vesc/erpm_offset')
		self.desired_rpm = self.last_rpm

		self.last_servo = rospy.get_param('/vesc/servo_offset')
		self.desired_servo_position = self.last_servo
		
		self.last_current = rospy.get_param('/vesc/current_offset')
		self.desired_current = self.last_current

		# Create topic subscribers and publishers
		self.rpm_output = rospy.Publisher(self.rpm_output_topic, Float64,queue_size=1)
		self.servo_output = rospy.Publisher(self.servo_output_topic, Float64,queue_size=1)
		self.current_output = rospy.Publisher(self.current_output_topic, Float64,queue_size=1)

		rospy.Subscriber(self.rpm_input_topic, Float64, self._process_throttle_command)
		rospy.Subscriber(self.servo_input_topic, Float64, self._process_servo_command)
		rospy.Subscriber(self.current_input_topic, Float64, self._process_current_command)

		self.max_delta_servo = abs(self.steering_angle_to_servo_gain * self.max_steering_speed / self.smooth_rate)
		rospy.Timer(rospy.Duration(1.0/self.smooth_rate), self._publish_servo_command)

		self.max_delta_rpm = abs(self.speed_to_erpm_gain * self.max_acceleration / self.smooth_rate)
		rospy.Timer(rospy.Duration(1.0/self.smooth_rate), self._publish_rpm_command)

		self.max_delta_current = abs(self.max_jerk * self.vehicle_mass * self.wheel_radius * 4 / self.gear_ratio / self.torque_constant / self.smooth_rate)
		rospy.Timer(rospy.Duration(1.0/self.smooth_rate), self._publish_current_command)

		# run the node
		self._run()

    # Keep the node alive
	def _run(self):
		rospy.spin()

	def _publish_rpm_command(self, evt):
		desired_delta = self.desired_rpm-self.last_rpm
		clipped_delta = max(min(desired_delta, self.max_delta_rpm), -self.max_delta_rpm)
		smoothed_rpm = self.last_rpm + clipped_delta
		self.last_rpm = smoothed_rpm         
		# print self.desired_rpm, smoothed_rpm
		self.rpm_output.publish(Float64(smoothed_rpm))
            
	def _process_rpm_command(self,msg):
		input_rpm = msg.data
		# Do some sanity clipping
		input_rpm = min(max(input_rpm, self.min_rpm), self.max_rpm)
		self.desired_rpm = input_rpm

	def _publish_servo_command(self, evt):
		desired_delta = self.desired_servo_position-self.last_servo
		clipped_delta = max(min(desired_delta, self.max_delta_servo), -self.max_delta_servo)
		smoothed_servo = self.last_servo + clipped_delta
		self.last_servo = smoothed_servo         
		self.servo_output.publish(Float64(smoothed_servo))

	def _process_servo_command(self,msg):
		input_servo = msg.data
		# Do some sanity clipping
		input_servo = min(max(input_servo, self.min_servo), self.max_servo)
		# set the target servo position
		self.desired_servo_position = input_servo

	def _publish_current_command(self, evt):
		desired_delta = self.desired_current-self.last_current
		clipped_delta = max(min(desired_delta, self.max_delta_current), -self.max_delta_current)
		smoothed_current = self.last_current + clipped_delta
		self.last_current = smoothed_current         
		self.current_output.publish(Float64(smoothed_current))

	def _process_current_command(self,msg):
		input_current = msg.data
		# Do some sanity clipping
		input_current = min(max(input_current, self.min_current), self.max_current)
		# set the target servo position
		self.desired_current = input_current

# Boilerplate node spin up. 
if __name__ == '__main__':
	try:
		rospy.init_node('command_interpreter')
		p = InterpolateThrottle()
	except rospy.ROSInterruptException:
		pass
