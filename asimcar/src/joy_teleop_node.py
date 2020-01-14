#!/usr/bin/env python
import pygame
import rospy
from ackermann_msgs.msg import AckermannDriveStamped

pygame.init()

class JoyTeleop:
	def __init__(self):
		rospy.init_node('joy_teleop')
		self.ackermann_msg = AckermannDriveStamped()
		self.pub_topic = rospy.get_param('~teleop_output_topic', '/vesc/ackermann_cmd')
		self.pub = rospy.Publisher(self.pub_topic, AckermannDriveStamped, queue_size=1)
		
		self.port = rospy.get_param('/joy/port', 0)
		self.deadzone = rospy.get_param('/joy/deadzone', 0.05)
		self.frequency = rospy.get_param('/joy/frequency', 80.0)
		self.rate = rospy.Rate(self.frequency)

		self.steering_axis = rospy.get_param('/joy/steering_axis', 3)
		self.speed_axis = rospy.get_param('/joy/speed_axis', 2)
		self.acceleration_axis = rospy.get_param('/joy/acceleration_axis', 2)
		self.switch_mode = rospy.get_param('/joy/switch_mode', 0)
		self.switch_direction = rospy.get_param('/joy/switch_direction', 4)
		self.mode = False # false for speed control, true for acceleration control
		self.direction = 1 # 1 is forward, -1 is backward

		self.speed_scale = rospy.get_param('/joy/speed_scale', 1)
		self.acceleration_scale = rospy.get_param('/joy/acceleration_scale', 3)

		self.stick = pygame.joystick.Joystick(self.port)
		self.stick.init()

		self._run()
        
	def _run(self):
		while not rospy.is_shutdown():
			for event in pygame.event.get():
				if event.type == pygame.JOYBUTTONDOWN:
					if self.stick.get_button(self.switch_mode):
						self.mode = not self.mode
						if self.mode:
							rospy.loginfo("Switch to Acceleration Control")
						else:
							rospy.loginfo("Switch to Speed Control")
							
					if self.stick.get_button(self.switch_direction):
						self.direction = self.direction*-1
						rospy.loginfo("Switch direction to %d", self.direction)
						
				if event.type == pygame.JOYAXISMOTION:
					self.ackermann_msg.header.stamp = rospy.Time.now()
					if self.mode:
						self.ackermann_msg.drive.acceleration = (self._deadzone_trigger(self.acceleration_axis) + 1)/2*self.acceleration_scale*self.direction
					else:
						self.ackermann_msg.drive.speed = (self._deadzone_trigger(self.speed_axis) + 1)/2*self.speed_scale*self.direction
					self.ackermann_msg.drive.steering_angle = self._deadzone_stick(self.steering_axis)
					
			self.pub.publish(self.ackermann_msg)
			self.rate.sleep()
    	
		self.stick.quit()
    	
	def _deadzone_stick(self, axis):
		if abs(self.stick.get_axis(axis)) < self.deadzone:
			return 0.0
		else:
			return self.stick.get_axis(axis)
			
	def _deadzone_trigger(self, axis):
		if abs(self.stick.get_axis(axis) + 1) < self.deadzone:
			return -1.0
		else:
			return self.stick.get_axis(axis)
    
if __name__ == '__main__':
	try:
		teleop = JoyTeleop()
	except rospy.ROSInterruptException:
		pass
