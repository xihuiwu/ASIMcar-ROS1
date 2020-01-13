#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDriveStamped

class JoyTeleop:
    def __init__(self):
        # Allocate topics
        self.new_msg = AckermannDriveStamped()
        self.teleop_input_topic = rospy.get_param('~teleop_input_topic', '/joy')
        self.teleop_output_topic = rospy.get_param('~teleop_output_topic', '/vesc/ackermann_cmd')
        self.output = rospy.Publisher(self.teleop_output_topic, AckermannDriveStamped, queue_size=10)
        rospy.Subscriber(self.teleop_input_topic, Joy, self._publish_joy_command)
        
        # Define variables
        self.steering_axis = rospy.get_param('~steering_axis', 3)
        self.speed_axis = rospy.get_param('~speed_axis', 1)
        self._run()

    def _run(self):
        rospy.spin()
    
    def _publish_joy_command(self,msg):
        self.new_msg.header.stamp = rospy.Time.now()
        self.new_msg.drive.steering_angle = msg.axes[self.steering_axis]
        self.new_msg.drive.speed = msg.axes[self.speed_axis]
        self.new_msg.drive.steering_angle_velocity = 0.0
        self.new_msg.drive.acceleration = 0.0
        self.new_msg.drive.jerk = 0.0
        self.output.publish(self.new_msg)

if __name__ == '__main__':
    try:
        rospy.init_node('joy_teleop')
        JoyTeleop()
    except rospy.ROSInterruptException:
        pass
