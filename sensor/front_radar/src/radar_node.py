#!/usr/bin/env python
import rospy
from lidar_lite import Lidar_Lite
from std_msgs.msg import Float32

rospy.loginfo("Initialize front radar...")
lidar = Lidar_Lite(1)
rospy.loginfo("Done Initialization!")

def radar():
	pub = rospy.Publisher('/front_radar', Float32, queue_size=1)
	rospy.init_node('radar_node', anonymous=True)
	rate = rospy.Rate(100)
	while not rospy.is_shutdown():
		dist_cm = lidar.getDistance()
		pub.publish(dist_cm)
		rate.sleep()

if __name__ == '__main__':
	try:
		radar()
	except rospy.ROSInterruptException:
		pass
