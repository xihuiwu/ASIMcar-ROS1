#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu

f = open("acceleration_data.txt", "w")

def callback(data):
    f.write(repr(data.linear_acceleration) + "\n")
    f.write(repr(data.angular_velocity) + "\n")
    
def recorder():
    rospy.init_node('recorder', anonymous=True)
    rospy.Subscriber("/imu/data_raw", Imu, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        recorder()
    except rospy.ROSInterruptException:
        f.close()
