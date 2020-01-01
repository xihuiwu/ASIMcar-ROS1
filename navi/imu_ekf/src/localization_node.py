#!/usr/bin/env python
import numpy as np
import math
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose2D
from bicycle_msgs.msg import BicycleStateStamped
from vesc_msgs.msg import VescStateStamped

class ekf_localization:
	def __init__(self):
		# initialize variable for usage
		float self.steering, self.acce, time, pre_time, beta

		gps_flag = rospy.get_param("gps_flag")
		wheelbase = rospy.get_param("/vesc/wheelbase")
		lf = rospy.get_param("/vesc/lf")
		lr = rospy.get_param("/vesclr")
		procCov = rospy.get_param("procCov")
		stateCov = rospy.get_param("stateCov")
		acceCov = rospy.get_param("acceCov")
		angularCov = rospy.get_param("angularCov")

		self.gps_sub = rospy.Subscriber("gps", Pose2D, self.gpsCallback)
		self.acce_sub = rospy.Subscriber("/vesc/sensors/core", VescStateStamped, self.acceCallback)
		self.steering_sub = rospy.Subscriber("/vesc/sensors/servo_position_command", Float64, self.acceCallback)
		self.imu_sub = rospy.Subscriber("/imu/data", Imu, self.imuCallback)
		self.pose_pub = rospy.Publisher("/asimcar/state", BicycleStateStamped, queue_size=1)

		gps = np.zeros(shape=(2,1))
		state = np.zeros(shape=(4,1))
		state_tilde = np.zeros(shape=(4,1))

		P = stateCov*np.eye(4)
		Q = procCov*np.eye(4)
		S = np.zeros(shape=(4,4))
		K = np.zeros(shape=(4,4))

		if gps_flag == 1:
			state[0] = gps[0]
			state[1] = gps[1]
			gpsCov = rospy.get_param("gpsCov")
			R = np.diag([gpsCov[0], gpsCov[1], angularCov[2], acceCov[0]])
			H = np.diag([1, 1, 1, 1])
		else:
			R = np.diag([0, 0, angularCov[2], acceCov[0]])
			H = np.diag([0, 0, 1, 1])

		state_msg = BicycleStateStamped()

		pre_time = rospy.Time.now()

		self.run()

	def run(self):
		rospy.spin()

	def gpsCallback(self, gps_msg):
		gps[0] = gps_msg.x
		gps[1] = gps_msg.y

	def acceCallback(self, vesc_msg):
		self.acce = vesc_msg.state.speed

	def steeringCallback(self, steering_msg):
		self.steering = steering_msg.data

	def imuCallback(self, imu_msg):
		time = rospy.Time.now()
		dt = time - pre_time
		pre_time = time

		# prediction
		state[0] = state[0] + state[3]*math.cos(state[2])*dt
		state[1] = state[1] + state[3]*math.sin(state[2])*dt
		beta = math.atan(lr/wheelbase*math.tan(self.steering))
		state[2] = state[2] + state[3]/lr*math.sin(beta)
		state[3] = state[3] + self.acce*dt

		F = [[1, 0, state[3]*math.sin(state[2])*dt, cos(state[2])*dt],
		     [0, 1, state[3]*math.cos(state[2])*dt, sin(state[2])*dt],
		     [0, 0, 1, math.tan(self.steering)/wheelbase*dt],
		     [0, 0, 0, 1]]
		P = F*P*np.transpose(F) + Q

		# correction
		if gps_flag == 1:
			state_tilde = [gps[0]-state[0], gps[1]-state[1], imu_msg.angular_velocity*dt, imu_msg.linear_acceleration*dt]
		else:
			state_tilde = [0, 0, imu_msg.angular_velocity*dt, imu_msg.linear_acceleration*dt]
		S = H*P*np.transpose(H) + R
		K = P*H*np.transpose(S)
		state = state + K*state_tilde
		P = (np.eye(4) - K*H)*P

		state_msg.header.stamp = time
		state_msg.x = state[0]
		state_msg.y = state[1]
		state_msg.theta = state[2]
		state_msg.v = state[3]

		self.pose_pub.publish(state_msg)

if __name__ == "__main__":
	rospy.init_node('ekf_localization', anonymous=True)
	try:
		localization = ekf_localization()
	except rospy.ROSInterruptException:
		pass


