#!/usr/bin/python

import rospy
import numpy as np
from sensor_msgs.msg import FluidPressure
from geometry_msgs.msg import PoseWithCovarianceStamped
from sam_msgs.msg import ThrusterAngles
from uavcan_ros_bridge.msg import ESCStatus
from nav_msgs.msg import Odometry

class SamDR(object):

	def __init__(self):
		
		# self.esc_fb_topic = rospy.get_param(rospy.get_name() + '/esc_status', '/pressure')
		self.dr_thrust_topic = rospy.get_param(rospy.get_name() + '/thrust_dr', '/pressure')
		self.thrust_vec_fb_topic = rospy.get_param(rospy.get_name() + '/thrust_fb', '/thrust_fb')

		self.subs_thrust = rospy.Subscriber(self.thrust_vec_fb_topic, ESCStatus, self.thrustAngCB)
		# self.subs_esc= rospy.Subscriber(self.esc_fb_topic, ESCStatus, self.escStatCB)
		
 		self.odom_pub = rospy.Publisher(self.dr_thrust_topic, Odometry)

 		self.time_now = 0
 		self.prev_time = 0
 		self.x = 0
 		self.y = 0
 		self.P = 0.001

		rospy.spin()

	def thrustAngCB(self, thrust_msg):

		self.time_now = rospy.Time.now() 

		dt = (self.time_now - self.prev_time).to_sec()

		msg_odom = Odometry()
		msg_odom.header.stamp = self.time_now
		msg_odom.header.frame_id = "sam_odom"
		msg_odom.child_frame_id = "sam/base_link"

		self.x =+ (self.P * thrust_msg.rpm)*dt
		msg_odom.pose.pose.position.x = self.x
		msg_odom.pose.pose.position.y = self.y
		msg_odom.pose.covariance = [1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01]
		msg_odom.twist.twist.linear.x = self.P * thrust_msg.rpm
 		msg_odom.twist.covariance = [1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01]

		self.odom_pub.publish(msg_odom)	

		self.prev_time = self.time_now	




if __name__ == "__main__":

	rospy.init_node('sam_dr')
	try:
		SamDR()
	except rospy.ROSInterruptException:
		pass