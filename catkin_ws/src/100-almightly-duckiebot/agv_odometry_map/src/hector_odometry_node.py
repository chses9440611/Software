#!/usr/bin/env python
import rospy
import time
import tf
import math
from math import sin, cos, pi
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose, Twist, Vector3, Quaternion, Point
class State:
	def __init__(self):
		self.trans  = None
		self.rot = None
		self.time = None

class HectorOdometryNode(object):

	def __init__(self): 
		self.node_name = rospy.get_name()
		rospy.loginfo("[%s] Initializing " %(self.node_name))
		
		self.listener = tf.TransformListener()

		self.target_frame = "scanmatcher_frame"
		self.source_frame = "map"
		self.odom_frame = "odom_frame"

		self.wheel_state_record = State()
		self.wheel_state = State()
		
		self.broadcaster_odom = tf.TransformBroadcaster()
		self.pub_odom = rospy.Publisher("~laserscan_odometry", Odometry, queue_size=10)
		
		self.sub_laser = rospy.Subscriber("/scan", LaserScan, self.cbLaserScan, queue_size=10)
		

	def drawOdometry(self):
		#Draw hector slam odometry
		odom = Odometry()
		odom.header.stamp = self.wheel_state.time
		odom.header.frame_id = self.odom_frame
		odom.child_frame_id = "base_link_2Dlidar"

		odom.pose.pose.position = Point(self.wheel_state.trans[0], self.wheel_state.trans[1], self.wheel_state.trans[2])
		odom.pose.pose.orientation = Quaternion(self.wheel_state.rot[0], self.wheel_state.rot[1], self.wheel_state.rot[2], self.wheel_state.rot[3])

		#print type(self.wheel_state.trans) 
		#print type(self.wheel_state.rot) 

		dt = self.wheel_state.time.secs - self.wheel_state_record.time.secs
		vx = (self.wheel_state.trans[0] - self.wheel_state_record.trans[0]) * dt
		vy = (self.wheel_state.trans[1] - self.wheel_state_record.trans[1]) * dt
		euler = tf.transformations.euler_from_quaternion(self.wheel_state.rot)
		euler_record = tf.transformations.euler_from_quaternion(self.wheel_state_record.rot)
		vth = (euler[2] - euler_record[2]) * dt

		odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))
		self.pub_odom.publish(odom)

	def cbLaserScan(self, msg):
		# create listener 
		self.wheel_state_record = self.wheel_state

		try:
			dur = rospy.Duration(0.5)
			self.listener.waitForTransform(self.source_frame, self.target_frame, msg.header.stamp, dur)
			(self.wheel_state.trans,self.wheel_state.rot) = self.listener.lookupTransform(self.source_frame, self.target_frame, msg.header.stamp)
			self.wheel_state.time = msg.header.stamp
			if self.wheel_state.trans is not None and self.wheel_state_record.trans is not None:
				self.drawOdometry()

		except :
			rospy.loginfo("[%s] LookupTransform '%s' to '%s' timed out." %(self.node_name, self.source_frame, self.target_frame))


	def onShutdown(self):
		rospy.loginfo('[%s] Closing Control Node.' %(self.node_name))
		self.is_shutdown=True
		rospy.loginfo("[%s] Shutdown." %(self.node_name))

if __name__ == '__main__':
	rospy.init_node('hector_odometry_node', anonymous=False)
	hector_odometry_node = HectorOdometryNode()

	rospy.on_shutdown(hector_odometry_node.onShutdown)
	rospy.spin()