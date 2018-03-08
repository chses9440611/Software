#!/usr/bin/env python

from duckietown_msgs.msg import Twist2DStamped
import numpy as np
import rospy
import threading
import time
import math
import sys

class Test(object):
	def __init__(self):
		self.node_name = rospy.get_name()
		rospy.loginfo("[%s] Initializing" %(self.node_name))
		self.sub_joy_cmd = rospy.Subscriber("~joy_cmd", Twist2DStamped, self.cb_test, queue_size=1)
		self.pub_final_cmd = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1)

	def cb_test(self, msg):
		car_cmd_msg = Twist2DStamped()
		car_cmd_msg.source = 1 # 1 for joy_stick
		car_cmd_msg.v = msg.v
		car_cmd_msg.omega = msg.omega
		self.pub_final_cmd.publish(car_cmd_msg)

	def onShutdown(self):
		rospy.loginfo('[%s] Closing Control Node.' %(self.node_name))
		self.is_shutdown = True
		rospy.loginfo('[%s] Shutdown.' %(self.node_name))


if __name__ == "__main__":
	rospy.init_node('test', anonymous=False)
	test = Test()
	rospy.onshutdown(test.onShutdown)
	rospy.spin()
