#!/usr/bin/env python
from sensor_msgs.msg import Joy
from duckietown_msgs.msg import  Twist2DStamped
import numpy as np
import rospy
import threading
import time
import math
import sys
class JoyMapperNode(object):
	def __init__(self):
		self.node_name = rospy.get_name()
		rospy.loginfo("[%s] Initializing " %(self.node_name))
		self.Joy = None
		self.v_gain = rospy.get_param("~v_gain", 0.3)
		self.omega_gain = rospy.get_param("~omega_gain", 0.5)

		# Publications
		self.pub_car_cmd = rospy.Publisher("~joy_cmd", Twist2DStamped, queue_size=1)
		
		# Subscriptions
		self.sub_joy = rospy.Subscriber("joy", Joy, self.cbJoy, queue_size=1)

	def cbJoy(self, msg):
		self.processStick(msg)

	def processStick(self, msg):
		car_cmd_msg = Twist2DStamped()
		car_cmd_msg.v = msg.axes[1] * self.v_gain
		car_cmd_msg.omega = msg.axes[3] * self.omega_gain
		self.pub_car_cmd.publish(car_cmd_msg)

	def onShutdown(self):
		rospy.loginfo('[%s] Closing Control Node.' %(self.node_name))
		self.is_shutdown=True
		rospy.loginfo("[%s] Shutdown." %(self.node_name))

if __name__ == "__main__":
	rospy.init_node('agv_joy_mapper', anonymous=False)
	agv_joy_mapper = JoyMapperNode()
	rospy.on_shutdown(agv_joy_mapper.onShutdown)
	rospy.spin()
