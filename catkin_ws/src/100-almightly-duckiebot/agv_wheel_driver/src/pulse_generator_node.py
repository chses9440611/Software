#!/usr/bin/env python
# From   : NCTU & Motorcon inc. co-host program.
# Purpose: Generate pulse sequence by the corresponding input.
# Author : Hao-Wei Hsu / IanHsu@motorcontech.com

# APIs:
import sys
import time
import math
import numpy
import signal
import yaml
import threading
import RPi.GPIO as GPIO
import rospy
from geometry_msgs.msg import Twist

"""
Constant values:
 kRadius : Wheel's radius (cm). 
 kEncRes : Encoder resolution.
 kSmpTime: Sampling Time (ms).
 kMaxVel : Max cartesian velocity (km/h).
 kMaxPPMS: Max Pulse Per MilliSecond.
"""
class AgvWheelDriverNode(object):
	def __init__(self):
		self.node_name = self.get_name()
		rospy.loginfo("[%s] Initializing " %(self.node_name))

		self.kRadius = self.setup_parameter('~kRadius', 8.5)
		self.kEncRes = self.setup_parameter('~kEncRes', 1024)
		self.kSmpTime = self.setup_parameter('~kSmpTime', 10)
		self.kMaxVel = self.setup_parameter('~kMaxVel', 40)

		

		self.kMaxPPMS = kMaxVel*kSmpTime/36.0 / (2*math.pi*kRadius) * kEncRes



	def setup_parameter(self, param_name, default_value):
		value = rospy.get_param(param_name, default_value)
		rospy.set_param(param_name, value)
		rospy.loginfo("[%s] %s = %s" %(self.node_name, param_name, value))
		return value


	def onShutdown(self):
		rospy.loginfo('[%s] Closing Control Node.' %(self.node_name))
		self.is_shutdown=True
		rospy.loginfo("[%s] Shutdown." %(self.node_name))


if __name__ == 'main':
	rospy.init_node("agv_wheel_driver", anonymous=False)
	wheel_driver = AgvWheelDriverNode()
	rospy.on_shutdown(wheel_driver.onShutdown)
	rospy.spin()