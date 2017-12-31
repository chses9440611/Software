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
import time
import os
from duckietown_msgs.msg import Twist2DStamped, WheelsCmdStamped
from duckietown_msgs.srv import SetValue, SetValueRequest, SetValueResponse
from std_srvs.srv import EmptyRequest, EmptyResponse, Empty
from pulse import Pulse
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
		self.node_name = rospy.get_name()
		rospy.loginfo("[%s] Initializing " %(self.node_name))

		self.config_path = rospy.get_param("~config_path")
		self.veh_name = rospy.get_param("~veh_name")
		
		self.ps = Pulse([5 ,6, 13, 19])
		self.fname = None
		self.v = 0
		self.omega = 0
		self.readParamFromFile()
		self.shutdown = False

		self.kRadius = self.setup_parameter('~kRadius', 8.5)
		self.kEncRes = self.setup_parameter('~kEncRes', 1024)
		self.kSmpTime = self.setup_parameter('~kSmpTime', 10)
		self.kMaxVel = self.setup_parameter('~kMaxVel', 40)
		self.updatekMaxPPMS()
		
		#Publisher
		self.pub_wheelcmd = rospy.Publisher("~wheel_cmd", WheelsCmdStamped, queue_size=1)

		#open new thread
		rospy.Timer(rospy.Duration(0.2), self.threadSetSpeed)
		time.sleep(0.2)

		self.srv_kRadius = rospy.Service("~set_kRadius", SetValue, self.cbSrvSetkRadius)
		self.srv_kEncRes = rospy.Service("~set_kEncRes", SetValue, self.cbSrvSetkEncRes)
		self.srv_kSmpTime = rospy.Service('~set_kSmpTime', SetValue, self.cbSrvSetkSmpTime)
		self.srv_kMaxVel = rospy.Service('~set_kMaxVel', SetValue, self.cbSrvSetkMaxVel)
		self.srv_save_param = rospy.Service('~save_param', Empty, self.cbSrvSaveParam)

		#Subsriber
		self.sub_carcmd = rospy.Subscriber("~car_cmd", Twist2DStamped, self.cbCarcmd, queue_size=1)

	def threadSetSpeed(self, event):

		#rospy.loginfo("[%s] Set speed " %(self.node_name))
		left = int(-(self.v+self.omega)*self.kMaxPPMS)
		right = int((self.v-self.omega)*self.kMaxPPMS)
		self.ps.set_speed([left, right])
		
		wheelcmd = WheelsCmdStamped()

		wheelcmd.header.stamp = rospy.Time.now()
		wheelcmd.vel_left = -left
		wheelcmd.vel_right = right
		self.pub_wheelcmd.publish(wheelcmd)

	def cbCarcmd(self, msg):
		#rospy.loginfo("velocity: [%f] omega: [%f]" %(msg.v, msg.omega))
		self.v = msg.v
		self.omega = msg.omega

	def readParamFromFile(self):
		#check the file 
		fname_ = self.config_path + self.veh_name + ".yaml"
		self.fname = fname_
		if not os.path.isfile(fname_):
			rospy.logwarn("[%s] %s does not exist. Using default.yaml." %(self.node_name,fname_))
			fname_ = self.config_path + "default.yaml"

		with open(fname_, 'r') as in_file:
			try:
				yaml_ = yaml.load(in_file)
			except yaml.YAMLError as exc:
				rospy.logfatal("[%s] YAML syntax error. File: %s fname. Exc: %s" %(self.node_name, fname_, exc))
				rospy.signal_shutdown()
				return

		if yaml_ is None:
			 return

		for param_name in ["kRadius", "kEncRes", "kSmpTime", "kMaxVel"]:
			value = yaml_.get(param_name)
			if param_name is not None:
				rospy.set_param("~"+param_name, value)


	def cbSrvSaveParam(self, req):
		data={
			"kRadius": self.kRadius,
			"kEncRes": self.kEncRes,
			"kSmpTime": self.kSmpTime,
			"kMaxVel": self.kMaxVel,
		}
		fname_ = self.fname
		with open(fname_, "w") as out_file:
			out_file.write(yaml.dump(data, default_flow_style=False))
		self.printValues()
		rospy.loginfo("[%s] Saved to %s" %(self.node_name, fname_))
		return EmptyResponse()

	def cbSrvSetkRadius(self, req):
		self.kRadius = req.value
		self.updatekMaxPPMS()
		return SetValueResponse()
	
	def cbSrvSetkEncRes(self, req):
		self.kEncRes = req.value
		self.updatekMaxPPMS()
		return SetValueResponse()
	
	def cbSrvSetkSmpTime(self, req):
		self.kSmpTime = req.value
		self.updatekMaxPPMS()
		self.ps.updatekSmpTime(self.kSmpTime)
		return SetValueResponse()

	def cbSrvSetkMaxVel(self, req):
		self.kMaxVel = req.value
		self.updatekMaxPPMS()
		return SetValueResponse()

	def updatekMaxPPMS(self):
		self.kMaxPPMS = self.kMaxVel*self.kSmpTime/36.0 / (2*math.pi*self.kRadius) * self.kEncRes
		self.ps.updatekMaxPPMS(self.kMaxPPMS)
		self.printValues()

	def printValues(self):
		rospy.loginfo("[%s] kRadius: %s kEncRes: %s kSmpTime: %s kMaxVel: %s kMaxPPMS: %s" % (self.node_name, self.kRadius, self.kEncRes, self.kSmpTime, self.kMaxVel, self.kMaxPPMS))

	def setup_parameter(self, param_name, default_value):
		value = rospy.get_param(param_name, default_value)
		rospy.set_param(param_name, value)
		rospy.loginfo("[%s] %s = %s" %(self.node_name, param_name, value))
		return value

	def onShutdown(self):
		self.shutdown = True
		rospy.loginfo('[%s] Closing Control Node.' %(self.node_name))
		self.is_shutdown = True
		rospy.loginfo("[%s] Shutdown." %(self.node_name))


if __name__ == "__main__":
	rospy.init_node("agv_wheel_driver", anonymous=False, disable_signals=True)

	wheel_driver = AgvWheelDriverNode()
	rospy.on_shutdown(wheel_driver.onShutdown)
	rospy.spin()


