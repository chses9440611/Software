#!/usr/bin/env python
from duckietown_utils.jpg import image_cv_from_jpg
from sensor_msgs.msg import Image, CompressedImage, Joy
from duckietown_msgs.msg import  Twist2DStamped, BoolStamped, WheelsCmdStamped
from guidedog_msgs.msg import ImageCarcmdSync
import cv2
import numpy as np
import rospy
import threading
import time
from cv_bridge import CvBridge

class GdCropImageCarcmd(object):
	def __init__(self):
		self.node_name = rospy.get_name()

		self.pub_image = rospy.Publisher("~img", CompressedImage, queue_size = 1)
		self.sub_cmd = rospy.Subscriber("~wheels_cmd", WheelsCmdStamped, self.cbWheelsCmd, queue_size = 1)
		self.sub_image = rospy.Subscriber("~sync_data", ImageCarcmdSync, self.cbSync, queue_size=300)
		

		self.wheels_msg = WheelsCmdStamped()
		self.img_counnts = 0
		self.bridge = CvBridge()
		self.sync_stamp = []
		self.wheels_cmd_stamp = []

		self.path = '/home/tony/Desktop/graduate/guide_dog/testing-data/Joystick_45_1/'
		self.text_file = open(self.path + "carcmd.txt", "w")
		self.text_file.write("car_c, car_omega \n")
		self.text_file_wheel = open(self.path + "wheel_cmd.txt", "w")			
		self.text_file_wheel.write("vel_left, vel_right \n")			

	def cbSync(self, sync_msg):
		carcmd = Twist2DStamped()
		carcmd = sync_msg.carcmd
		v = carcmd.v
		omega = carcmd.omega
		
		self.text_file.write("%s,%s\n" %( v, omega))
		rospy.loginfo('v = %s, omega = %s' %(v, omega ) )
		
		# match the header.stamp
		index = self.contain(self.wheels_cmd_stamp, carcmd.header.stamp, lambda x: x == carcmd.header.stamp )
		if index != -1:
			#del self.wheels_cmd_stamp[index]
			self.text_file_wheel.write("%s,%s\n" %(self.wheels_msg.vel_left, self.wheels_msg.vel_right))
			rospy.loginfo('vel_left = %s, vel_right = %s' %(self.wheels_msg.vel_left, self.wheels_msg.vel_right ) )
		else:
			self.sync_stamp.append(carcmd.header.stamp)

		img_cv = image_cv_from_jpg(sync_msg.image.data)
		img_path = self.path + 'image/' +  str(self.img_counnts) + '.jpg'
		cv2.imwrite(img_path, img_cv)
		self.img_counnts += 1;

	def cbWheelsCmd(self, wheel_cmd_msg):
		self.wheels_msg = wheel_cmd_msg

		# match the header.stamp
		index = self.contain(self.sync_stamp, self.wheels_msg.header.stamp, lambda x: x == self.wheels_msg.header.stamp )
		if index != -1:
			#del self.sync_stamp[index]
			self.text_file_wheel.write("%s,%s\n" %(self.wheels_msg.vel_left, self.wheels_msg.vel_right))
			rospy.loginfo('vel_left = %s, vel_right = %s' %(self.wheels_msg.vel_left, self.wheels_msg.vel_right ) )
		else:
			self.wheels_cmd_stamp.append(self.wheels_msg.header.stamp)

	def contain(self, stamp_list, stamp, filter):
		for tmp in stamp_list:
			if filter(tmp):
				return stamp_list.index(tmp) 
		return -1

if __name__ == '__main__':
    rospy.init_node('gd_crop_image_carcmd',anonymous=False)
    crop_data = GdCropImageCarcmd()
    rospy.spin()
