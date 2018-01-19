#!/usr/bin/env python
from duckietown_utils.jpg import image_cv_from_jpg
from sensor_msgs.msg import Image, CompressedImage, Joy
from duckietown_msgs.msg import  Twist2DStamped, BoolStamped, LanePose, WheelsCmdStamped
import cv2
import numpy as np
import rospy
import threading
import time
from cv_bridge import CvBridge

class GdCropLaneFollowingCarcmd(object):
	def __init__(self):
		self.node_name = rospy.get_name()

		self.sub_image = rospy.Subscriber("~image", CompressedImage, self.cbImage, queue_size=1)
		self.sub_lanepose = rospy.Subscriber("~lane_pose", LanePose, self.cbPose, queue_size=1)
		self.sub_car_cmd = rospy.Subscriber("~car_cmd", Twist2DStamped, self.cbCarcmd, queue_size=1)
		self.sub_wheels_cmd = rospy.Subscriber("~wheels_cmd", WheelsCmdStamped, self.cbWheelsCmd, queue_size=1)

		self.img_counnts = 0
		self.bridge = CvBridge()
		self.path = '/home/tony/Desktop/graduate/guide_dog/testing-data/nibot_laneFollow/'
		self.text_file = open(self.path + "lane_pose.txt", "w")
		self.text_file.write("stamp, lane_pose_d, lane_pose_phi \n")
		self.text_file_carcmd = open(self.path + "carcmd.txt", "w")
		self.text_file_carcmd.write("stamp, v, omega \n")
		self.text_file_wheelcmd = open(self.path + "wheel_cmd.txt", "w")
		self.text_file_wheelcmd.write("stamp, vel_left, vel_right \n")

		self.img_stamp= []
		self.img_cv = []

	def cbImage(self, img_msg):
		self.img_cv.append(image_cv_from_jpg(img_msg.data))
		self.img_stamp.append(img_msg.header.stamp)

	def contain(self, stamp_list, stamp, filter):
		for tmp in stamp_list:
			if filter(tmp):
				return stamp_list.index(tmp) 
		return -1

	def cbPose(self, pose_msg):
		lane_pose_stamp = pose_msg.header.stamp
		if self.img_cv != None:
			index = self.contain(self.img_stamp, lane_pose_stamp, lambda x: x == lane_pose_stamp )
			if index != -1:
				d = pose_msg.d
				phi = pose_msg.phi

				self.text_file.write("%s,%s,%s\n" %(lane_pose_stamp, d, phi))
				rospy.loginfo('stamp = %s, d = %s, phi = %s' %(self.img_stamp[index], d, phi ) )
				img_path = self.path + 'image/' +  str(self.img_counnts) + '.jpg'
				cv2.imwrite(img_path, self.img_cv[index])
				self.img_counnts += 1;
		
	def cbCarcmd(self, carcmd_msg):
		carcmd_stamp = carcmd_msg.header.stamp
		index = self.contain(self.img_stamp, carcmd_stamp, lambda x: x == carcmd_stamp )
		if index != -1:
			v = carcmd_msg.v
			omega = carcmd_msg.omega
			self.text_file_carcmd.write("%s,%s,%s\n" %(carcmd_stamp ,v, omega))
			rospy.loginfo('v = %s, omega = %s' %(v, omega ) )

	def cbWheelsCmd(self, wheelcmd_msgs):
		wheelscmd_stamp = wheelcmd_msgs.header.stamp
		index = self.contain(self.img_stamp, wheelscmd_stamp, lambda x: x == wheelscmd_stamp )
		if index != -1:
			left = wheelcmd_msgs.vel_left
			right = wheelcmd_msgs.vel_right
			self.text_file_wheelcmd.write("%s,%s,%s\n" %(wheelscmd_stamp, left, right))
			rospy.loginfo('vel_left = %s, vel_right = %s' %(left, right ) )


if __name__ == '__main__':
    rospy.init_node('gd_crop_lanefollowing_carcmd_sync',anonymous=False)
    gd_crop_lanefollowing_carcmd_sync = GdCropLaneFollowingCarcmd()
    rospy.spin()
