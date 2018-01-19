#!/usr/bin/env python
from duckietown_utils.jpg import image_cv_from_jpg
from sensor_msgs.msg import Image, CompressedImage, Joy
from duckietown_msgs.msg import  Twist2DStamped, BoolStamped
from guidedog_msgs.msg import Imagecarcmdsync
import cv2
import numpy as np
import rospy
import threading
import time

class GdImageCarcmdSync(object):
	def __init__(self):
		self.node_name = self.node_name = rospy.get_name()

		self.sub_image = rospy.Subscriber("~image", CompressedImage, self.cbImage, queue_size=300)
		self.sub_cmd = rospy.Subscriber("~carcmd", Twist2DStamped, self.cbCmd, queue_size=300)

		self.pub_sync = rospy.Publisher("~sync_cmd", ImageCarcmdSync, queue_size=1)

		self.sync_msg = ImageCarcmdSync();

	def cbImage(self, img_msg):

		self.sync_msg.header.stamp = img_msg.header.stamp
		self.sync_msg.image = img_msg

	def cbCmd(self, cmd_msg):

		carcmd = Twist2DStamped();
		carcmd.v = cmd_msg.v
		carcmd.omega = cmd_msg.omega
		carcmd.header.stamp = cmd_msg.header.stamp

		self.sync_msg.carcmd = carcmd
		self.pub_sync.publish(self.sync_msg)

if __name__ == '__main__':
    rospy.init_node('gd_image_carcmd_sync',anonymous=False)
    image_carcmd_sync = GdImageCarcmdSync()
    rospy.spin()
