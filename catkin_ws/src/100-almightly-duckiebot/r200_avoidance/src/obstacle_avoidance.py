#!/usr/bin/env python
import numpy as np
import rospy
from duckietown_msgs.msg import Twist2DStamped, BoolStamped
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sklearn.cluster import DBSCAN 
from matplotlib import pyplot as plt
import time

class ObstacleAvoidance(object):

	def __init__(self):
		self.node_name = rospy.get_name()
		rospy.loginfo("[%s] Initializing " %(self.node_name))

		self.bridge = CvBridge()
		# Subscriber
		self.sub_image_color = rospy.Subscriber("~image_color", Image, self.cbImageColor, queue_size=1)
		self.sub_image_depth = rospy.Subscriber("~image_depth", Image, self.cbImageDepth, queue_size=1)

		#Publisher
		self.pub_image_depth = rospy.Publisher("~image_obstacle_depth", Image, queue_size=1)
		self.pub_image_color = rospy.Publisher("~image_obstacle_color", Image, queue_size=1)
		self.pub_state_stop = rospy.Publisher("~state", BoolStamped, queue_size=1)

		#avoid latency
		self.counts_img = 0
		self.img_color = None
		self.gate = 2
		self.state_stop = False
		self.distance_range = [800, 2000]
		self.obstacle_location_record = []

	def cbImageColor(self, msg):
		try:
			self.img_color = self.bridge.imgmsg_to_cv2(msg, "rgb8")
		except CvBridgeError as e:
			print(e)

	def obstacleStopDecision(self, cluster_points, img_width):
		if not len(cluster_points) is 0:
			cluster_avg = sum(cluster_points)/len(cluster_points)
			print cluster_avg
			if (cluster_avg >= img_width*0.2 ) and (cluster_avg <= img_width * 0.8):
				self.state_stop = True
				rospy.loginfo("[%s] The car need to stop" %(self.node_name))


	

	# failed => cause too many time
	def obstacleClustering(self, points, img):
		self.state_stop = False

		db = DBSCAN(eps=10, min_samples = 50, algorithm='kd_tree').fit(points)

		# every pixel about its cluster number
		labels = db.labels_ 
		# return only different value EX:[0,0,1]->[0,1]
		unique_labels = set(labels) 
		cluster_num = len(unique_labels)-1

		print('Estimated number of clusters: %d' % cluster_num) 
		
		# noise point = False, cluster =True
		sample_mask = np.zeros_like(labels, dtype=bool)
		sample_mask[db.core_sample_indices_] = True

		for k in unique_labels:
			if k == -1 : #noise
				color = (0, 0, 0)
			# find the member mask of cluster
			cluster_member_mask = (labels == k)

			# True and True => not noise and belong to this cluster
			cluster_points = points[cluster_member_mask & sample_mask]
			self.obstacleStopDecision(cluster_points, img.shape[1])
			for p in cluster_points:
				img[(p[0],p[1])] = (255, 0, 0)

			# noise points
			noise_points = points[cluster_member_mask & ~sample_mask]
			for p in noise_points:
				img[(p[0],p[1])] = (0, 255, 0)

		state = BoolStamped()
		state.data = self.state_stop
		self.pub_state_stop.publish(state)	


	def cbImageDepth(self, msg):
		self.counts_img += 1
		if self.counts_img >= self.gate and not self.img_color is None:
			#tStart = time.time()

			self.counts_img = 0
			try:
				img_depth = self.bridge.imgmsg_to_cv2(msg, "16UC1")
			except CvBridgeError as e:
				print(e)

			# resize color img from (640, 480) to (120, 160)
			img_color = cv2.resize(self.img_color,(160, 120), interpolation=cv2.INTER_CUBIC)
			img_depth = cv2.resize(img_depth,(160, 120), interpolation=cv2.INTER_CUBIC)
			(height, width) = img_depth.shape
			
			obstacle_points = []
			# R200 x and y are rotate 90 degree
			for x in range(height/3, height):
				for y in range(0, width):
					if img_depth[(x ,y)] > self.distance_range[0] and img_depth[(x, y)] < self.distance_range[1]:
						obstacle_points.append((x ,y))

			obstacle_points = np.asarray(obstacle_points)
			self.obstacleClustering(obstacle_points, img_color)	
		
			try :
				#print len(obstacle_points)
				#tEnd = time.time()
			 	#self.pub_image_depth.publish(self.bridge.cv2_to_imgmsg(img_depth, "16UC.1"))
			 	self.pub_image_color.publish(self.bridge.cv2_to_imgmsg(img_color, "rgb8"))
			 	#print tEnd - tStart
			except CvBridgeError as e:
				print(e)

	def onShutdown(self):
		rospy.loginfo('[%s] Closing Control Node.' %(self.node_name))
		self.is_shutdown=True
		rospy.loginfo("[%s] Shutdown." %(self.node_name))
		

if __name__ == '__main__':
	rospy.init_node('obsatcle_avoid', anonymous=False)
	obstacle_avoidance = ObstacleAvoidance()
	rospy.on_shutdown(obstacle_avoidance.onShutdown)
	rospy.spin()