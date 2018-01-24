#!/usr/bin/env python
import rospy
import os
import fnmatch
from duckietown_msgs.msg import Twist2DStamped
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
import numpy as np
import sys
import cv2
import heapq
import copy
import time
sys.path.insert(0, '/home/tony/caffe/python')
import caffe
caffe.set_device(0)
caffe.set_mode_gpu()

class CaffePrediction(object):

	def __init__(self):
		self.node_name = rospy.get_name()
		rospy.loginfo("[%s] Initializing" %(self.node_name))

		self.model_dir = rospy.get_param('~model_dir')
		self.model_weights = None #caffe model
		self.model_def = None #prototxt 
		self.check_getModel = False
		self.net = None
		self.transformer = None
		self.dim = None
		self.getModel()
		
		self.counts = 0

		self.sub_image = rospy.Subscriber("~compressed", CompressedImage, self.cbImage, queue_size=1)
		self.pub_pred = rospy.Publisher("~prediction", String, queue_size = 1)

	def getModel(self):

		listOfFiles = os.listdir(self.model_dir)  
		rospy.loginfo("[%s] list Of Files = [%s]" %(self.node_name, listOfFiles))

		for entry in listOfFiles:  
			if fnmatch.fnmatch(entry, "*.caffemodel"):
				self.model_weights = self.model_dir + entry
			elif fnmatch.fnmatch(entry, "*.prototxt"):
				self.model_def = self.model_dir + entry
			elif fnmatch.fnmatch(entry, "*.txt"):
				self.label = np.loadtxt(self.model_dir + entry, str, delimiter='\n')
			else:
				continue

		if self.model_weights is not None and self.model_def is not  None:
			self.check_getModel = True
			rospy.loginfo("[%s] caffe model = %s" %(self.node_name, self.model_weights))
			rospy.loginfo("[%s] caffe prototxt = %s" %(self.node_name, self.model_def))
			rospy.loginfo("[%s] caffe label = %s" %(self.node_name, self.label))
			self.getDim()

			self.net = caffe.Net(self.model_def, self.model_weights, caffe.TEST)

			self.transformer = caffe.io.Transformer({'data': self.net.blobs['data'].data.shape})
			self.transformer.set_transpose('data', (2,0,1)) 

		else:
			rospy.loginfo("[%s] Didn't find the caffe model file" %(self.node_name))

	def getDim(self):
		self.dim = []
		with open(str(self.model_def), "r") as file:
			for line in file:
				if "dim" in line:
					line_split = line.split(" ")
					i = 0;
					while i < len(line_split):
						if "dim" in line_split[i]:
							i += 1
							self.dim.append(int(line_split[i]))
						i += 1
		rospy.loginfo("[%s] dimension = %s" %(self.node_name, self.dim))

	def cbImage(self, msg):

		self.counts += 1
		if self.check_getModel is True and self.counts == 1:
			self.counts = -1
			time = rospy.Time.now()
			tmp = time - msg.header.stamp
			print (tmp.to_sec())

			np_arr = np.fromstring(msg.data, np.uint8)
			img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

			img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)	
			img = cv2.resize(img, (self.dim[2], self.dim[3]))
			img = img.astype(np.float32)
			
			img_m = np.zeros((self.dim[2], self.dim[3], 1), np.float32)
			img_m[:] = (128.0)
			img = cv2.subtract(img, img_m)	
			img = img * 0.0078125

			self.net.blobs['data'].data[...] = img
			output = copy.deepcopy(self.net.forward())
			output_prob = output['prob'][0]
			largest_class = heapq.nlargest(3, xrange(len(output_prob)), key=output_prob.__getitem__)
			largest_prob = heapq.nlargest(3, output_prob)
			image_prob = output_prob

			pred_result =""
			for i in range(0, 3):
				pred_result += str(self.label[largest_class[i]]) + " = " + str(largest_prob[i]) + " , "
			self.pub_pred.publish(pred_result)



	def onShutdown(self):
		rospy.loginfo("[%s] Closing camera." %(self.node_name))
		self.is_shutdown=True
		rospy.loginfo("[%s] Shutdown." %(self.node_name))

if __name__ == '__main__': 
	rospy.init_node('caffe_prediction',anonymous=False)
	caffe_prediction_node = CaffePrediction()
	rospy.on_shutdown(caffe_prediction_node.onShutdown)
	rospy.spin()


