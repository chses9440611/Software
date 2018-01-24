#!/usr/bin/env python
import rospy
import os
import fnmatch
from duckietown_msgs.msg import Twist2DStamped
from sensor_msgs.msg import CompressedImage
import sys
sys.path.insert(0, '/home/tony/caffe/python')
import caffe
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
		self.setModel()

		self.sub_image = rospy.Subscriber("~compressed", CompressedImage, self.cbImage, queue_size=1)


	def setModel(self):

		listOfFiles = os.listdir(self.model_dir)  
		rospy.loginfo("[%s] list Of Files = [%s]" %(self.node_name, listOfFiles))

		for entry in listOfFiles:  
			if fnmatch.fnmatch(entry, "*.caffemodel"):
				self.model_weights = self.model_dir + entry
			elif fnmatch.fnmatch(entry, "*.prototxt"):
				self.model_def = self.model_dir + entry
			else:
				continue
		if self.model_weights!=None and self.model_def != None:
			self.check_getModel = True
			rospy.loginfo("[%s] caffe model = [%s]" %(self.node_name, self.model_weights))
			rospy.loginfo("[%s] caffe prototxt = [%s]" %(self.node_name, self.model_def))
			

		else:
			rospy.loginfo("[%s] Didn't find the caffe model file" %(self.node_name))

	def cbImage(self, msg):
		print ("test")
		np_arr = np.fromstring(msg.data, np.uint8)
		img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

		img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)	
		img = cv2.resize(img, self.dim)
		img = img.astype(np.float32)
		
		img_m = np.zeros((self.dim[0], self.dim[1], 1), np.float32)
		img_m[:] = (128.0)
		img = cv2.subtract(img, img_m)	
		img = img * 0.0078125


	def onShutdown(self):
		rospy.loginfo("[%s] Closing camera." %(self.node_name))
		self.is_shutdown=True
		rospy.loginfo("[%s] Shutdown." %(self.node_name))

if __name__ == '__main__': 
	rospy.init_node('caffe_prediction',anonymous=False)
	caffe_prediction_node = CaffePrediction()
	rospy.on_shutdown(caffe_prediction_node.onShutdown)
	rospy.spin()


