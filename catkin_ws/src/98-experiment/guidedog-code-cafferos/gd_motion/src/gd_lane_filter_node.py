#!/usr/bin/env python
import rospy
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from duckietown_msgs.msg import Pixel, LanePose, BoolStamped, Twist2DStamped
from scipy.stats import multivariate_normal, entropy
from scipy.ndimage.filters import gaussian_filter
from math import floor, atan2, pi, cos, sin, sqrt
import time
from guidedog_msgs.msg import CaffePrediction, CaffePredictions



class GdLaneFilterNode(object):
    """
    
Guidedog Lane Filter Node

Author: Daniel Huang

Inputs: CaffePredictions

Outputs: LanePose


    """
    def __init__(self):
        self.node_name = rospy.get_name()
        self.active = True
        self.updateParams(None)
        
        self.d,self.phi = np.mgrid[self.d_min:self.d_max:self.delta_d,self.phi_min:self.phi_max:self.delta_phi]
        self.beliefRV=np.empty(self.d.shape)
        self.initializeBelief()
        self.lanePose = LanePose()
        self.lanePose.d=self.mean_0[0]
        self.lanePose.phi=self.mean_0[1]

        self.t_last_update = rospy.get_time()
        self.v_current = 0
        self.w_current = 0
        self.v_last = 0
        self.w_last = 0
        self.v_avg = 0
        self.w_avg = 0
        self.label2d = np.array([-0.2, -0.2, 0, 0, 0, 0.2, 0.2])
        self.label2phi = np.array([0, pi/6, -pi/6, 0, pi/6, -pi/6, 0])
        # Subscribers
        if self.use_propagation:
        	self.sub_velocity = rospy.Subscriber("~car_cmd", Twist2DStamped, self.updateVelocity)
        self.sub = rospy.Subscriber("~caffe_predictions", CaffePredictions, self.processPredictions, queue_size=1)
            #self.sub_velocity = rospy.Subscriber("~/gd_text_car_cmd_node/car_cmd", Twist2DStamped, self.updateVelocity)
        #self.sub = rospy.Subscriber("/gc/gd_L2R_text_text_node/caffe_predictions", CaffePredictions, self.processPredictions, queue_size=1)

        # Publishers
        self.pub_lane_pose  = rospy.Publisher("~lane_pose", LanePose, queue_size=1)
        self.pub_belief_img = rospy.Publisher("~belief_img", Image, queue_size=1)
        self.pub_entropy    = rospy.Publisher("~entropy",Float32, queue_size=1)
    	#self.pub_prop_img = rospy.Publisher("~prop_img", Image, queue_size=1)
        self.pub_in_lane    = rospy.Publisher("~in_lane",BoolStamped, queue_size=1)
        self.sub_switch = rospy.Subscriber("~switch", BoolStamped, self.cbSwitch, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration.from_sec(1.0), self.updateParams)


    def updateParams(self, event):
        self.mean_0 = [rospy.get_param("~mean_d_0",0) , rospy.get_param("~mean_phi_0",0)]
        self.cov_0  = [ [rospy.get_param("~sigma_d_0",0.1) , 0] , [0, rospy.get_param("~sigma_phi_0",0.01)] ]
        self.delta_d     = rospy.get_param("~delta_d",0.02) # in meters
        self.delta_phi   = rospy.get_param("~delta_phi",0.02) # in radians
        self.d_max       = rospy.get_param("~d_max",0.5)
        self.d_min       = rospy.get_param("~d_min",-0.7)
        self.phi_min     = rospy.get_param("~phi_min",-pi/2)
        self.phi_max     = rospy.get_param("~phi_max",pi/2)

        self.cov_v       = rospy.get_param("~cov_v",0.5) # linear velocity "input"
        self.cov_omega   = rospy.get_param("~cov_omega",0.01) # angular velocity "input"
        self.linewidth_white = rospy.get_param("~linewidth_white",0.04)
        self.linewidth_yellow = rospy.get_param("~linewidth_yellow",0.02)
        self.lanewidth        = rospy.get_param("~lanewidth",0.4)
        self.min_max = rospy.get_param("~min_max", 0.3) # nats


        # For propagation
        self.use_propagation = rospy.get_param("~use_propagation",True)
        self.cov_mask = [rospy.get_param("~sigma_d_mask",0.1) , rospy.get_param("~sigma_phi_mask",0.1)]

    def cbSwitch(self, switch_msg):
        self.active = switch_msg.data

    def processPredictions(self,CaffePredictions):
        if not self.active:
            return
        t_start = rospy.get_time()

        if self.use_propagation:
            self.propagateBelief()
            self.t_last_update = rospy.get_time()
	datum = CaffePredictions.predictions
        data = datum[0].probs
        print data
	if len(data)!=7:
            print 'wrong data len', len(data)
    	
	# initialize measurement likelihood
        measurement_likelihood = np.zeros(self.d.shape)    	
	#calculate i,j index
   	for index in range(len(data)):
            i = floor((self.label2d[index] - self.d_min)/self.delta_d)
            j = floor((self.label2phi[index] - self.phi_min)/self.delta_phi)
            measurement_likelihood[i,j] = measurement_likelihood[i,j] +  data[index]
        self.updateBelief(measurement_likelihood)
        maxids = np.unravel_index(self.beliefRV.argmax(),self.beliefRV.shape)
        # rospy.loginfo('maxids: %s' % maxids)
        self.lanePose.header.stamp = CaffePredictions.header.stamp
        self.lanePose.d = self.d_min + maxids[0]*self.delta_d
        self.lanePose.phi = self.phi_min + maxids[1]*self.delta_phi
        self.lanePose.status = self.lanePose.NORMAL
        print 'lane pose: d', self.d_min + maxids[0]*self.delta_d, 'phi', self.phi_min + maxids[1]*self.delta_phi

        if np.linalg.norm(measurement_likelihood) == 0:
            return
        measurement_likelihood = measurement_likelihood/np.sum(measurement_likelihood)

        if self.use_propagation:
            self.updateBelief(measurement_likelihood)
        else:
            self.beliefRV = measurement_likelihood

        # TODO entropy test:
        #print self.beliefRV.argmax()

        maxids = np.unravel_index(self.beliefRV.argmax(),self.beliefRV.shape)


        # publish the belief image
        bridge = CvBridge()
        belief_img = bridge.cv2_to_imgmsg((255*self.beliefRV).astype('uint8'), "mono8")
        belief_img.header.stamp = CaffePredictions.header.stamp
        
        max_val = self.beliefRV.max()
        
        self.pub_lane_pose.publish(self.lanePose)
        self.pub_belief_img.publish(belief_img)

     

        # Publish in_lane according to the ent
        in_lane_msg = BoolStamped()
        in_lane_msg.header.stamp = CaffePredictions.header.stamp
        in_lane_msg.data = self.lanePose.in_lane
        # ent = entropy(self.beliefRV)
        # if (ent < self.max_entropy):
        #     in_lane_msg.data = True
        # else:
        #     in_lane_msg.data = False
        self.pub_in_lane.publish(in_lane_msg)




    def updateVelocity(self,twist_msg):
        self.v_current = twist_msg.v
        self.w_current = twist_msg.omega
        
        #self.v_avg = (self.v_current + self.v_last)/2.0
        #self.w_avg = (self.w_current + self.w_last)/2.0

        #self.v_last = v_current
        #self.w_last = w_current

    def initializeBelief(self):
        pos = np.empty(self.d.shape + (2,))
        pos[:,:,0]=self.d
        pos[:,:,1]=self.phi
        self.cov_0
        RV = multivariate_normal(self.mean_0,self.cov_0)
        self.beliefRV=RV.pdf(pos)

    def propagateBelief(self):
        delta_t = rospy.get_time() - self.t_last_update

        d_t = self.d + self.v_current*delta_t*np.sin(self.phi)
        phi_t = self.phi + self.w_current*delta_t

        p_beliefRV = np.zeros(self.beliefRV.shape)

        for i in range(self.beliefRV.shape[0]):
            for j in range(self.beliefRV.shape[1]):
                if self.beliefRV[i,j] > 0:
                    if d_t[i,j] > self.d_max or d_t[i,j] < self.d_min or phi_t[i,j] < self.phi_min or phi_t[i,j] > self.phi_max:
                        continue
                    i_new = floor((d_t[i,j] - self.d_min)/self.delta_d)
                    j_new = floor((phi_t[i,j] - self.phi_min)/self.delta_phi)
                    p_beliefRV[i_new,j_new] += self.beliefRV[i,j]

        s_beliefRV = np.zeros(self.beliefRV.shape)
        gaussian_filter(100*p_beliefRV, self.cov_mask, output=s_beliefRV, mode='constant')

        if np.sum(s_beliefRV) == 0:
            return
        self.beliefRV = s_beliefRV/np.sum(s_beliefRV)

    	#bridge = CvBridge()
        #prop_img = bridge.cv2_to_imgmsg((255*self.beliefRV).astype('uint8'), "mono8")
        #self.pub_prop_img.publish(prop_img)
                
        return

    def updateBelief(self,measurement_likelihood):
        self.beliefRV=np.multiply(self.beliefRV+1,measurement_likelihood+1)-1
        self.beliefRV=self.beliefRV/np.sum(self.beliefRV)#np.linalg.norm(self.beliefRV)



    def onShutdown(self):
        rospy.loginfo("[GdLaneFilterNode] Shutdown.")


if __name__ == '__main__':
    rospy.init_node('gd_lane_filter_node',anonymous=False)
    gd_lane_filter_node = GdLaneFilterNode()
    rospy.on_shutdown(gd_lane_filter_node.onShutdown)
    rospy.spin()
