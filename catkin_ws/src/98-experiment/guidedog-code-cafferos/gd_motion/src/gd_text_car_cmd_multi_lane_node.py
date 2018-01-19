#!/usr/bin/env python
import rospy
import math
from duckietown_msgs.msg import Twist2DStamped
from guidedog_msgs.msg import CaffePredictions, CaffePrediction
from std_msgs.msg import String, Bool

class GdTextCarCmdMultiLaneNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing " %(self.node_name))
 
        # motion weight for YB and FT only
        self.lane_mode = 'YB'
        self.classes_label = ['YB_L', 'YB_S', 'YB_R', 'FT_L', 'FT_S', 'FT_R']
        self.omega_weight_YB = [-1.5, 0, 1.5, 0, 0, 0]
        self.omega_weight_FT = [0, 0, 0, -1.2, 0, 1.2]
        self.setParams()
        if(self.lane_mode == 'FT'):
            self.omega_weight = self.omega_weight_FT
            print 'switch to FT trail following'
        else:
            self.omega_weight = self.omega_weight_YB
            print 'switch to YB trail following'

        # PD controller not used
        self.v = 0
        self.omega = 0
        self.kp = 0.7
        
        # publishers
        self.pub_car_cmd = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1)
        self.pub_image_switch = rospy.Publisher("~image_switch", Bool, queue_size=1)

        # subscribers
        self.sub_caffe_predictions = rospy.Subscriber("~caffe_predictions", CaffePredictions, self.cbCaffePredictions, queue_size=1)
        self.sub_lane_mode = rospy.Subscriber("~lane_mode", String, self.cbLaneMode, queue_size=1)
    
    def setParams(self):
        # set lane mode
        self.lane_mode  = rospy.get_param('~lane_mode')

    def cbLaneMode(self,msg):
        if(msg.data == 'YB'):
            self.omega_weight = self.omega_weight_YB
            print 'switch to YB trail following'
        if(msg.data == 'FT'):
            self.omega_weight = self.omega_weight_FT   		
            print 'switch to FT trail following'

    def cbCaffePredictions(self,msg):
        v = 0
        omega = 0
        self.v = 0
        self.omega = 0

        for i in range(msg.batch_size):
            # transfer prediction to velocity and omega
            v = self.cal_vel_v1(msg.predictions[i])
            omega = self.cal_omega_v1(msg.predictions[i])
            # visualization
            #print "v : ", v, " omega : ", omega
            # work #
            #for j in range(msg.predictions[i].output_number):
            #    print msg.predictions[i].classes[j], msg.predictions[i].probs[j]
            #    # accumulate velocity and omega
            #    self.v = self.v + v
            #    self.omega = self.omega + omega
            # work
            self.v += v
            self.omega += omega
        self.publish_car_cmd( (self.v/msg.batch_size), (self.omega/msg.batch_size), 0)
        image_switch_msg = Bool()
        image_switch_msg.data = True
        self.pub_image_switch.publish(image_switch_msg)
    
    def cal_vel_v1(self,msg):
        v = 0.38
        return v

    def cal_omega_v1(self, msg):
        omega = 0
        for i in range(len(msg.classes)):
            for j in range(len(self.classes_label)):
                if(msg.classes[i] == self.classes_label[j]):
                    omega += self.omega_weight[j] * self.tf_prob2vel(msg.probs[i])
        return omega   

    def tf_prob2vel(self,prob):
        o = 1/(1+math.exp(-prob)) 
        o = o * 9
        return o;

    def publish_car_cmd(self, v, omega, time):
        car_cmd_msg = Twist2DStamped()
        car_cmd_msg.header.stamp = rospy.Time.now()
        car_cmd_msg.v = v
        car_cmd_msg.omega = omega
        self.pub_car_cmd.publish(car_cmd_msg)
        rospy.sleep(time)

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutting down."%(rospy.get_name()))

if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('gd_text_car_cmd_multi_lane_node', anonymous=False)
    # Create the DaguCar object
    node = GdTextCarCmdMultiLaneNode()
    # Setup proper shutdown behavior 
    rospy.on_shutdown(node.on_shutdown)
    # Keep it spinning to keep the node alive
    rospy.spin()
