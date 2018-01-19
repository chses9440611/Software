#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import Bool
from duckietown_msgs.msg import Twist2DStamped
from guidedog_msgs.msg import CaffePredictions, CaffePrediction

class GdTextCarCmdNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing " %(self.node_name))
        # set parameters
        self.setParams()
        # PD controller not used
        self.v = 0
        self.omega = 0
        self.kp = 0.7
        # 7-class lateral distance and view angle base
        self.l_d_base = [-0.2, -0.2, 0, 0, 0, 0.2, 0.2]
        self.v_a_base = [0, 30, -30, 0, 30, -30, 0]
        # publishers
        self.pub_car_cmd = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1)
        self.pub_image_switch = rospy.Publisher("~image_switch", Bool, queue_size=1)

        # subscribers
        self.sub_caffe_predictions = rospy.Subscriber("~caffe_predictions", CaffePredictions, self.cbCaffePredictions, queue_size=1)

    def setParams(self):
        # get classes number
        self.motion_mode = rospy.get_param('~motion_mode')
        # get classes number
        self.classes = rospy.get_param('~classes')
        # get prediction classes index
        self.classes_index = rospy.get_param('~classes_index')
        # get prediction classes
        self.classes_label = rospy.get_param('~classes_label')
        # get weight of omega
        self.omega_weight = rospy.get_param('~omega_weight')
        # robot like motion table [v, omega, time]
        self.l = rospy.get_param('~l')
        self.s = rospy.get_param('~s')
        self.r = rospy.get_param('~r')
        # robot like motion map
        self.robot_like_motion_map = rospy.get_param('~robot_like_motion_map') 
        # set prediction classes index number
        for index, class_number in enumerate(self.classes_index):
            if(self.classes == class_number):
                self.classes_index_choose = index    

    def cbCaffePredictions(self,msg):
        v = 0
        omega = 0
        self.v = 0
        self.omega = 0
        l_d = 0
        v_a = 0

        if(self.motion_mode == 0):
            for i in range(msg.batch_size):
                # transfer prediction to velocity and omega
                v = self.cal_vel_v1(msg.predictions[i])
                omega = self.cal_omega_v1(msg.predictions[i], self.classes_index_choose)
                # 7-class lane pose estimation
                if(self.classes == 7):
                    l_d, v_a = self.cal_lane_pose(msg.predictions[i], self.classes_index_choose)
                    print "lateral distance: ", l_d, " view angle: ", v_a
                # visualization
                print "v : ", v, " omega : ", omega
                # work motion
                #for j in range(msg.predictions[i].output_number):
                #    print msg.predictions[i].classes[j], msg.predictions[i].probs[j]
                #    # accumulate velocity and omega
                #    self.v = self.v + v
                #    self.omega = self.omega + omega
                # work mootion
                self.v += v
                self.omega += omega
            self.publish_car_cmd( (self.v/msg.batch_size), (self.omega/msg.batch_size), 0)
        
        if(self.motion_mode == 1):
            self.robot_like_motion(msg.predictions[0], self.classes_index_choose)

        image_switch_msg = Bool()
        image_switch_msg.data = True
        self.pub_image_switch.publish(image_switch_msg)

    def cal_vel_v1(self, msg):
        v = 0.38
        return v

    def cal_omega_v1(self, msg, index):
        omega = 0
        for i in range(len(msg.classes)):
            for j in range(len(self.classes_label[index])):
                if(msg.classes[i] == self.classes_label[index][j]):
                    omega += self.omega_weight[index][j] * self.tf_prob2vel(msg.probs[i])
        return omega  

    def cal_lane_pose(self, msg, index):
        l_d = 0
        v_a = 0
        for i in range(len(msg.classes)):
            for j in range(len(self.classes_label[index])):
                if(msg.classes[i] == self.classes_label[index][j]):
                    l_d += self.l_d_base[j] * msg.probs[i]
                    v_a += self.v_a_base[j] * msg.probs[i]
        return l_d, v_a

    def tf_prob2vel(self,prob):
        prob_left = 0.2
        prob_right = 0.8
        o = 1/(1+math.exp(-prob)) 
        o = o * 9
        
        if(prob <= prob_left):
            o = prob/1.2;
        elif(prob >= prob_right):
            o = 1.8+prob/(3-1.8);
        else:
            o = 1.2+prob/(1.8-1.2);

        return o;

    def publish_car_cmd(self, v, omega, time):
        car_cmd_msg = Twist2DStamped()
        car_cmd_msg.header.stamp = rospy.Time.now()
        car_cmd_msg.v = v
        car_cmd_msg.omega = omega
        self.pub_car_cmd.publish(car_cmd_msg)
        rospy.sleep(time)

    def robot_like_motion(self, msg, index):
        for i in range(len(self.classes_label[index])):
            if(msg.classes[0] == self.classes_label[index][i]):
                print self.robot_like_motion_map[index][i], self.robot_like_motion_map[index][i]
                if(self.robot_like_motion_map[index][i] == 'l'):
                    self.publish_car_cmd(self.l[0], self.l[1], self.l[2])
                    print "v : ", self.l[0], " omega : ", self.l[1]
                if(self.robot_like_motion_map[index][i] == 's'):
                    self.publish_car_cmd(self.s[0], self.s[1], self.s[2])
                    print "v : ", self.s[0], " omega : ", self.s[1]
                if(self.robot_like_motion_map[index][i] == 'r'):
                    self.publish_car_cmd(self.r[0], self.r[1], self.r[2])
                    print "v : ", self.r[0], " omega : ", self.r[1]

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutting down."%(rospy.get_name()))

if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('gd_text_car_cmd_node', anonymous=False)
    # Create the DaguCar object
    node = GdTextCarCmdNode()
    # Setup proper shutdown behavior 
    rospy.on_shutdown(node.on_shutdown)
    # Keep it spinning to keep the node alive
    rospy.spin()
