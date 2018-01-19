#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from duckietown_msgs.msg import WheelsCmdStamped
from guidedog_msgs.msg import CaffePredictions

class GdTextWheelCmdNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing " %(self.node_name))
        
        self.max_speed = 1
        self.min_speed = 0.2

        # publishers
        self.pub_wheel_cmd = rospy.Publisher("~wheels_cmd",WheelsCmdStamped, queue_size=1)

        # subscribers
        self.sub_caffe_predictions = rospy.Subscriber("~caffe_predictions", CaffePredictions, self.cbCaffePredictions, queue_size=1)
        self.sub_gain = rospy.Subscriber("~set_gain", Float32, self.cbSetMaxSpeed, queue_size=1)

    def cbCaffePredictions(self,msg):
    	vel_left_sum, vel_right_sum = self.cal_vel(msg)
        wheel_cmd_msg = WheelsCmdStamped()
        wheel_cmd_msg.header.stamp = rospy.Time.now()
        wheel_cmd_msg.vel_left = vel_left_sum
        wheel_cmd_msg.vel_right = vel_right_sum
        self.pub_wheel_cmd.publish(wheel_cmd_msg)

    def cal_vel(self,msg):
        vel_left = 0
        vel_right = 0
        speed = 0
        l_prob = 0.001
        s_prob = 0.001
        r_prob = 0.001

        for i in range(len(msg.classes)):
            if msg.classes[i] == "S":
                s_prob = msg.probs[i]
            if msg.classes[i] == "R":
                r_prob = msg.probs[i]
            if msg.classes[i] == "L":
                l_prob = msg.probs[i]

        if msg.classes[0] == "S":
            speed = self.max_speed * (1 + s_prob) / 2
            #if r_prob != 0 and l_prob != 0:
            vel_left = (1-(r_prob*2)) * speed
            vel_right = (1-(l_prob*2)) * speed
            #else:
            #    vel_left = r_prob*speed
            #    vel_right = l_prob*speed
        else:
            speed = (r_prob+l_prob) * 0.5 * self.max_speed
            vel_left = l_prob * speed * (self.max_speed-self.min_speed) + self.min_speed
            vel_right = r_prob * speed * (self.max_speed-self.min_speed) + self.min_speed
        print speed, vel_left, vel_right
        return vel_left, vel_right


    def cal_vel_org(self,msg):
        vel_left = 0
        vel_right = 0
        for i in range(len(msg.classes)):
            if msg.classes[i] == "R":
                vel_right += self.tf_prob2vel(msg.probs[i])
            if msg.classes[i]  == "L":
                vel_left += self.tf_prob2vel(msg.probs[i]) 
        return vel_left, vel_right

    def tf_prob2vel(self,prob):
        prob_left = 0.2
        prob_right = 0.8
        if(prob <= prob_left):
            o = prob/1.2;
        elif(prob >= prob_right):
            o = 1.8+prob/(3-1.8);
        else:
            o = 1.2+prob/(1.8-1.2);
        return o;

    def cbSetMaxSpeed(self,speed):
    	self.max_speed = speed

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutting down."%(rospy.get_name()))

if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('gd_text_wheels_cmd_node', anonymous=False)
    # Create the DaguCar object
    node = GdTextWheelCmdNode()
    # Setup proper shutdown behavior 
    rospy.on_shutdown(node.on_shutdown)
    # Keep it spinning to keep the node alive
    rospy.spin()
