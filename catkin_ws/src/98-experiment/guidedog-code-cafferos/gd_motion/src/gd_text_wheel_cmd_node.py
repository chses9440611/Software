#!/usr/bin/env python
import rospy
from duckietown_msgs.msg import WheelsCmdStamped
from guidedog_msgs.msg import CaffePredictions

class GdTextWheelCmdNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing " %(self.node_name))
        
        # publishers
        self.pub_wheel_cmd = rospy.Publisher("~wheels_cmd",WheelsCmdStamped, queue_size=1)

        # subscribers
        self.sub_caffe_predictions = rospy.Subscriber("~caffe_predictions", CaffePredictions, self.cbCaffePredictions, queue_size=1)

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
        for i in range(len(msg.classes)):
            if msg.classes[i] == "R":
                vel_right += self.tf_prob2vel(msg.probs[i])
            if msg.classes[i]  == "L":
                vel_left += self.tf_prob2vel(msg.probs[i]) 
        return vel_left, vel_right

    def tf_prob2vel(self,prob):
#        prob_left = 0.2
#        prob_right = 0.8
#        if(prob <= prob_left):
#            o = prob/1.2;
#        elif(prob >= prob_right):
#            o = 1.8+prob/(3-1.8);
#        else:
#            o = 1.2+prob/(1.8-1.2);
        return o;

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
