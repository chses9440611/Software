#!/usr/bin/env python
import rospy
import numpy as np
import math
from duckietown_msgs.msg import  Twist2DStamped, LanePose
from geometry_msgs.msg import Twist

class lane_controller(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        self.turn = 0.7
        self.gain = 0.5

        # Publicaiton
        self.pub_car_twist = rospy.Publisher("/cmd_vel",Twist,queue_size=1)
        # Subscriptions
        self.sub_lane_reading = rospy.Subscriber("~lane_pose", LanePose, self.cbPose, queue_size=1)

        # safe shutdown
        rospy.on_shutdown(self.custom_shutdown)

        # timer
        rospy.loginfo("[%s] Initialized " %(rospy.get_name()))

    def custom_shutdown(self):
        car_twist_msg = Twist()
        car_twist_msg.linear.x = 0
        car_twist_msg.angular.z = 0
        self.pub_car_twist.publish(car_twist_msg)
        rospy.sleep(0.5)

    def cbPose(self,lane_pose_msg):

        turn = self.turn * lane_pose_msg.d *10
        #rospy.loginfo(lane_pose_msg.d)
        car_twist_msg = Twist()
        car_twist_msg.linear.x = self.gain
        car_twist_msg.angular.z = turn
        self.pub_car_twist.publish(car_twist_msg)


if __name__ == "__main__":
    rospy.init_node("lane_controller",anonymous=False)
    lane_control_node = lane_controller()
    rospy.spin()
