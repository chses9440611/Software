#!/usr/bin/env python
import rospy
import copy
import cv2
import numpy as np
import time
import sys
import rospkg
from duckietown_msgs.msg import  Twist2DStamped, BoolStamped
from duckietown_msgs.srv import  SetValue
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64

class ControlNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing......" %(self.node_name))
        self.joy = None
        self.duration = 0.2
        self.duration_step = 0.05
        self.gain = 1
        self.gain_step = 0.05

        # Publications
        self.pub_car_cmd = rospy.Publisher("~car_cmd", Twist2DStamped,queue_size=1)
        self.pub_duration = rospy.Publisher("gd_R2L_picamera_node/duration", Float64, queue_size=1)
        self.pub_gain = rospy.ServiceProxy("inverse_kinematics_node/set_gain", SetValue)

        # Subscriptions
        self.sub_joy_ = rospy.Subscriber("joy", Joy, self.cbJoy, queue_size=1)
        self.sub_prediction_car_cmd = rospy.Subscriber("~prediction_car_cmd", Twist2DStamped, self.cbprediction, queue_size=1)
        self.sub_joy_car_cmd = rospy.Subscriber("~joy_car_cmd", Twist2DStamped, self.cbjoy_car_cmd, queue_size=1)
        self.sub_switch = rospy.Subscriber("~joystick_override", BoolStamped, self.processButtons, queue_size=1)
        self.switch = "JoyStick"  
    
    def cbJoy(self, joy_msg):
        if(joy_msg.buttons[0] == 1):
            if(self.switch == "RL_Control"):
                print '2'
                self.robot_like_motion(-0.1, 0, 0.1)
                self.robot_like_motion(0, 0, 0.1)
            else:
                self.gain = self.gain - self.gain_step
                print "down gain to ", self.gain
                self.pub_gain(self.gain)
        if(joy_msg.buttons[3] == 1):
            if(self.switch == "RL_Control"):
                self.robot_like_motion(0.1, 0, 0.1)
                self.robot_like_motion(0, 0, 0.1)
            else:
                self.gain = self.gain + self.gain_step
                print "up gain to ", self.gain
                self.pub_gain(self.gain)
        if(joy_msg.buttons[1] == 1):
            if(self.switch == "RL_Control"):
                self.robot_like_motion(0, -1, 0.1)
                self.robot_like_motion(0, 0, 0.1)
            else:
                self.duration = self.duration - self.duration_step
                print "down duration to ", self.duration
                duration_msg = Float64()
                duration_msg.data = self.duration
                self.pub_duration.publish(duration_msg)
        if(joy_msg.buttons[2] == 1):
            if(self.switch == "RL_Control"):
                self.robot_like_motion(0, 1, 0.1)
                self.robot_like_motion(0, 0, 0.1)
            else:
                self.duration = self.duration + self.duration_step
                print "up duration to ", self.duration
                duration_msg = Float64()
                duration_msg.data = self.duration
                self.pub_duration.publish(duration_msg)

    def cbprediction(self, prediction_msg):
        if self.switch == "Prediction":
            self.pub_car_cmd.publish(prediction_msg)

    def cbjoy_car_cmd(self, joy_car_cmd_msg):
        if self.switch == "JoyStick":
            self.pub_car_cmd.publish(joy_car_cmd_msg)

    def processButtons(self, switch_msg):
        #print switch_msg.data
        if (switch_msg.data == False) :
            if (self.switch == "JoyStick") :
                self.switch = "Prediction"
                print "Prediction Control"
            elif(self.switch == "Prediction"):
                self.switch = "RL_Control"
                print "Robot-Like Control"
            elif(self.switch == "RL_Control"):
                self.switch = "JoyStick"
                print "Joystick Control"

    def robot_like_motion(self, v, omega, time):
        car_cmd_msg = Twist2DStamped()
        car_cmd_msg.v = v
        car_cmd_msg.omega = omega
        car_cmd_msg.header.stamp = rospy.Time.now()
        self.pub_car_cmd.publish(car_cmd_msg)
        rospy.sleep(time)

    def onShutdown(self):
        rospy.loginfo("[%s] Closing Control Node." %(self.node_name))
        self.is_shutdown=True
        rospy.loginfo("[%s] Shutdown." %(self.node_name))

if __name__ == '__main__': 
    rospy.init_node('control',anonymous=False)
    control_node = ControlNode()
    rospy.on_shutdown(control_node.onShutdown)
    rospy.spin()
