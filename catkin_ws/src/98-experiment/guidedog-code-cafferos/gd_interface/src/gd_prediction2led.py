#!/usr/bin/env python
import rospy
import numpy as np
import math
import time
from std_msgs.msg import Int8
from guidedog_msgs.msg import CaffePredictions, CaffePrediction, ClassLeds
#import Adafruit_PCA9685


class GdPrediction2Led(object):
    def __init__(self):
        self.node_name = rospy.get_name()

        # load classes labels
        self.classes = rospy.get_param('~classes')
        self.class3 = rospy.get_param('~class3')
        self.class5 = rospy.get_param('~class5')
        self.class6 = rospy.get_param('~class6')
        self.class66 = rospy.get_param('~class66')
        self.class7 = rospy.get_param('~class7')
        self.class72 = rospy.get_param('~class72')
        self.class8 = rospy.get_param('~class8')
        self.class9 = rospy.get_param('~class9')
        
        # set GPIO pin
        self.class3_pin = rospy.get_param('~class3_pin')
        self.class5_pin = rospy.get_param('~class5_pin')
        self.class6_pin = rospy.get_param('~class6_pin')
        self.class7_pin = rospy.get_param('~class7_pin')
        self.class8_pin = rospy.get_param('~class8_pin')
        self.class9_pin = rospy.get_param('~class9_pin')
        
        self.class3_pin_ws = rospy.get_param('~class3_pin_ws')
        self.class5_pin_ws = rospy.get_param('~class5_pin_ws')
        self.class6_pin_ws = rospy.get_param('~class6_pin_ws')
        self.class66_pin_ws = rospy.get_param('~class66_pin_ws')
        self.class7_pin_ws = rospy.get_param('~class7_pin_ws')
        self.class8_pin_ws = rospy.get_param('~class8_pin_ws')
        self.class9_pin_ws = rospy.get_param('~class9_pin_ws')
        
        # set PWM level
        self.led_high = rospy.get_param('~led_high')
        self.led_mid = rospy.get_param('~led_mid')
        self.led_low = rospy.get_param('~led_low')
        
        # set prob threshold
        self.thres_high = rospy.get_param('~thres_high')
        self.thres_low = rospy.get_param('~thres_low')
        # set specific classes
        self.set_classes()

        self.led_classes = [0, 0, 0, 0, 0, 0, 0]      

        # subscribers
        self.sub_caffe_predictions = rospy.Subscriber("~caffe_predictions", CaffePredictions, self.cbPredict2LED, queue_size=1)
        # publishers
        self.pub_led_classes = rospy.Publisher("~led_classes", ClassLeds, queue_size=1)
        
        #self.pwm = Adafruit_PCA9685.PCA9685()
        #self.pwm.set_pwm_freq(60)

        #initial the pwm to 0 
        #self.initialize()

    def set_classes(self):
        if(self.classes == 3):
            self.class_list = self.class3
            self.class_pin = self.class3_pin
            self.class_pin_ws = self.class3_pin_ws
        elif(self.classes == 5):
            self.class_list = self.class5           
            self.class_pin = self.class5_pin 
            self.class_pin_ws = self.class5_pin_ws
        elif(self.classes == 6):
            self.class_list = self.class6           
            self.class_pin = self.class6_pin
            self.class_pin_ws = self.class6_pin_ws
        elif(self.classes == 66):
            self.class_list = self.class66           
            self.class_pin_ws = self.class66_pin_ws
        elif(self.classes == 7):
            self.class_list = self.class7           
            self.class_pin = self.class7_pin
            self.class_pin_ws = self.class7_pin_ws
        elif(self.classes == 72):
            self.class_list = self.class72           
            self.class_pin = self.class7_pin
            self.class_pin_ws = self.class7_pin_ws
        elif(self.classes == 8):
            self.class_list = self.class8           
            self.class_pin = self.class8_pin
            self.class_pin_ws = self.class8_pin_ws
        elif(self.classes == 9):
            self.class_list = self.class9 
            self.class_pin = self.class9_pin
            self.class_pin_ws = self.class9_pin_ws

    def initialize(self):
        for i in range(16):
            self.pwm.set_pwm(i, 0 , 0)

    def cbPredict2LED(self, msg):
        for caffe_pred in msg.predictions:
            # if top 1 prediction equal to N_L, all led red
            if(caffe_pred.classes[0] == 'N_L'):
                for i in range(len(self.class_pin_ws)):
                    #self.pwm.set_pwm(self.class_pin[i] + 1, 0 , self.led_high)
                    self.led_classes[self.class_pin_ws[i]] = 7
                led_classes_msg = ClassLeds()
                led_classes_msg.classes_led = self.led_classes
                print self.led_classes
                led_classes_msg.header.stamp = rospy.Time.now()
                self.pub_led_classes.publish(led_classes_msg)
                continue
            # n
            for i in range(caffe_pred.output_number):
                for j in range(len(self.class_list)):
                    if(caffe_pred.classes[i] == self.class_list[j]):
                        if(caffe_pred.classes[i] != 'N_L'):
                            self.sent_pwm(caffe_pred.probs[i], j)
            
            led_classes_msg = ClassLeds()
            led_classes_msg.classes_led = self.led_classes
            print self.led_classes
            led_classes_msg.header.stamp = rospy.Time.now()
            self.pub_led_classes.publish(led_classes_msg)

    def sent_pwm(self, prob, pin_index):
        # green: probability >= thres_high
        if(prob >= self.thres_high):
            #self.pwm.set_pwm(self.class_pin[pin_index] + 1, 0 , self.led_high)
            #self.pwm.set_pwm(self.class_pin[pin_index] , 0 , 0)
            self.led_classes[self.class_pin_ws[pin_index]] = 4
        # red: thres_high > probability >= thres_low
        elif(prob >= self.thres_low):
            #self.pwm.set_pwm(self.class_pin[pin_index] + 1, 0 , 0)
            #self.pwm.set_pwm(self.class_pin[pin_index] , 0 , self.led_high)
            self.led_classes[self.class_pin_ws[pin_index]] = 6
        # no light: thres_low > probability
        else:
            #self.pwm.set_pwm(self.class_pin[pin_index] + 1, 0 , 0)
            #self.pwm.set_pwm(self.class_pin[pin_index], 0 , 0)
            self.led_classes[self.class_pin_ws[pin_index]] = 0
        
if __name__ == "__main__":
    #set your node_name
    rospy.init_node("gd_prediction2led",anonymous=False)
    gdprediction2led = GdPrediction2Led()
    rospy.spin()
