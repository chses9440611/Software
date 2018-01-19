#!/usr/bin/env python
import rospy
import numpy as np
import math
import time
from guidedog_msgs.msg import ClassLeds
from std_msgs.msg import Int32


from neopixel import *

class GdStanderledNi(object):
    def __init__(self):
        self.node_name = rospy.get_name()

        # LED strip configuration:
        self.LED_COUNT      = 7      # Number of LED pixels.
        self.LED_PIN        = 18      # GPIO pin connected to the pixels (18 uses PWM!).
        #LED_PIN        = 10      # GPIO pin connected to the pixels (10 uses SPI /dev/spidev0.0).
        self.LED_FREQ_HZ    = 800000  # LED signal frequency in hertz (usually 800khz)
        self.LED_DMA        = 5       # DMA channel to use for generating signal (try 5)
        self.LED_BRIGHTNESS = 255     # Set to 0 for darkest and 255 for brightest
        self.LED_INVERT     = False   # True to invert the signal (when using NPN transistor level shift)
        self.LED_CHANNEL    = 0       # set to '1' for GPIOs 13, 19, 41, 45 or 53
        self.LED_STRIP      = ws.WS2811_STRIP_GRB   # Strip type and colour ordering
        # Create NeoPixel object with appropriate configuration.
        self.strip = Adafruit_NeoPixel(self.LED_COUNT, self.LED_PIN, self.LED_FREQ_HZ, self.LED_DMA, self.LED_INVERT, self.LED_BRIGHTNESS, self.LED_CHANNEL, self.LED_STRIP)
        # Intialize the library (must be called once before other functions).
        self.strip.begin()

        # multiple for color 
        self.mul = 3

        # load led labels
        self.red = self.SetColor(rospy.get_param('~led_red'))
        self.purple = self.SetColor(rospy.get_param('~led_purple'))
        self.blue = self.SetColor(rospy.get_param('~led_blue'))
        self.green = self.SetColor(rospy.get_param('~led_green'))
        self.yellow = self.SetColor(rospy.get_param('~led_yellow'))
        self.orange = self.SetColor(rospy.get_param('~led_orange'))
        self.white = self.SetColor(rospy.get_param('~led_white'))
        self.off = self.SetColor(rospy.get_param('~led_off'))

        self.color = [self.off, self.white, self.orange, self.yellow, self.green, self.blue, self.purple, self.red]


        #Subscribers
        self.sub_led_classes = rospy.Subscriber("~led_classes", ClassLeds, self.cbLedClass, queue_size=1)
        self.sub_led = rospy.Subscriber("~led", Int32, self.cbLed, queue_size=1)

        # example of red light
        self.colorWipe(self.green)
        time.sleep(0.3)
        self.colorWipe(self.off)
        print ('Led Set Done!')


    def SetColor(self, color):
        #return Color(color[0] * self.mul, color[1] * self.mul, color[2] * self.mul)
        return Color(color[0], color[1], color[2])

    # Callback of led classes msg
    def cbLedClass(self, msg):
        print msg.classes_led
        for i in range(self.strip.numPixels()):
            self.strip.setPixelColor(i, self.color[msg.classes_led[i]])
        self.strip.show()

    def cbLed(self, msg):
        #self.colorWipe(self.color[color.data])
        for i in range(self.strip.numPixels()):
            if(i == msg.data):
                self.strip.setPixelColor(i, self.color[i])
            else:
                self.strip.setPixelColor(i, self.color[0])
        self.strip.show()

    # Define functions which animate LEDs in various ways.
    def colorWipe(self, color): #, wait_ms=50):
        """Wipe color across display a pixel at a time."""
        for i in range(self.strip.numPixels()):
            self.strip.setPixelColor(i, color)
        self.strip.show()
        #time.sleep(wait_ms/1000.0)

    def theaterChase(self, color, iterations=10):
        """Movie theater light style chaser animation."""
        for j in range(iterations):
            for q in range(3):
                for i in range(0, self.strip.numPixels(), 3):
                    self.strip.setPixelColor(i+q, color)
                self.strip.show()
            for i in range(0, self.strip.numPixels(), 3):
                self.strip.setPixelColor(i+q, 0)

    def wheel(pos):
        """Generate rainbow colors across 0-255 positions."""
        if pos < 85:
            return Color(pos * 3, 255 - pos * 3, 0)
        elif pos < 170:
            pos -= 85
            return Color(255 - pos * 3, 0, pos * 3)
        else:
            pos -= 170
            return Color(0, pos * 3, 255 - pos * 3)

    def rainbow(self, iterations=1):
        """Draw rainbow that fades across all pixels at once."""
        for j in range(256*iterations):
            for i in range(self.strip.numPixels()):
                self.strip.setPixelColor(i, wheel((i+j) & 255))
            self.strip.show()

    def rainbowCycle(self, iterations=5):
        """Draw rainbow that uniformly distributes itself across all pixels."""
        for j in range(256*iterations):
            for i in range(self.strip.numPixels()):
                self.strip.setPixelColor(i, wheel((int(i * 256 / self.strip.numPixels()) + j) & 255))
            self.strip.show()

    def theaterChaseRainbow(self):
        """Rainbow movie theater light style chaser animation."""
        for j in range(256):
            for q in range(3):
                for i in range(0, self.strip.numPixels(), 3):
                    self.strip.setPixelColor(i+q, wheel((i+j) % 255))
                self.strip.show()
            for i in range(0, self.strip.numPixels(), 3):
                self.strip.setPixelColor(i+q, 0)


# Main program logic follows:
if __name__ == '__main__':
    # Create NeoPixel object with appropriate configuration.
    #strip = Adafruit_NeoPixel(LED_COUNT, LED_PIN, LED_FREQ_HZ, LED_DMA, LED_INVERT, LED_BRIGHTNESS, LED_CHANNEL, LED_STRIP)
    # Intialize the library (must be called once before other functions).
    #strip.begin()
    
    rospy.init_node("gd_standerledni",anonymous=False)
    gd_standerledni = GdStanderledNi()
    rospy.spin()
    
    #neopixel = Neopixel()
    #print ('Press Ctrl-C to quit.')
    #while True:
        #print ('Color red')
        #neopixel.colorWipe(Color(10, 0, 0))
        #time.sleep(0.05)
        #print ('Color green')
        #neopixel.colorWipe(Color(0, 0, 10))
        #time.sleep(0.05)
    

        #print ('Color wipe animations.')
        #colorWipe(strip, Color(255, 0, 0))  # Red wipe
        #colorWipe(strip, Color(0, 255, 0))  # Blue wipe
        #colorWipe(strip, Color(0, 0, 255))  # Green wipe
        #print ('Theater chase animations.')
        #theaterChase(strip, Color(127, 127, 127))  # White theater chase
        #theaterChase(strip, Color(127,   0,   0))  # Red theater chase
        #theaterChase(strip, Color(  0,   0, 127))  # Blue theater chase
        #print ('Rainbow animations.')
        #rainbow(strip)
        #rainbowCycle(strip)
        #theaterChaseRainbow(strip)
