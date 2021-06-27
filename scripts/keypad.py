#!/usr/bin/env python

import RPi.GPIO as GPIO
import time
import rospy
from std_msgs.msg import String, Bool 

#############################################################################
class keypad:
#############################################################################
        
    #############################################################################
    def __init__(self):
    #############################################################################
        rospy.init_node('keypad')
        nodename = rospy.get_name()
        rospy.loginfo("%s started" % nodename)
        self.rate = rospy.get_param("~rate", 10)
        
        # GPIO
        # row pins
        self.L1 = 5
        self.L2 = 6
        self.L3 = 13
        self.L4 = 19

        # column pins
        self.C1 = 12
        self.C2 = 16
        self.C3 = 20
        
        self.keypadPressed = -1
        self.input = ''
        self.otp = '1234'
        
        # Setup GPIO
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        # GPIO output
        GPIO.setup(self.L1, GPIO.OUT)
        GPIO.setup(self.L2, GPIO.OUT)
        GPIO.setup(self.L3, GPIO.OUT)
        GPIO.setup(self.L4, GPIO.OUT)
        # Use the internal pull-down resistors
        GPIO.setup(self.C1, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(self.C2, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(self.C3, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        # Detect the rising edges on the column lines of the
        # keypad. This way, we can detect if the user presses
        # a button when we send a pulse.
        GPIO.add_event_detect(self.C1, GPIO.RISING, callback=self.keypadCallback)
        GPIO.add_event_detect(self.C2, GPIO.RISING, callback=self.keypadCallback)
        GPIO.add_event_detect(self.C3, GPIO.RISING, callback=self.keypadCallback)

        # publisher
        self.statPub = rospy.Publisher('status', Bool, queue_size = 10)
        
        # subscriber
        rospy.Subscriber('otp',String, self.otpCallback)
        
    #############################################################################
    def otpCallback(self,msg):
    #############################################################################
        self.otp = msg.data
        
    #############################################################################
    def keypadCallback(self,channel):
    #############################################################################
        if self.keypadPressed == -1:
            self.keypadPressed = channel

    #############################################################################
    def setAllLines(self,state):
    #############################################################################
        GPIO.output(self.L1, state)
        GPIO.output(self.L2, state)
        GPIO.output(self.L3, state)
        GPIO.output(self.L4, state)

    #############################################################################
    def checkSpecialKeys(self):
    #############################################################################
        pressed = False

        # check if * is pressed
        GPIO.output(self.L4, GPIO.HIGH)
        if (GPIO.input(self.C1) == 1):
            pressed = True
        GPIO.output(self.L4, GPIO.LOW)
        
        # check if # is pressed 
        GPIO.output(self.L4, GPIO.HIGH)
        if (not pressed and GPIO.input(self.C3) == 1):
            self.statPub(self.input == self.otp)
            pressed = True
        GPIO.output(self.L4, GPIO.LOW)

        if pressed:
            self.input = ""

        return pressed

    #############################################################################
    def readLine(self, line, characters):
    #############################################################################
        GPIO.output(line, GPIO.HIGH)
        if(GPIO.input(self.C1) == 1):
            self.input = self.input + characters[0]
        if(GPIO.input(self.C2) == 1):
            self.input = self.input + characters[1]
        if(GPIO.input(self.C3) == 1):
            self.input = self.input + characters[2]
        GPIO.output(line, GPIO.LOW)
    
    #############################################################################
    def spin(self):
    #############################################################################
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.check()
            r.sleep()
            
    #############################################################################
    def check(self):
    #############################################################################
        # If a button was previously pressed,
        # check, whether the user has released it yet
        if self.keypadPressed != -1:
            self.setAllLines(GPIO.HIGH)
            if GPIO.input(self.keypadPressed) == 0:
                self.keypadPressed = -1
            else:
                time.sleep(0.1)
        # Otherwise, just read the input
        else:
            if not self.checkSpecialKeys():
                self.readLine(self.L1, ["1","2","3"])
                self.readLine(self.L2, ["4","5","6"])
                self.readLine(self.L3, ["7","8","9"])
                self.readLine(self.L4, ["*","0","#"])
            time.sleep(0.1)
            
#############################################################################
#############################################################################                
if __name__ == '__main__':
    k = keypad()
    k.spin()

    