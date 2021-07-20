#!/usr/bin/env python

# Used to control the bot using the ROS-mobile app availbale in play store
# Buttons must be added in the ROS-mobile app. Check ros_mobile_button_layout picture in images folder for button layout reference

import rospy
from std_msgs.msg import Int16, Bool

#############################################################
class navigator_teleop_phone():
#############################################################
    
    #############################################################
    def __init__(self):
    #############################################################
        rospy.init_node("navigator_teleop_phone")
        self.nodename = rospy.get_name()
        rospy.loginfo("-I- %s started" % self.nodename)  #10260
        
        # subscribers
        rospy.Subscriber("app/S", Bool, self.S_callBack)
        rospy.Subscriber("app/L", Bool, self.L_callBack)
        rospy.Subscriber("app/R", Bool, self.R_callBack)
        rospy.Subscriber("app/F", Bool, self.F_callBack)
        rospy.Subscriber("app/B", Bool, self.B_callBack)
        
        # publishers
        self.left_pwm_pub = rospy.Publisher("motors/pwm/lwheel", Int16, queue_size = 10)
        self.right_pwm_pub = rospy.Publisher("motors/pwm/rwheel", Int16, queue_size = 10)
        
    #############################################################
    def spin(self):
    #############################################################
        while not rospy.is_shutdown():
            rospy.spin()

    #############################################################
    def send(self, l, r):
    #############################################################
        self.left_pwm_pub.publish(l)
        self.right_pwm_pub.publish(r)
    
    #############################################################
    def S_callBack(self,msg):
    #############################################################
        self.send(0, 0)
        
    #############################################################
    def F_callBack(self,msg):
    #############################################################
        if not msg.data:
            self.send(0, 0)
        else:
            self.send(128, 128)
            
    #############################################################
    def B_callBack(self,msg):
    #############################################################
        if not msg.data:
            self.send(0, 0)
        else:
            self.send(-128, -128)
        
    #############################################################
    def L_callBack(self,msg):
    #############################################################
        if not msg.data:
            self.send(0, 0)
        else:
            self.send(-128, 128)
        
    #############################################################
    def R_callBack(self,msg):
    #############################################################
        if not msg.data:
            self.send(0, 0)
        else:
            self.send(128, -128)
        
#############################################################
#############################################################
if __name__ == "__main__":
    teleop = navigator_teleop_phone()
    teleop.spin()          