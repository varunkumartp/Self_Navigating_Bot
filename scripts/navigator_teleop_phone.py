#!/usr/bin/env python

# Used to control the bot using the ROS-mobile app availbale in play store

import rospy
from std_msgs.msg import String, Bool

#############################################################
class navigator_teleop_phone():
#############################################################
    
    #############################################################
    def __init__(self):
    #############################################################
        rospy.init_node("navigator_teleop_phone")
        self.nodename = rospy.get_name()
        rospy.loginfo("-I- %s started" % self.nodename)  #10260
        
        self.l = False
        self.r = False
        self.f = False
        self.b = False
        self.s = False
        
        # subscribers
        rospy.Subscriber("app/S", Bool, self.S_callBack)
        rospy.Subscriber("app/L", Bool, self.L_callBack)
        rospy.Subscriber("app/R", Bool, self.R_callBack)
        rospy.Subscriber("app/F", Bool, self.F_callBack)
        rospy.Subscriber("app/B", Bool, self.B_callBack)
        
        # publishers
        self.controlPub = rospy.Publisher("motors/control", String, queue_size = 10)
        
    #############################################################
    def spin(self):
    #############################################################
        while not rospy.is_shutdown():
            rospy.spin()

    #############################################################
    def S_callBack(self,msg):
    #############################################################
        self.controlPub.publish("S")
        
    #############################################################
    def F_callBack(self,msg):
    #############################################################
        self.controlPub.publish("S" if not msg.data else "F")
        
    #############################################################
    def B_callBack(self,msg):
    #############################################################
        self.controlPub.publish("S" if not msg.data else "B")
        
    #############################################################
    def L_callBack(self,msg):
    #############################################################
        self.controlPub.publish("S" if not msg.data else "L")
        
    #############################################################
    def R_callBack(self,msg):
    #############################################################
        self.controlPub.publish("S" if not msg.data else "R")
        
#############################################################
#############################################################
if __name__ == "__main__":
    teleop = navigator_teleop_phone()
    teleop.spin()          