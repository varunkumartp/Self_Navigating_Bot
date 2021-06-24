#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16, Bool

#############################################################
class PhoneToMotors:
#############################################################
        
    #############################################################
    def __init__(self):
    #############################################################
        rospy.init_node("phone_to_motors")
        self.nodename = rospy.get_name()
        rospy.loginfo("-I- %s started" % self.nodename)  #10260
        
        # Publishers
        self.to_motor = rospy.Publisher("intake", Int16, queue_size = 10)
        
        # Subscribers
        rospy.Subscriber('F', Bool, self.f_callback)
        rospy.Subscriber('B', Bool, self.b_callback)
        rospy.Subscriber('L', Bool, self.l_callback)
        rospy.Subscriber('R', Bool, self.r_callback)
        
    #############################################################
    def spin(self):
    #############################################################
        while not rospy.is_shutdown():
            rospy.spin()
            
    #############################################################
    def send(self, data, boolean):
    #############################################################
        if boolean:
            self.to_motor.publish(data)
        else:
            self.to_motor.publish(0)
            
    #############################################################
    def f_callback(self,msg):
    #############################################################
        self.send(1, msg.data)
        
    #############################################################
    def l_callback(self,msg):
    #############################################################
        self.send(4, msg.data)
        
    #############################################################
    def r_callback(self,msg):
    #############################################################
        self.send(3, msg.data)
        
    #############################################################
    def b_callback(self,msg):
    #############################################################
        self.send(2, msg.data)
        
#############################################################
#############################################################
if __name__ == '__main__':
    PTM = PhoneToMotors()
    PTM.spin()