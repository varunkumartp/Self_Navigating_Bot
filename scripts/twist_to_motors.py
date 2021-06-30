#!/usr/bin/env python

# twist_to_motors - converts a twist message to motor commands.  Needed for navigation stack

import math
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist 

#############################################################
class TwistToMotors():
#############################################################

    #############################################################
    def __init__(self):
    #############################################################
        rospy.init_node("twist_to_motors")
        self.nodename = rospy.get_name()
        rospy.loginfo("-I- %s started" % self.nodename)  #10260
        
        self.w = rospy.get_param("~base_width", 0.24)
    
        self.pub_lmotor = rospy.Publisher('lwheel_vtarget', Float32,queue_size=10)
        self.pub_rmotor = rospy.Publisher('rwheel_vtarget', Float32,queue_size=10)
        rospy.Subscriber('/cmd_vel', Twist, self.twistCallback)
    
        self.dia = rospy.get_param("~diametre",0.066)
        self.rate = rospy.get_param("~rate", 50)
        self.timeout_ticks = rospy.get_param("~timeout_ticks", 2)
        self.left = 0
        self.right = 0
        
    #############################################################
    def spin(self):
    #############################################################
    
        r = rospy.Rate(self.rate)
        idle = rospy.Rate(10)
        then = rospy.Time.now()
        self.ticks_since_target = self.timeout_ticks
    
        ###### main loop  ######
        while not rospy.is_shutdown():
        
            while not rospy.is_shutdown() and self.ticks_since_target < self.timeout_ticks:
                self.spinOnce()
                r.sleep()
            idle.sleep()
                
    #############################################################
    def spinOnce(self):
    #############################################################
    
        # dx = (l + r) / 2
        # dr = (r - l) / w
        self.right = self.dx + self.dr * self.w / 2 
        self.left = self.dx - self.dr * self.w / 2
        #rospy.loginfo("velocity = (%f, %f)", self.left, self.right)
        self.left = self.left * 60 / ( self.dia * math.pi )
        self.right = self.right * 60 / ( self.dia * math.pi )
        #rospy.loginfo("RPM = (%d, %d)", self.left, self.right)       
        self.pub_lmotor.publish(self.left)
        self.pub_rmotor.publish(self.right)
        self.ticks_since_target += 1

    #############################################################
    def twistCallback(self,msg):
    #############################################################
        # rospy.loginfo("-D- twistCallback: %s" % str(msg))
        self.ticks_since_target = 0
        self.dx = msg.linear.x
        self.dr = msg.angular.z
        self.dy = msg.linear.y
    
#############################################################
#############################################################
if __name__ == '__main__':
    """ main """
    twistToMotors = TwistToMotors()
    twistToMotors.spin()
