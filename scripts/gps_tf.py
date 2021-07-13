#!/usr/bin/env python

import serial
import rospy
from math import sin, cos
from std_msgs.msg import Float32
from geometry_msgs.msg import Quaternion, PoseStamped
 
#############################################################################
class gps_tf:
#############################################################################

    #############################################################################
    def __init__(self):
    #############################################################################
        rospy.init_node("gps_tf")
        self.nodename = rospy.get_name()
        rospy.loginfo("-I- %s started" % self.nodename)  #10260
        
        self.latitude = 12.940574645996094
        self.longitude = 77.60774230957031
        self.quaternion = Quaternion(0,0,0,1)
                       
        # subscriptions
        rospy.Subscriber("imu/yaw", Float32, self.headingCallback)
        rospy.Subscriber("coordinates/lat", Float32, self.latCallback)
        rospy.Subscriber("coordinates/lon", Float32, self.lonCallback)
                
        #Publishers
        self.gps_goal_pose = rospy.Publisher("gps_goal_pose", PoseStamped, queue_size = 10)
        self.local_xy_origin = rospy.Publisher("local_xy_origin", PoseStamped, queue_size = 10)
        
    #############################################################################
    def spin(self):
    #############################################################################
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()
     
    #############################################################################
    def update(self):
    #############################################################################
        Pose = PoseStamped()
        Pose.header.stamp = rospy.Time.now()
        Pose.header.frame_id = 'map'
        Pose.pose.position.x = self.latitude
        Pose.pose.position.y = self.longitude
        Pose.pose.position.z = 0
        Pose.pose.orientation = self.quaternion
        #self.gps_goal_pose.publish(Pose)
        self.local_xy_origin.publish(Pose)

    #############################################################################
    def headingCallback(self, msg):
    #############################################################################
        self.quaternion.z = sin(msg.data / 2)
        self.quaternion.w = cos(msg.data / 2)
        
    #############################################################################
    def latCallback(self, msg):
    #############################################################################
        self.latitude = msg.data
        
    #############################################################################
    def lonCallback(self, msg):
    #############################################################################
        self.longitude = msg.data
            
#############################################################################
#############################################################################
if __name__ == '__main__':
    """ main """
    gpsTf = gps_tf()
    gpsTf.spin()