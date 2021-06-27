#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Quaternion, PoseStamped
import numpy as np

#############################################################################
class gps_tf:
#############################################################################

    #############################################################################
    def __init__(self):
    #############################################################################
        rospy.init_node("gps_tf")
        self.nodename = rospy.get_name()
        rospy.loginfo("-I- %s started" % self.nodename)  #10260
        
        self.latitude = 0
        self.longitude = 0
        self.quaternion = Quaternion(0,0,0,1)
                
        # subscriptions
        rospy.Subscriber("lat", Float32, self.latCallback)
        rospy.Subscriber("lon", Float32, self.lonCallback)
        rospy.Subscriber("quaternion", Quaternion, self.quatCallback)
        
        #Publishers
        self.gps_goal_fix = rospy.Publisher("gps_goal_fix", NavSatFix, queue_size = 10)
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
        self.gps_goal_pose.publish(Pose)
        self.local_xy_origin.publish(Pose)

        navsatfix = NavSatFix()
        navsatfix.header.stamp = rospy.Time.now()
        navsatfix.header.frame_id = 'map'
        navsatfix.status.status = 1
        navsatfix.status.service = 1
        navsatfix.latitude = self.latitude
        navsatfix.longitude = self.longitude
        navsatfix.altitude = 0
        self.gps_goal_fix.publish(navsatfix)
        

    #############################################################################
    def quatCallback(self, msg):
    #############################################################################
        qArray = [msg.x, msg.y, msg.z, msg.w]/np.linalg.norm([msg.x, msg.y, msg.z, msg.w])
        self.quaternion.x = qArray[0]
        self.quaternion.y = -qArray[1]
        self.quaternion.z = -qArray[2]
        self.quaternion.w = qArray[3]
    
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
    
    
   
