#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import NavSatFix
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
        
        #### parameters #######
        self.rate = rospy.get_param('~rate',10.0)  # the rate at which to publish the transform
        self.odom_frame_id = rospy.get_param('~odom_frame_id', 'odom') # the name of the odometry reference frame

        self.then = rospy.Time.now()
        self.t_delta = rospy.Duration(1.0/self.rate)
        self.t_next = rospy.Time.now() + self.t_delta
        
        self.latitude = Float32()
        self.longitude = Float32()
        self.quaternion = Quaternion()
                
        # subscriptions
        rospy.Subscriber("lat", Float32, self.latCallback)
        rospy.Subscriber("lon", Float32, self.lonCallback)
        rospy.Subscriber("qx", Float32, self.qxCallback)
        rospy.Subscriber("qy", Float32, self.qyCallback)
        rospy.Subscriber("qz", Float32, self.qzCallback)
        rospy.Subscriber("qw", Float32, self.qwCallback)
        
        #Publishers
        self.gps_goal_fix = rospy.Publisher("gps_goal_fix", NavSatFix, queue_size = 10)
        self.gps_goal_pose = rospy.Publisher("gps_goal_pose", PoseStamped, queue_size = 10)
        self.local_xy_origin = rospy.Publisher("local_xy_origin", PoseStamped, queue_size = 10)
        
    #############################################################################
    def spin(self):
    #############################################################################
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()
       
     
    #############################################################################
    def update(self):
    #############################################################################
        now = rospy.Time.now()
        if now > self.t_next:
            elapsed = now - self.then
            self.then = now
            elapsed = elapsed.to_sec()

            Pose = PoseStamped()
            Pose.header.stamp = now
            Pose.header.frame_id = 'map'
            Pose.pose.position.x = self.latitude
            Pose.pose.position.y = self.longitude
            Pose.pose.position.z = 0
            Pose.pose.orientation = self.quaternion
            self.gps_goal_pose.publish(Pose)
            self.local_xy_origin.publish(Pose)

            navsatfix = NavSatFix()
            navsatfix.header.stamp = now
            navsatfix.header.frame_id = 'map'
            navsatfix.status.status = 1
            navsatfix.status.service = 1
            navsatfix.latitude = self.latitude
            navsatfix.longitude = self.longitude
            navsatfix.altitude = 0
            self.gps_goal_fix.publish(navsatfix)
            

    #############################################################################
    def qxCallback(self, msg):
    #############################################################################
        self.quaternion.x = msg.data
    
    #############################################################################
    def qyCallback(self, msg):
    #############################################################################
        self.quaternion.y = msg.data
    
    #############################################################################
    def qzCallback(self, msg):
    #############################################################################
        self.quaternion.z = msg.data
    
    #############################################################################
    def qwCallback(self, msg):
    #############################################################################
        self.quaternion.w = msg.data
    
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
    
    
   
