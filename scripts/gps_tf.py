#!/usr/bin/env python

import serial
import rospy
from math import sin, cos
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
        
        self.latitude = 0
        self.longitude = 0
        self.quaternion = Quaternion(0,0,0,1)
        
        # serial port
        self.serialPort = serial.Serial(port = "/dev/ttyAMA1", baudrate=9600, bytesize=8, timeout=2)
                
        # subscriptions
        rospy.Subscriber("mag/heading", Float32, self.headingCallback)
                
        #Publishers
        self.gps_goal_fix = rospy.Publisher("gps_goal_fix", NavSatFix, queue_size = 10)
        self.gps_goal_pose = rospy.Publisher("gps_goal_pose", PoseStamped, queue_size = 10)
        self.local_xy_origin = rospy.Publisher("local_xy_origin", PoseStamped, queue_size = 10)
        
    #############################################################################
    def spin(self):
    #############################################################################
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.get_gps()
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
    def get_gps(self):
    #############################################################################
        if(self.serialPort.in_waiting > 0):
            a = self.serialPort.readline().decode('ascii', errors='replace')
            for line in a.split('\n') :
                if line.startswith( '$GPGGA' ) :
                    lat, _, lon = line.strip().split(',')[2:5]
                    self.latitude = round(float(lat)/100,6)
                    self.longitude = round(float(lon)/100,6)
        #rospy.loginfo("latitude = %f - longitude = %f",self.latitude, self.longitude)     

    #############################################################################
    def headingCallback(self, msg):
    #############################################################################
        self.quaternion.z = sin(msg.data / 2)
        self.quaternion.w = cos(msg.data / 2)
        
#############################################################################
#############################################################################
if __name__ == '__main__':
    """ main """
    gpsTf = gps_tf()
    gpsTf.spin()